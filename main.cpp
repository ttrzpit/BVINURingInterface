// =============================================================================
// main.cpp — BVI NURing Interface
//
// Entry point. Initializes all system components and runs the main control loop.
//
// Startup sequence:
//   1. Load config.yaml (falls back to hardcoded defaults if missing)
//   2. Construct all handlers (each validates its own setup)
//   3. Start background threads (camera capture, serial receive)
//   4. Show the ArUco marker grid on the touchscreen
//   5. Enter the main loop
//
// Main loop (runs at ~camera frame rate, throttled by CameraHandler):
//   a. Poll keyboard  — keeps OpenCV windows responsive (must be called each iter)
//   b. Get camera frame — non-blocking copy from the camera thread's latest frame
//   c. Detect ArUco markers in the grayscale channel
//   d. Read touch state from the touchscreen
//   e. Update the operator display (camera frame + overlays)
//   f. Send/receive serial data  — stub; uncomment when Teensy is connected
//
// Shutdown:
//   ESC key or SIGINT (Ctrl-C) → sets g_running = false → clean thread join
// =============================================================================

#include <chrono>
#include <csignal>
#include <iostream>
#include <string>

#include "ArucoHandler.h"
#include "Cal1Handler.h"
#include "Cal2Handler.h"
#include "Cal3Handler.h"
#include "CameraHandler.h"
#include "Colors.h"
#include "Config.h"
#include "ControllerHandler.h"
#include "DisplayHandler.h"
#include "FittsTaskHandler.h"
#include "GestureHandler.h"
#include "KeyboardHandler.h"
#include "PacketTypes.h"
#include "PretensionHandler.h"
#include "SerialHandler.h"
#include "TouchHandler.h"

// Global shutdown flag — written by SIGINT handler, read by the main loop
static volatile bool g_running = true;

void signalHandler(int signum) {
    std::cout << "\nMain: Signal " << signum << " received — shutting down cleanly.\n";
    g_running = false;
}

int main() {
    std::signal(SIGINT, signalHandler);

    // OpenCV's internal parallel_for_ thread pool is not safe to dispatch into
    // concurrently from multiple application threads (camera capture thread +
    // ArUco detection thread both call into it) — this caused an intermittent
    // heap corruption / double-free crash inside cv::findContours during ArUco
    // candidate detection. The app already parallelizes via its own handler
    // threads, so disable OpenCV's internal threading entirely.
    cv::setNumThreads(1);

    std::cout << "\n=== BVI NURing Interface ===\n\n";

    // ---- Load configuration -------------------------------------------------
    Config cfg;
    if (!cfg.load("config.yaml")) {
        std::cout << "Main: config.yaml not found — using built-in defaults.\n";
    }

    // ---- Construct handlers -------------------------------------------------
    // Each handler receives only the sub-config it needs (not the whole Config).
    // This keeps dependencies explicit and prevents handlers from reading each
    // other's configuration accidentally.

    CameraHandler camera(cfg.camera);

    ArucoHandler aruco(cfg.arucoMarker,
                       cfg.arucoDetector,
                       cfg.arucoDisplay,
                       cfg.arucoCalGrid,
                       cfg.touchscreen,
                       cfg.camera.cameraMatrix,
                       cfg.camera.distCoeffs);

    TouchHandler touch(cfg.touchscreen);
    Cal3Handler cal3(cfg.touchscreen, cfg.camera, cfg.arucoCalGrid, cfg.cal3);
    FittsTaskHandler fitts(cfg.arucoDisplay, cfg.touchscreen, cfg.camera);

    ControllerHandler controller(cfg.controllerGains);
    PretensionHandler pretension(controller);
    Cal1Handler cal1(controller, cfg.cal1);
    Cal2Handler cal2(controller, cal1.GetBoundary(), cfg.cal2);
    GestureHandler gesture(controller, cfg.gesture);

    DisplayHandler display(cfg.display,
                           cv::Point2i(static_cast<int>(cfg.camera.cx),
                                       static_cast<int>(cfg.camera.cy)),
                           cfg.telemetry,
                           cfg.controllerPanel);

    SerialHandler serial(cfg.serial);

    // ---- Start background threads -------------------------------------------
    camera.start();  // Camera grab loop runs on its own thread
    aruco.Start();   // ArUco detection runs on its own thread
    serial.start();  // Serial receive loop runs on its own thread (stub)

    // The ArUco grid image is generated at startup (inside the ArucoHandler
    // constructor) but the window is NOT shown until the system enters CALIBRATING.

    std::cout << "\nMain: Running. Press ESC to quit.\n";
    std::cout << "Main: See keyboard_reference.md for the full key command reference.\n\n";

    KeyboardHandler keyboard;

    double lastFrameTimestamp = -1.0;
    SystemState prevState = SystemState::IDLE;
    InputState prevInputState = InputState::IDLE;
    int prevFittsTarget = 0;
    bool cal3CompletionHandled = false;
    bool cal2ProfileApplied = false;      ///< Set once Cal2's K(theta) has been applied to the controller
    bool readyRequested = false;          ///< RobotState ladder: 'E' sets true, 'e' clears it
    bool prevPretensionComplete = false;  ///< Detects pretension.IsComplete() false->true
    PcToTeensyPacket lastTxPkt = {};      // Pending TX values updated each frame — sent by TX thread at 200 Hz

    // Controller — dt is measured between loop iterations, independent of camera frame rate
    double lastControllerSecs = cv::getTickCount() / cv::getTickFrequency();

    // Motor test state — set by testA/testB/testC commands, cleared after 1 s
    using Clock = std::chrono::steady_clock;
    bool motorTestActive = false;
    char motorTestMotor = 'A';
    uint16_t motorTestPwm = 2047;
    Clock::time_point motorTestStart;

    // ---- Main loop ----------------------------------------------------------
    while (g_running) {
        // a. Keyboard — PollKey() must be called every iteration to keep all
        //    OpenCV windows responsive. The result is fed to KeyboardHandler
        //    which manages multi-character commands and the quit flag.
        int key = display.PollKey();
        keyboard.ProcessKey(key);
        const KeyboardState& kb = keyboard.GetState();
        if (kb.quitRequested) break;

        // Serial connect/disconnect toggle ('S' key)
        if (kb.pendingSerialAction == SerialAction::TOGGLE) {
            if (serial.IsConnected()) {
                serial.Disconnect();
                keyboard.SetExternalStatus("PC-->Teensy connection disabled.");
            } else {
                serial.Connect();
                keyboard.SetExternalStatus(serial.IsConnected()
                                               ? "PC-->Teensy connection established."
                                               : "Connect failed, check port.");
            }
            keyboard.ClearSerialAction();
        }

        // RobotState ladder — global 'e'/'E' keys (work regardless of menu state)
        if (kb.pendingRobotStateRequest == RobotStateRequest::GO_IDLE) {
            readyRequested = false;
            keyboard.SetExternalStatus("Returning to IDLE — guidance/ready disabled.");
            keyboard.ClearRobotStateRequest();
        } else if (kb.pendingRobotStateRequest == RobotStateRequest::GO_READY) {
            if (pretension.IsComplete()) {
                readyRequested = true;
                keyboard.SetExternalStatus("READY enabled — preload tension active.");
            } else {
                keyboard.SetExternalStatus("Cannot enter READY: pretensioning not complete.");
            }
            keyboard.ClearRobotStateRequest();
        }

        // Stiffness gain toggle ('k' key) — enables/disables applying K(theta)
        // in place of gain_kP, for performance comparisons. Has no effect
        // until Cal2 has produced a valid profile (ControllerHandler::
        // HasStiffnessProfile()).
        if (kb.pendingStiffnessGainToggle) {
            bool enabled = !controller.IsStiffnessGainEnabled();
            controller.SetStiffnessGainEnabled(enabled);
            if (!controller.HasStiffnessProfile()) {
                keyboard.SetExternalStatus(std::string("Stiffness gain ") + (enabled ? "enabled" : "disabled") +
                                           " (no K(theta) profile yet — using gain_kP).");
            } else {
                keyboard.SetExternalStatus(std::string("Stiffness gain ") + (enabled ? "enabled" : "disabled") + ".");
            }
            keyboard.ClearStiffnessGainToggle();
        }

        // Handle system state transitions
        if (kb.systemState != prevState) {
            // Close whichever grid was open in the previous state
            if (prevState == SystemState::CAL3) {
                aruco.SetCalibrationGridVisible(false);
                aruco.SetCalibrationDetection(false);  // Restore DICT_4X4_50
            } else {
                aruco.SetGridVisible(false);
            }

            if (kb.systemState == SystemState::CALIBRATING) {
                // General calibration state — Fitts-style marker grid
                aruco.SetGridVisible(true);
            } else if (kb.systemState == SystemState::CAL3) {
                // Cal3: dense calibration grid for camera-to-fingertip offset measurement
                aruco.SetCalibrationGridVisible(true);
                aruco.SetCalibrationDetection(true);  // Switch to DICT_4X4_250
            } else if (kb.systemState == SystemState::FITTS) {
                // FITTS starts with a blank white screen — first target appears on 'r'
                aruco.ShowBlankTouchscreen();
                fitts.Reset();
            }
            // IDLE and any other state — window already closed above

            if (kb.systemState == SystemState::CAL3) {
                cal3.Reset();
                cal3CompletionHandled = false;
            }
            if (kb.systemState == SystemState::PRETENSION) {
                pretension.Reset();
            }
            // TENSION_ADJUST — standalone tension adjustment (no guided
            // sequence): enable output + manual tension mode directly.
            if (kb.systemState == SystemState::TENSION_ADJUST) {
                controller.SetOutputEnabled(true);
                controller.SetManualTensionMode(true);
            }
            // Safety net: if PRETENSION is aborted before completion (e.g.
            // 'grave', or 'T' back to the menu), make sure PWM output and
            // manual tension mode are disabled again. If pretensioning
            // completed normally, leave output enabled — the RobotState
            // ladder below takes over and holds preload tension (READY).
            if (prevState == SystemState::PRETENSION && kb.systemState != SystemState::PRETENSION && !pretension.IsComplete()) {
                controller.SetOutputEnabled(false);
                controller.SetManualTensionMode(false);
            }
            if (prevState == SystemState::TENSION_ADJUST && kb.systemState != SystemState::TENSION_ADJUST) {
                // Capture the manually-adjusted tensions as the new preload
                // baseline, same as step-by-step pretensioning step 4/4.
                controller.SetPreloadTensions();
                controller.SetOutputEnabled(false);
                controller.SetManualTensionMode(false);
            }
            if (prevState == SystemState::FITTS) prevFittsTarget = 0;
            prevState = kb.systemState;
        }

        // CAL_ROM (AROM calibration) entry/exit — a sub-state of CALIBRATING,
        // so it isn't seen by the systemState transition block above.
        if (kb.inputState != prevInputState) {
            if (kb.inputState == InputState::CAL_ROM) {
                cal1.Reset();
                controller.SetOutputEnabled(true);
                controller.SetManualTensionMode(false);
            }
            if (prevInputState == InputState::CAL_ROM && kb.inputState != InputState::CAL_ROM) {
                controller.SetOutputEnabled(false);
            }

            // CAL_STI (stiffness calibration) entry/exit — requires Cal1's
            // AROM boundary; Cal2Handler::Reset() enters BLOCKED otherwise.
            if (kb.inputState == InputState::CAL_STI) {
                cal2.Reset();
                cal2ProfileApplied = false;
                controller.SetOutputEnabled(true);
                controller.SetManualTensionMode(false);
                controller.SetCalibrationForceMode(true);
            }
            if (prevInputState == InputState::CAL_STI && kb.inputState != InputState::CAL_STI) {
                controller.SetOutputEnabled(false);
                controller.SetCalibrationForceMode(false);
            }

            prevInputState = kb.inputState;
        }

        // Current time, shared by CAL3, the controller, and pretensioning
        double nowSecs = cv::getTickCount() / cv::getTickFrequency();

        // b. Camera frame — check if the camera thread has produced a NEW frame
        //    by comparing timestamps. Without this, the non-blocking getLatestFrame()
        //    would return the same frame repeatedly and the loop would spin at
        //    hundreds of Hz with no useful work done.
        CameraFrame frame = camera.getLatestFrame();
        bool isNewFrame = frame.ready && (frame.timestamp != lastFrameTimestamp);
        if (isNewFrame) {
            lastFrameTimestamp = frame.timestamp;
        }

        // c. ArUco detection — only submit when there is genuinely a new frame.
        //    GetLatestDetection() is always called so the main loop always has
        //    the freshest result, even if a new frame hasn't arrived yet.
        if (isNewFrame) {
            aruco.SubmitFrame(frame.gray);
        }
        std::vector<DetectedMarker> markers = aruco.GetLatestDetection();

        // d. Touch state — drains pending X11 events, returns current state
        TouchState touchState = touch.getLatestTouch();

        // e. CAL3 — update touch-collection state machine now that both
        //    markers and touchState are available
        if (kb.systemState == SystemState::CAL3) {
            cal3.Update(touchState, markers, nowSecs);
            keyboard.SetExternalStatus(cal3.GetStatus());

            if (cal3.IsComplete() && !cal3CompletionHandled) {
                cal3CompletionHandled = true;
                display.SetCal3State(true, cal3.GetFinalOffset(), cal3.GetRollRef());
                aruco.SetCalibrationGridVisible(false);
                aruco.SetCalibrationDetection(false);
            }
        }

        // CAL_ROM — AROM calibration: record the virtual fingertip position
        // while preload tension is held (see CAL_ROM entry handling above).
        if (kb.inputState == InputState::CAL_ROM) {
            cal1.Update(nowSecs);
            keyboard.SetExternalStatus(cal1.GetStatus());
        }

        // CAL_STI — stiffness calibration: drive the per-heading force ramp
        // and apply the commanded force via calibration force mode (see
        // CAL_STI entry handling above). Once complete, push K(theta) into
        // the controller's stiffness profile.
        if (kb.inputState == InputState::CAL_STI) {
            cal2.Update(nowSecs);
            cv::Point2f calForce = cal2.GetCommandedForce();
            controller.SetCalibrationForce(calForce.x, calForce.y);
            keyboard.SetExternalStatus(cal2.GetStatus());

            if (cal2.IsComplete() && !cal2ProfileApplied) {
                controller.SetStiffnessProfile(cal2.GetStiffnessProfile());
                cal2ProfileApplied = true;
            }
        }

        // In FITTS state, show the selected target marker whenever it changes
        if (kb.systemState == SystemState::FITTS &&
            kb.fittsTargetId != prevFittsTarget && kb.fittsTargetId > 0) {
            aruco.ShowSingleMarker(kb.fittsTargetId);
            fitts.OnNewTarget(kb.fittsTargetId);
            prevFittsTarget = kb.fittsTargetId;
            controller.ResetRamp(nowSecs);
        }

        // f. Controller — runs every loop iteration so dt tracks wall-clock
        //    time, independent of camera frame rate.
        float dt = static_cast<float>(nowSecs - lastControllerSecs);
        lastControllerSecs = nowSecs;

        // Active target marker (set via kb.activeTagId during FITTS) drives
        // guidance: the marker's camera-relative position IS the position
        // error (pos_target - pos_virtual), since the camera is rigid with
        // the virtual fingertip — when the marker is centered under the
        // camera, the error is zero and the target has been reached.
        const DetectedMarker *activeMarker = nullptr;
        if (kb.activeTagId > 0) {
            for (const auto &m : markers) {
                if (m.id == kb.activeTagId) {
                    activeMarker = &m;
                    break;
                }
            }
        }

        cv::Point2f pos_target     = controller.GetVirtualPosition();
        bool        isTargetActive = false;
        if (activeMarker) {
            pos_target.x += activeMarker->positionMm.x;
            pos_target.y += activeMarker->positionMm.y;
            isTargetActive = true;
        }

        TeensyToPcPacket rxPkt = {};
        bool hasRxPkt = serial.GetLatestPacket(rxPkt);
        if (hasRxPkt) {
            controller.Update(rxPkt, pos_target, isTargetActive, nowSecs, dt);
        }

        // Pretensioning — guided state machine (see PretensionHandler.h)
        if (kb.systemState == SystemState::PRETENSION) {
            pretension.Update(nowSecs);

            // Step 3/4 just began (ZERO->TENSION) — default to TEN_SEL_ALL so
            // [+/-] adjusts all three motors immediately, no [a/b/c/d] needed.
            if (pretension.ConsumeTensionPhaseEntered()) {
                keyboard.SetInputState(InputState::TEN_SEL_ALL);
            }

            // Step 3/4 — live tension adjustments from the Tension interface,
            // applied only while ControllerHandler is in manual tension mode.
            if (kb.pendingTensionAdjust.active) {
                if (controller.IsManualTensionMode()) {
                    if (kb.pendingTensionAdjust.isAbsolute) {
                        controller.SetManualTension(kb.pendingTensionAdjust.motor,
                                                    kb.pendingTensionAdjust.valueN);
                    } else {
                        controller.AdjustManualTension(kb.pendingTensionAdjust.motor,
                                                       kb.pendingTensionAdjust.deltaN);
                    }
                }
                keyboard.ClearTensionAdjust();
            }

            if (kb.pendingPretensionAdvance) {
                pretension.Advance(rxPkt, nowSecs);
                keyboard.ClearPretensionAdvance();
            }
            keyboard.SetExternalStatus(pretension.GetStatus());
        }

        // Standalone tension adjustment — manual tension mode only, no
        // guided unspool/zero/home-recording steps (see TENSION_ADJUST entry
        // handling above).
        if (kb.systemState == SystemState::TENSION_ADJUST) {
            if (kb.pendingTensionAdjust.active) {
                if (kb.pendingTensionAdjust.isAbsolute) {
                    controller.SetManualTension(kb.pendingTensionAdjust.motor,
                                                kb.pendingTensionAdjust.valueN);
                } else {
                    controller.AdjustManualTension(kb.pendingTensionAdjust.motor,
                                                   kb.pendingTensionAdjust.deltaN);
                }
                keyboard.ClearTensionAdjust();
            }
            keyboard.SetExternalStatus(pretension.GetTensionAdjustStatus());
        }

        // Direction-dependent gain tuning ('G' key) — adjusts gainTune_A/B/C,
        // an extra K(theta)-style boost added to kP_effective (Stage 1) on
        // top of K(theta)/gain_kP. Works alongside normal operation; does not
        // gate output or manual tension mode.
        if (kb.inputState == InputState::GAIN_ALL || kb.inputState == InputState::GAIN_A ||
            kb.inputState == InputState::GAIN_B   || kb.inputState == InputState::GAIN_C) {
            if (kb.pendingGainAdjust.active) {
                controller.AdjustGainTune(kb.pendingGainAdjust.motor, kb.pendingGainAdjust.deltaGain);
                keyboard.ClearGainAdjust();
            }
            keyboard.SetExternalStatus(controller.GetGainTuneStatus());
        }

        // Auto-request READY the moment the guided pretensioning sequence
        // finishes, so tension is held without needing 'E' pressed.
        bool pretensionComplete = pretension.IsComplete();
        if (pretensionComplete && !prevPretensionComplete) {
            readyRequested = true;
        }
        prevPretensionComplete = pretensionComplete;

        // RobotState ladder — derive PWM-output policy from serial
        // connection + tensioning completeness + (future) guidance enable.
        // Independent of PRETENSION/TENSION_ADJUST/CAL_ROM, which manage
        // controller output/manual-tension mode directly while active.
        bool guidanceEnabled = false;  // placeholder — guidance [NOT YET IMPLEMENTED]

        bool inOverrideMode = (kb.systemState == SystemState::PRETENSION ||
                               kb.systemState == SystemState::TENSION_ADJUST ||
                               kb.inputState == InputState::CAL_ROM ||
                               kb.inputState == InputState::CAL_STI);

        RobotState robotState;
        if (!serial.IsConnected()) {
            robotState = RobotState::DISCONNECTED;
        } else if (guidanceEnabled) {
            robotState = RobotState::GUIDING;
        } else if (readyRequested && pretension.IsComplete()) {
            robotState = RobotState::READY;
        } else {
            robotState = RobotState::IDLE;
        }

        if (!inOverrideMode) {
            bool holdPreload = (robotState == RobotState::READY || robotState == RobotState::GUIDING);
            controller.SetOutputEnabled(holdPreload);
            controller.SetManualTensionMode(false);
        }

        // Flick/confirm gesture detection — armed only while READY (not
        // GUIDING/IDLE/etc); Reset() clears the state machine on every other
        // RobotState so stale velocity history can't fire a gesture right
        // after entering READY.
        if (robotState == RobotState::READY) {
            GestureEvent gestureEvent = gesture.Update(nowSecs);

            // In FITTS, a flick steps the active target marker up/down,
            // clamped to [1, 45] (45-marker grid). CONFIRM (circle) is
            // detected and shown but not yet bound to an action.
            if (kb.systemState == SystemState::FITTS) {
                if (gestureEvent == GestureEvent::FLICK_UP) {
                    keyboard.SetFittsTargetId(kb.fittsTargetId + 1);
                } else if (gestureEvent == GestureEvent::FLICK_DOWN) {
                    keyboard.SetFittsTargetId(kb.fittsTargetId - 1);
                }
            }
        } else {
            gesture.Reset();
        }

        // g. Serial — update the pending TX packet each new camera frame.
        //    The TX thread sends it independently at 200 Hz; packet_index is
        //    managed by the TX thread and does not need to be set here.
        if (kb.pendingMotorTest.active) {
            motorTestActive = true;
            motorTestMotor = kb.pendingMotorTest.motor;
            motorTestPwm = kb.pendingMotorTest.pwm;
            motorTestStart = Clock::now();
            keyboard.ClearMotorTest();
        }

        if (isNewFrame) {
            // Reflect the RobotState ladder to the Teensy: READY/GUIDING hold
            // preload (or full) tension, everything else is plain IDLE.
            lastTxPkt.state = (robotState == RobotState::READY || robotState == RobotState::GUIDING)
                                  ? static_cast<uint8_t>(PcState::READY)
                                  : static_cast<uint8_t>(PcState::IDLE);
            lastTxPkt.pwm_A = 2047;
            lastTxPkt.pwm_B = 2047;
            lastTxPkt.pwm_C = 2047;

            // Controller output — only forwarded once PretensionHandler has
            // enabled it (TENSION step); otherwise PWM stays at 2047 (off).
            if (controller.IsOutputEnabled()) {
                lastTxPkt.pwm_A = controller.GetPwmA();
                lastTxPkt.pwm_B = controller.GetPwmB();
                lastTxPkt.pwm_C = controller.GetPwmC();
            }

            if (motorTestActive) {
                double elapsed = std::chrono::duration<double>(Clock::now() - motorTestStart).count();
                if (elapsed < 1.0) {
                    if (motorTestMotor == 'A')
                        lastTxPkt.pwm_A = motorTestPwm;
                    else if (motorTestMotor == 'B')
                        lastTxPkt.pwm_B = motorTestPwm;
                    else if (motorTestMotor == 'C')
                        lastTxPkt.pwm_C = motorTestPwm;
                    else if (motorTestMotor == 'D') {
                        lastTxPkt.pwm_A = motorTestPwm;
                        lastTxPkt.pwm_B = motorTestPwm;
                        lastTxPkt.pwm_C = motorTestPwm;
                    }
                } else {
                    motorTestActive = false;
                    keyboard.SetExternalStatus("Motor test done — back to idle.");
                }
            }

            // Pretensioning — one-shot zero-encoder command (ZERO step)
            if (pretension.ShouldSendZeroCommand()) {
                lastTxPkt.state = static_cast<uint8_t>(PcState::ZERO_ENC);
            }

            serial.SetPendingTx(lastTxPkt);
        }

        // Assemble serial state for the display — always up to date even when
        // the panel only refreshes at 10 Hz.
        SerialState serialSt;
        serialSt.isConnected = serial.IsConnected();
        serialSt.txFrequencyHz = serial.GetTxFrequency();
        serialSt.lastTx = lastTxPkt;
        serialSt.lastTx.packet_index = serial.GetLastSentIndex();  // TX thread owns this — never set by main
        serialSt.hasRx = serial.GetLatestPacket(serialSt.lastRx);
        serialSt.robotState = robotState;

        // h. Fitts task — virtual fingertip cursor + touch error overlay,
        //    updated when in FITTS mode.
        if (isNewFrame) {
            if (kb.systemState == SystemState::FITTS) {
                fitts.Update(markers, touchState, cal3.IsComplete(),
                             cal3.GetFinalOffset(), cal3.GetRollRef());
                display.SetVirtualFingertip(fitts.HasVirtualFingertip(), fitts.GetVirtualFingertipPx());
                aruco.SetFittsOverlay(fitts.HasTouchSample(), fitts.GetTouchScreenPx(),
                                      fitts.GetErrorLine1(), fitts.GetErrorLine2());
            } else {
                display.SetVirtualFingertip(false);
                aruco.SetFittsOverlay(false, {}, "", "");
            }
        }

        // i. Operator display + telemetry — only refresh on a new camera frame
        if (isNewFrame) {
            display.SetControllerTelemetry(controller.GetTelemetry());
            display.SetCal1State(kb.inputState == InputState::CAL_ROM && !cal1.IsComplete(),
                                 cal1.GetSamples(), cal1.GetBoundary());
            display.SetGestureIndicator(gesture.IsIndicatorActive(nowSecs), gesture.GetLastGesture());
            display.Update(frame.undistorted, markers, touchState, kb, serialSt);
        }
    }

    // ---- Clean shutdown -----------------------------------------------------
    std::cout << "\nMain: Shutting down...\n";

    aruco.Stop();
    camera.stop();
    serial.stop();
    cv::destroyAllWindows();

    std::cout << "Main: Done.\n";
    return 0;
}
