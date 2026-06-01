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

#include <csignal>
#include <iostream>

#include "ArucoHandler.h"
#include "Cal3Handler.h"
#include "CameraHandler.h"
#include "Colors.h"
#include "Config.h"
#include "DisplayHandler.h"
#include "KeyboardHandler.h"
#include "PacketTypes.h"
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

    CameraHandler  camera(cfg.camera);

    ArucoHandler   aruco(cfg.arucoDetect,
                         cfg.arucoDetector,
                         cfg.arucoDisplay,
                         cfg.touchscreen,
                         cfg.camera.cameraMatrix,
                         cfg.camera.distCoeffs);

    TouchHandler   touch(cfg.touchscreen);
    Cal3Handler    cal3(cfg.touchscreen, cfg.camera, cfg.arucoDisplay);

    DisplayHandler display(cfg.display,
                           cv::Point2i(static_cast<int>(cfg.camera.cx),
                                       static_cast<int>(cfg.camera.cy)),
                           cfg.telemetry,
                           cfg.controllerPanel);

    SerialHandler  serial(cfg.serial);

    // ---- Start background threads -------------------------------------------
    camera.start();    // Camera grab loop runs on its own thread
    aruco.Start();     // ArUco detection runs on its own thread
    serial.start();    // Serial receive loop runs on its own thread (stub)

    // The ArUco grid image is generated at startup (inside the ArucoHandler
    // constructor) but the window is NOT shown until the system enters CALIBRATING.

    std::cout << "\nMain: Running. Press ESC to quit.\n";
    std::cout << "Main: Commands — 'cal' = calibrate, 'idle' = idle, 'fitts' = Fitts\n\n";

    KeyboardHandler keyboard;

    double           lastFrameTimestamp = -1.0;
    SystemState      prevState         = SystemState::IDLE;
    int              prevFittsTarget   = 0;
    PcToTeensyPacket lastTxPkt         = {};   // Pending TX values updated each frame — sent by TX thread at 200 Hz

    // ---- Main loop ----------------------------------------------------------
    while (g_running) {

        // a. Keyboard — PollKey() must be called every iteration to keep all
        //    OpenCV windows responsive. The result is fed to KeyboardHandler
        //    which manages multi-character commands and the quit flag.
        int key = display.PollKey();
        keyboard.ProcessKey(key);
        const KeyboardState& kb = keyboard.GetState();
        if (kb.quitRequested) break;

        // Serial connect/disconnect on demand
        if (kb.pendingSerialAction == SerialAction::CONNECT) {
            serial.Connect();
            keyboard.SetExternalStatus(serial.IsConnected() ? "Teensy connected." : "Connect failed — check port.");
            keyboard.ClearSerialAction();
        } else if (kb.pendingSerialAction == SerialAction::DISCONNECT) {
            serial.Disconnect();
            keyboard.SetExternalStatus("Teensy disconnected.");
            keyboard.ClearSerialAction();
        }

        // Handle system state transitions
        if (kb.systemState != prevState) {
            // Grid is visible in CALIBRATING, CAL3, and FITTS states
            bool showGrid = (kb.systemState == SystemState::CALIBRATING ||
                             kb.systemState == SystemState::CAL3         ||
                             kb.systemState == SystemState::FITTS);
            aruco.SetGridVisible(showGrid);

            if (kb.systemState == SystemState::CAL3) {
                cal3.Reset();   // Fresh start each time CAL3 is entered
            }
            if (prevState == SystemState::FITTS) prevFittsTarget = 0;
            prevState = kb.systemState;
        }

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
            double nowSecs = cv::getTickCount() / cv::getTickFrequency();
            cal3.Update(touchState, markers, nowSecs);
            keyboard.SetExternalStatus(cal3.GetStatus());
        }

        // In FITTS state, show the selected target marker whenever it changes
        if (kb.systemState == SystemState::FITTS &&
            kb.fittsTargetId != prevFittsTarget && kb.fittsTargetId > 0) {
            aruco.ShowSingleMarker(kb.fittsTargetId);
            prevFittsTarget = kb.fittsTargetId;
        }

        // g. Serial — update the pending TX packet each new camera frame.
        //    The TX thread sends it independently at 200 Hz; packet_index is
        //    managed by the TX thread and does not need to be set here.
        //    PWM defaults to 2047 (no power) until the controller is implemented.
        if (isNewFrame) {
            lastTxPkt.state = static_cast<uint8_t>(PcState::IDLE);  // TODO: map kb.systemState
            lastTxPkt.pwm_A = 2047;
            lastTxPkt.pwm_B = 2047;
            lastTxPkt.pwm_C = 2047;
            serial.SetPendingTx(lastTxPkt);
        }

        // Assemble serial state for the display — always up to date even when
        // the panel only refreshes at 10 Hz.
        SerialState serialSt;
        serialSt.isConnected   = serial.IsConnected();
        serialSt.txFrequencyHz = serial.GetTxFrequency();
        serialSt.lastTx        = lastTxPkt;
        serialSt.hasRx         = serial.GetLatestPacket(serialSt.lastRx);

        // h. Operator display + telemetry — only refresh on a new camera frame
        if (isNewFrame) {
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
