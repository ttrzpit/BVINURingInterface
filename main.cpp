// =============================================================================
// main.cpp - BVI NURing Interface
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
//   a. Poll keyboard  - keeps OpenCV windows responsive (must be called each iter)
//   b. Get camera frame - non-blocking copy from the camera thread's latest frame
//   c. Detect ArUco markers in the grayscale channel
//   d. Read touch state from the touchscreen
//   e. Update the operator display (camera frame + overlays)
//   f. Send/receive serial data  - stub; uncomment when Teensy is connected
//
// Shutdown:
//   ESC key or SIGINT (Ctrl-C) → sets g_running = false → clean thread join
// =============================================================================

#include <array>
#include <chrono>
#include <climits>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <unistd.h>
#include <unordered_set>
#include <vector>

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
#include "TrialLogger.h"

// Global shutdown flag - written by SIGINT handler, read by the main loop
static volatile bool g_running = true;

void signalHandler( int signum ) {
    std::cout << "\nMain: Signal " << signum << " received - shutting down cleanly.\n";
    g_running = false;
}

// Resolve <executable_dir>/../logging so trial logs always land in the project's
// logging/ directory (the sibling of build/), regardless of the working
// directory the binary was launched from. TrialLogger's "../logging" default is
// relative to the launch CWD, so running from the project root instead of build/
// silently redirected logs one level too high (Code/logging vs
// BVINURingInterface/logging). Anchoring to the executable removes that ambiguity.
static std::string ResolveProjectLoggingDir() {
    char    buf[PATH_MAX];
    ssize_t n = ::readlink( "/proc/self/exe", buf, sizeof( buf ) - 1 );
    if ( n <= 0 ) return "../logging";    // fallback: original relative default
    buf[n] = '\0';

    std::error_code       ec;
    std::filesystem::path exeDir = std::filesystem::path( buf ).parent_path();
    std::filesystem::path logDir = std::filesystem::weakly_canonical( exeDir / ".." / "logging", ec );
    if ( ec ) logDir = exeDir / ".." / "logging";    // keep the un-normalized path on error
    return logDir.string();
}

int main() {
    std::signal( SIGINT, signalHandler );

    // OpenCV's internal parallel_for_ thread pool is not safe to dispatch into
    // concurrently from multiple application threads (camera capture thread +
    // ArUco detection thread both call into it) - this caused an intermittent
    // heap corruption / double-free crash inside cv::findContours during ArUco
    // candidate detection. The app already parallelizes via its own handler
    // threads, so disable OpenCV's internal threading entirely.
    cv::setNumThreads( 1 );

    std::cout << "\n=== BVI NURing Interface ===\n\n";

    // ---- Load configuration -------------------------------------------------
    Config cfg;
    if ( !cfg.load( "config.yaml" ) ) {
        std::cout << "Main: config.yaml not found - using built-in defaults.\n";
    }

    // Trial logs go to <executable_dir>/../logging, independent of launch CWD.
    const std::string loggingDir = ResolveProjectLoggingDir();
    std::cout << "Main: trial logs -> " << loggingDir << "\n";

    // ---- Construct handlers -------------------------------------------------
    // Each handler receives only the sub-config it needs (not the whole Config).
    // This keeps dependencies explicit and prevents handlers from reading each
    // other's configuration accidentally.

    CameraHandler camera( cfg.camera );

    ArucoHandler aruco( cfg.arucoMarker,
                        cfg.arucoDetector,
                        cfg.arucoDisplay,
                        cfg.arucoCalGrid,
                        cfg.fittsBoard,
                        cfg.touchscreen,
                        cfg.camera.cameraMatrix,
                        cfg.camera.distCoeffs );

    TouchHandler     touch( cfg.touchscreen );
    Cal3Handler      cal3( cfg.touchscreen, cfg.camera, cfg.arucoCalGrid, cfg.cal3 );
    FittsTaskHandler fitts( cfg.fittsBoard, cfg.touchscreen, cfg.camera );
    TrialLogger      trialLogger( loggingDir );

    ControllerHandler controller( cfg.controllerGains );
    PretensionHandler pretension( controller );
    Cal1Handler       cal1( controller, cfg.cal1 );
    Cal2Handler       cal2( controller, cal1.GetBoundary(), cfg.cal2 );
    GestureHandler    gesture( controller, cfg.gesture );

    DisplayHandler display( cfg.display,
                            cv::Point2i( static_cast<int>( cfg.camera.cx ),
                                         static_cast<int>( cfg.camera.cy ) ),
                            cfg.telemetry,
                            cfg.controllerPanel );

    SerialHandler serial( cfg.serial );

    // ---- Start background threads -------------------------------------------
    camera.start();    // Camera grab loop runs on its own thread
    aruco.Start();     // ArUco detection runs on its own thread
    serial.start();    // Serial receive loop runs on its own thread (stub)

    // The ArUco grid image is generated at startup (inside the ArucoHandler
    // constructor) but the window is NOT shown until the system enters CALIBRATING.

    std::cout << "\nMain: Running. Press ESC to quit.\n";
    std::cout << "Main: See keyboard_reference.md for the full key command reference.\n\n";

    KeyboardHandler keyboard;
    // Fitts targets are the fine-marker band of the multi-scale board.
    keyboard.SetFittsTargetRange( aruco.GetFittsTargetIdMin(), aruco.GetFittsTargetIdMax() );
    // Manual entry ('m' key) can also reach coarse markers for testing.
    keyboard.SetFittsBoardMaxId( aruco.GetFittsBoardMaxId() );

    double lastFrameTimestamp = -1.0;

    // Recent camera frames, newest at the back - lets the operator display
    // show the exact frame the latest marker detection was computed from,
    // so the marker overlay never drifts relative to the image underneath.
    std::deque<CameraFrame> frameHistory;
    constexpr size_t        kFrameHistoryMax = 12;

    SystemState prevState = SystemState::IDLE;
    InputState  prevInputState = InputState::IDLE;
    int         prevFittsTarget = 0;

    // Board-pose fallback target (when the active marker isn't directly detected).
    // Recomputed only on new frames (solvePnP is the expensive path) and reused
    // between frames; invalidated as soon as the marker is directly re-acquired.
    bool                       fbTargetValid = false;
    cv::Point3f                fbTargetPosMm = {};
    float                      fbTargetRoll = 0.0f;
    std::array<cv::Point2f, 4> fbCorners = {};

    // Trial logging + distance-stratified target selection state.
    std::mt19937 targetRng{ std::random_device{}() };
    int          distanceBandCursor = 0;    // cycles 0..numDistanceBands-1
    bool         prevLogTouched = false;
    bool         primeLoggingAfterUserId = false;    // 'L' with no user ID set: prime once the ID is entered
    int          prevActiveTagId = 0;                ///< Detects kb.activeTagId changes -> ramps guidance force on new targets

    // Last-known camera-side target-circle position (persists when activeMarker
    // leaves the camera frame so the circle doesn't flicker off momentarily).
    cv::Point2i lastTargetCirclePx = {};
    int         lastTargetCircleRadiusPx = 0;
    bool        haveLastTargetCircle = false;

    // Active-target outline frozen at the trial-ending touch, drawn as a magenta
    // reference box until the next target is selected (fitts.HasTouchSample()).
    std::array<cv::Point2f, 4> touchedTargetCorners = {};
    bool                       touchedTargetBoxValid = false;
    bool                       prevFittsTouchSample = false;

    bool cal3CompletionHandled = false;
    bool cal2ProfileApplied = false;       ///< Set once Cal2's K(theta) has been applied to the controller
    bool cal1CompletionHandled = false;    ///< Set once Cal1 (AROM) completion has returned the system to IDLE

    // Manual random-target debug pool ('r' cycles these IDs, no repeats, then
    // falls back to the normal picker). Refreshed on every FITTS entry.
    std::vector<int> randomPoolRemaining;
    bool             randomPoolExhaustedNotified = false;
    // Memory of which markers a random 'r' pick has already used this Fitts
    // sequence, so the same marker is never pulled twice. Cleared on every 'F'
    // (FITTS entry below); auto-restarts once every selectable target is used.
    std::unordered_set<int> usedRandomTargets;
    PcToTeensyPacket        lastTxPkt = {};    // Pending TX values updated each frame - sent by TX thread at 200 Hz

    // Controller - dt is measured between loop iterations, independent of camera frame rate
    double lastControllerSecs = cv::getTickCount() / cv::getTickFrequency();

    // Motor test state - set by testA/testB/testC commands, cleared after 1 s
    using Clock = std::chrono::steady_clock;
    bool              motorTestActive = false;
    char              motorTestMotor = 'A';
    uint16_t          motorTestPwm = 2047;
    Clock::time_point motorTestStart;

    // ---- Main loop ----------------------------------------------------------
    while ( g_running ) {
        // Stall probe: time each iteration. camera.getLatestFrame() only ever
        // returns the newest frame, so any main-thread stall silently drops
        // every frame captured during it - which is what punches the gaps in
        // the trial log. Warn when an iteration runs long enough to have lost
        // frames (>25 ms ≈ 2+ frames at 90 fps) so the blocking step can be
        // pinned down.
        const Clock::time_point iterStart = Clock::now();

        // a. Keyboard - PollKey() must be called every iteration to keep all
        //    OpenCV windows responsive. The result is fed to KeyboardHandler
        //    which manages multi-character commands and the quit flag.
        int key = display.PollKey();
        // Gate Fitts entry ('F') on all three calibrations being complete - if
        // not, ProcessKey diverts to the (p)roceed/(r)eturn confirmation prompt.
        keyboard.SetCalibrationsComplete( cal1.IsComplete() && cal2.IsComplete() && cal3.IsComplete() );
        keyboard.ProcessKey( key );
        const KeyboardState& kb = keyboard.GetState();
        if ( kb.quitRequested ) break;

        // Serial connect/disconnect toggle ('S' key)
        if ( kb.pendingSerialAction == SerialAction::TOGGLE ) {
            if ( serial.IsConnected() ) {
                serial.Disconnect();
                keyboard.SetExternalStatus( "PC-->Teensy connection disabled." );
            } else {
                serial.Connect();
                keyboard.SetExternalStatus( serial.IsConnected()
                                                ? "PC-->Teensy connection established."
                                                : "Connect failed, check port." );
            }
            keyboard.ClearSerialAction();
        }

        // Guidance-output toggle - global 'e'/'E' keys (work regardless of menu
        // state). 'e' zeros the guidance force into Stage 2, so PWM output
        // falls back to preload/tension only; 'E' restores the guidance force.
        // RobotState::READY now follows pretension.IsComplete() directly.
        if ( kb.pendingRobotStateRequest == RobotStateRequest::GO_IDLE ) {
            controller.SetGuidanceOutputEnabled( false );
            keyboard.SetExternalStatus( "Amplifier output: tension only (guidance disabled)." );
            keyboard.ClearRobotStateRequest();
        } else if ( kb.pendingRobotStateRequest == RobotStateRequest::GO_READY ) {
            controller.SetGuidanceOutputEnabled( true );
            keyboard.SetExternalStatus( "Amplifier output: guidance enabled." );
            keyboard.ClearRobotStateRequest();
        }

        // Spacebar e-stop - single-key toggle of the guidance output. Engaged:
        // guidance force is zeroed so the amplifier holds tension PWM only;
        // released: full guidance output resumes. Same path as 'e'/'E'.
        if ( kb.pendingEStopToggle ) {
            const bool enable = !controller.IsGuidanceOutputEnabled();
            controller.SetGuidanceOutputEnabled( enable );
            keyboard.SetExternalStatus( enable ? "E-STOP released - full guidance output resumed."
                                               : "E-STOP engaged - tension PWM only." );
            keyboard.ClearEStopToggle();
        }

        // Stiffness gain toggle ('k' key) - enables/disables adding K(theta)
        // on top of the custom-tuned gain (gainTune), for performance
        // comparisons. Has no effect until Cal2 has produced a valid profile
        // (ControllerHandler::HasStiffnessProfile()).
        if ( kb.pendingStiffnessGainToggle ) {
            bool enabled = !controller.IsStiffnessGainEnabled();
            controller.SetStiffnessGainEnabled( enabled );
            if ( !controller.HasStiffnessProfile() ) {
                keyboard.SetExternalStatus( std::string( "Stiffness gain " ) + ( enabled ? "enabled" : "disabled" ) +
                                            " (no K(theta) profile yet, using gainTune only)." );
            } else {
                keyboard.SetExternalStatus( std::string( "Stiffness gain " ) + ( enabled ? "enabled" : "disabled" ) + "." );
            }
            keyboard.ClearStiffnessGainToggle();
        }

        // Handle system state transitions
        if ( kb.systemState != prevState ) {
            // Close whichever grid was open in the previous state
            if ( prevState == SystemState::CAL3 ) {
                aruco.SetCalibrationGridVisible( false );
                aruco.SetCalibrationDetection( false );    // Restore DICT_4X4_50
            } else if ( prevState == SystemState::FITTS ) {
                aruco.SetFittsBoardVisible( false );
                aruco.SetFittsBoardDetection( false );    // Restore DICT_4X4_50
            } else {
                aruco.SetGridVisible( false );
            }

            if ( kb.systemState == SystemState::CALIBRATING ) {
                // General calibration state - Fitts-style marker grid
                aruco.SetGridVisible( true );
            } else if ( kb.systemState == SystemState::CAL3 ) {
                // Cal3: dense calibration grid for camera-to-fingertip offset measurement
                aruco.SetCalibrationGridVisible( true );
                aruco.SetCalibrationDetection( true );    // Switch to DICT_4X4_250
            } else if ( kb.systemState == SystemState::FITTS ) {
                // FITTS: persistent multi-scale board (coarse perimeter + fine
                // grid). The board is always shown; the active target is marked
                // by the target-offset circle once selected via 'r'.
                aruco.SetFittsBoardDetection( true );    // DICT_4X4_250, per-ID sizing
                aruco.SetFittsBoardVisible( true );
                fitts.Reset();
                // Refresh the manual random-target debug pool for this session.
                randomPoolRemaining = cfg.accuracyTrials.randomPool;
                randomPoolExhaustedNotified = false;
                // Clear the random-target memory so a fresh Fitts sequence can
                // reuse every marker exactly once before any repeats.
                usedRandomTargets.clear();
            }
            // IDLE and any other state - window already closed above

            if ( kb.systemState == SystemState::CAL3 ) {
                cal3.Reset();
                cal3CompletionHandled = false;
            }
            if ( kb.systemState == SystemState::PRETENSION ) {
                pretension.Reset();
            }
            // TENSION_ADJUST - standalone tension adjustment (no guided
            // sequence): enable output + manual tension mode directly.
            if ( kb.systemState == SystemState::TENSION_ADJUST ) {
                controller.SetOutputEnabled( true );
                controller.SetManualTensionMode( true );
            }
            // Safety net: if PRETENSION is aborted before completion (e.g.
            // 'grave', or 'T' back to the menu), make sure PWM output and
            // manual tension mode are disabled again. If pretensioning
            // completed normally, leave output enabled - the RobotState
            // ladder below takes over and holds preload tension (READY).
            if ( prevState == SystemState::PRETENSION && kb.systemState != SystemState::PRETENSION && !pretension.IsComplete() ) {
                controller.SetOutputEnabled( false );
                controller.SetManualTensionMode( false );
            }
            if ( prevState == SystemState::TENSION_ADJUST && kb.systemState != SystemState::TENSION_ADJUST ) {
                // Capture the manually-adjusted tensions as the new preload
                // baseline, same as step-by-step pretensioning step 4/4.
                controller.SetPreloadTensions();
                controller.SetOutputEnabled( false );
                controller.SetManualTensionMode( false );
            }
            if ( prevState == SystemState::FITTS ) {
                prevFittsTarget = 0;
                prevActiveTagId = 0;
                trialLogger.Cancel();    // drop any in-progress capture on FITTS exit
            }
            prevState = kb.systemState;
        }

        // CAL_ROM (AROM calibration) entry/exit - a sub-state of CALIBRATING,
        // so it isn't seen by the systemState transition block above. The
        // gain-tuning overlay (GAIN_ALL/A/B/C, 'G') is skipped entirely here
        // so it can't trigger CAL_ROM/CAL_STI exit logic while tuning gains
        // mid-calibration.
        if ( !IsGainTuneInputState( kb.inputState ) && kb.inputState != prevInputState ) {
            if ( kb.inputState == InputState::CAL_ROM ) {
                cal1.Reset();
                cal1CompletionHandled = false;
                controller.SetOutputEnabled( true );
                controller.SetManualTensionMode( false );
            }
            if ( prevInputState == InputState::CAL_ROM && kb.inputState != InputState::CAL_ROM ) {
                controller.SetOutputEnabled( false );
            }

            // CAL_STI (stiffness calibration) entry/exit - requires Cal1's
            // AROM boundary; Cal2Handler::Reset() enters BLOCKED otherwise.
            if ( kb.inputState == InputState::CAL_STI ) {
                cal2.Reset();
                cal2ProfileApplied = false;
                controller.SetOutputEnabled( true );
                controller.SetManualTensionMode( false );
                controller.SetCalibrationForceMode( true );
            }
            if ( prevInputState == InputState::CAL_STI && kb.inputState != InputState::CAL_STI ) {
                controller.SetOutputEnabled( false );
                controller.SetCalibrationForceMode( false );
            }

            prevInputState = kb.inputState;
        }

        // Current time, shared by CAL3, the controller, and pretensioning
        double nowSecs = cv::getTickCount() / cv::getTickFrequency();

        // b. Camera frame - check if the camera thread has produced a NEW frame
        //    by comparing timestamps. Without this, the non-blocking getLatestFrame()
        //    would return the same frame repeatedly and the loop would spin at
        //    hundreds of Hz with no useful work done.
        CameraFrame frame = camera.getLatestFrame();
        bool        isNewFrame = frame.ready && ( frame.timestamp != lastFrameTimestamp );
        if ( isNewFrame ) {
            lastFrameTimestamp = frame.timestamp;

            frameHistory.push_back( frame );
            if ( frameHistory.size() > kFrameHistoryMax ) {
                frameHistory.pop_front();
            }
        }

        // c. ArUco detection - only submit when there is genuinely a new frame.
        //    GetLatestDetection() is always called so the main loop always has
        //    the freshest result, even if a new frame hasn't arrived yet.
        if ( isNewFrame ) {
            aruco.SubmitFrame( frame.gray, frame.timestamp );
        }
        std::vector<DetectedMarker> markers = aruco.GetLatestDetection();

        // d. Touch state - drains pending X11 events, returns current state
        TouchState touchState = touch.getLatestTouch();

        // e. CAL3 - update touch-collection state machine now that both
        //    markers and touchState are available
        if ( kb.systemState == SystemState::CAL3 ) {
            cal3.Update( touchState, markers, nowSecs );
            keyboard.SetExternalStatus( cal3.GetStatus() );

            if ( cal3.IsComplete() && !cal3CompletionHandled ) {
                cal3CompletionHandled = true;
                display.SetCal3State( true, cal3.GetFinalOffset(), cal3.GetRollRef() );
                aruco.SetCalibrationGridVisible( false );
                aruco.SetCalibrationDetection( false );
                // Multi-step process finished - return to the default IDLE state
                // and wait for the next command.
                keyboard.SetInputState( InputState::IDLE );
            }
        }

        // CAL_ROM - AROM calibration: record the virtual fingertip position
        // while preload tension is held (see CAL_ROM entry handling above).
        if ( kb.inputState == InputState::CAL_ROM ) {
            cal1.Update( nowSecs );
            keyboard.SetExternalStatus( cal1.GetStatus() );

            if ( cal1.IsComplete() && !cal1CompletionHandled ) {
                cal1CompletionHandled = true;
                // Multi-step process finished - return to the default IDLE state.
                keyboard.SetInputState( InputState::IDLE );
            }
        }

        // CAL_STI - stiffness calibration: drive the per-heading force ramp
        // and apply the commanded force via calibration force mode (see
        // CAL_STI entry handling above). Once complete, push K(theta) into
        // the controller's stiffness profile.
        if ( kb.inputState == InputState::CAL_STI ) {
            cal2.Update( nowSecs );
            cv::Point2f calForce = cal2.GetCommandedForce();
            controller.SetCalibrationForce( calForce.x, calForce.y );
            keyboard.SetExternalStatus( cal2.GetStatus() );

            if ( cal2.IsComplete() && !cal2ProfileApplied ) {
                controller.SetStiffnessProfile( cal2.GetStiffnessProfile() );
                cal2ProfileApplied = true;
                // Multi-step process finished - return to the default IDLE state.
                keyboard.SetInputState( InputState::IDLE );
            }
        }

        // 'L' - system-level trial-logging toggle (works in any state). If no
        // user ID has been entered yet, jump to the user-ID prompt first and
        // prime automatically once the ID is set (see the watcher below).
        if ( kb.pendingLoggingToggle ) {
            if ( kb.activeUserId < 0 ) {
                keyboard.SetInputState( InputState::LOG_UID );
                keyboard.SetExternalStatus( "Enter user ID (000-999) to start logging..." );
                primeLoggingAfterUserId = true;
            } else {
                trialLogger.SetUserId( kb.activeUserId );
                trialLogger.TogglePrimed();
                keyboard.SetExternalStatus( trialLogger.IsPrimed() ? "Trial logging primed."
                                                                   : "Trial logging off." );
            }
            keyboard.ClearLoggingToggle();
        }

        // Auto-prime once an 'L'-triggered user-ID prompt has been completed.
        if ( primeLoggingAfterUserId ) {
            if ( kb.activeUserId >= 0 ) {
                primeLoggingAfterUserId = false;
                trialLogger.SetUserId( kb.activeUserId );
                if ( !trialLogger.IsPrimed() ) trialLogger.TogglePrimed();
                keyboard.SetExternalStatus( "User ID set - trial logging primed." );
            } else if ( kb.inputState != InputState::LOG_UID ) {
                primeLoggingAfterUserId = false;    // user left the prompt without setting an ID
            }
        }

        // Once the logging-prime flow has set the user ID, it lands in the LOG
        // state - a dead end with no commands of its own, so it would block the
        // IDLE-gated commands ('F', 'C', ...). Drop back to the default IDLE state
        // to wait for the next input, mirroring how the guided tensioning returns
        // to IDLE after its final step. (LOG_UID, the live user-ID prompt, is left
        // alone so the operator can still type; an 'L' armed mid-task never enters
        // LOG, so this won't disturb a running FITTS/calibration.)
        if ( kb.inputState == InputState::LOG ) {
            keyboard.SetInputState( InputState::IDLE );
        }

        // 'r' - distance-stratified random target. Bands of distance (from the
        // PREVIOUS target's position) are cycled for an even spread; a target is
        // picked uniformly within the current band, never repeating the previous.
        // The very first target after entering FITTS is uniform-random.
        if ( kb.pendingRandomTarget ) {
            const auto& sel = aruco.GetFittsSelectableTargetIds();
            if ( !sel.empty() ) {
                const int   prevId = kb.fittsTargetId;
                const float mmpp = cfg.touchscreen.mmPerPixel;
                int         nextId = sel[0];

                if ( !randomPoolRemaining.empty() ) {
                    // Manual debug pool: pick a random unused ID from the
                    // operator-supplied pool (config: accuracy_trials.random_pool)
                    // and remove it so it never repeats until the pool refreshes.
                    const int idx = std::uniform_int_distribution<int>( 0, ( int )randomPoolRemaining.size() - 1 )( targetRng );
                    nextId = randomPoolRemaining[idx];
                    randomPoolRemaining.erase( randomPoolRemaining.begin() + idx );
                } else {
                    // Pool empty: fall back to the normal whole-board picker. If
                    // a pool was configured and we have just used the last entry,
                    // announce the switch exactly once.
                    if ( !cfg.accuracyTrials.randomPool.empty() && !randomPoolExhaustedNotified ) {
                        std::cout << "Moving outside of random pool, selecting new random value" << std::endl;
                        randomPoolExhaustedNotified = true;
                    }

                    // Restrict to markers not yet used this sequence (the no-repeat
                    // memory). Once every selectable target has been used, restart
                    // the memory so the task keeps running.
                    std::vector<int> avail;
                    avail.reserve( sel.size() );
                    for ( int id : sel )
                        if ( !usedRandomTargets.count( id ) ) avail.push_back( id );
                    if ( avail.empty() ) {
                        std::cout << "All Fitts targets used - resetting random-target memory." << std::endl;
                        usedRandomTargets.clear();
                        avail = sel;
                    }

                    if ( prevId <= 0 || aruco.GetGridMarkerCenterPx( prevId ) == cv::Point2i{} ) {
                        nextId = avail[std::uniform_int_distribution<int>( 0, ( int )avail.size() - 1 )( targetRng )];
                    } else {
                        const cv::Point2i                  pPx = aruco.GetGridMarkerCenterPx( prevId );
                        float                              dmin = 1e9f, dmax = 0.0f;
                        std::vector<std::pair<int, float>> cand;
                        for ( int id : avail ) {
                            if ( id == prevId ) continue;    // never repeat the immediately previous target
                            const cv::Point2i cPx = aruco.GetGridMarkerCenterPx( id );
                            const float       dx = ( cPx.x - pPx.x ) * mmpp;
                            const float       dy = ( cPx.y - pPx.y ) * mmpp;
                            const float       d = std::sqrt( dx * dx + dy * dy );
                            cand.push_back( { id, d } );
                            dmin = std::min( dmin, d );
                            dmax = std::max( dmax, d );
                        }
                        if ( cand.empty() ) {
                            // Only unused target left was prevId itself - just take it.
                            nextId = avail[std::uniform_int_distribution<int>( 0, ( int )avail.size() - 1 )( targetRng )];
                        } else {
                            const int nBands = std::max( 1, cfg.fittsBoard.numDistanceBands );
                            const int band = distanceBandCursor % nBands;
                            distanceBandCursor = ( distanceBandCursor + 1 ) % nBands;
                            const float      w = ( dmax - dmin ) / nBands;
                            const float      lo = dmin + band * w;
                            const float      hi = ( band == nBands - 1 ) ? dmax + 1.0f : lo + w;
                            std::vector<int> inBand;
                            for ( auto& pr : cand )
                                if ( pr.second >= lo && pr.second <= hi ) inBand.push_back( pr.first );
                            if ( inBand.empty() )
                                for ( auto& pr : cand ) inBand.push_back( pr.first );    // fallback: any
                            if ( !inBand.empty() )
                                nextId = inBand[std::uniform_int_distribution<int>( 0, ( int )inBand.size() - 1 )( targetRng )];
                        }
                    }
                }

                // Record this pick so it isn't pulled again until the sequence is
                // restarted ('F') or the memory auto-resets on exhaustion. Covers
                // both the debug-pool and whole-board paths.
                usedRandomTargets.insert( nextId );

                keyboard.SetFittsTargetId( nextId );
                keyboard.SetExternalStatus( "Active marker set to " + std::to_string( nextId ) + "." );
                // If logging is primed, this 'r' also begins the trial capture.
                if ( trialLogger.IsPrimed() ) {
                    trialLogger.SetUserId( kb.activeUserId );
                    trialLogger.StartTrial( nextId );
                    // Header metadata: target centroid relative to screen centre
                    // [mm] (x right+, y down+) and the measured Cal3 fingertip
                    // offset (0,0,0 if Cal3 never ran).
                    const cv::Point2i tCenterPx = aruco.GetGridMarkerCenterPx( nextId );
                    const float       tScreenXmm = ( tCenterPx.x - cfg.touchscreen.width * 0.5f ) * cfg.touchscreen.mmPerPixel;
                    const float       tScreenYmm = ( tCenterPx.y - cfg.touchscreen.height * 0.5f ) * cfg.touchscreen.mmPerPixel;
                    const cv::Point3f ftOff = cal3.GetFinalOffset();
                    trialLogger.SetTrialMeta( tScreenXmm, tScreenYmm,
                                              ftOff.x, ftOff.y, ftOff.z, cal3.IsComplete() );

                    // Calibration metadata for offline reconstruction (MATLAB):
                    // the Cal1 AROM envelope spline control points and the Cal2
                    // stiffness measurements, plus the calibration headings.
                    const AromBoundary& arom = cal1.GetBoundary();
                    const auto          stiff = controller.GetStiffnessProfile();
                    trialLogger.SetCalibrationMeta(
                        std::vector<float>( CONSTANT_CALIBRATION_ANGLES_DEG,
                                            CONSTANT_CALIBRATION_ANGLES_DEG + CONSTANT_CALIBRATION_ANGLES_COUNT ),
                        arom.valid,
                        std::vector<float>( arom.theta.begin(), arom.theta.end() ),
                        std::vector<float>( arom.radius.begin(), arom.radius.end() ),
                        std::vector<float>( arom.accel.begin(), arom.accel.end() ),
                        controller.HasStiffnessProfile(),
                        std::vector<float>( stiff.begin(), stiff.end() ) );

                    // Sync prevLogTouched so a touch already in progress at trial
                    // start is not immediately detected as the finish rising edge.
                    prevLogTouched = touchState.isTouched;
                }
            }
            keyboard.ClearRandomTarget();
        }

        // In FITTS state, arm the new target whenever it changes. The board is
        // persistent (all markers stay shown); only the target-offset circle
        // and the fitts task state move to the new target marker.
        if ( kb.systemState == SystemState::FITTS &&
             kb.fittsTargetId != prevFittsTarget && kb.fittsTargetId > 0 ) {
            fitts.OnNewTarget( kb.fittsTargetId );
            prevFittsTarget = kb.fittsTargetId;
        }

        // Whenever the active target marker changes - however it was selected
        // (random 'r', manual +/-, etc.) - restart the guidance force ramp so
        // the sudden jump in position error doesn't snap the finger toward
        // the new target; force instead rises smoothly over ramp_duration_secs.
        if ( kb.activeTagId != prevActiveTagId ) {
            controller.ResetRamp( nowSecs );    // also resets the setpoint IIR filter
            prevActiveTagId = kb.activeTagId;
            haveLastTargetCircle = false;
            // Tell the detection thread which marker needs full 3D pose - it
            // skips pose for all others, keeping the dense board cheap.
            aruco.SetActiveTagId( kb.activeTagId );
        }

        // f. Controller - runs every loop iteration so dt tracks wall-clock
        //    time, independent of camera frame rate.
        float dt = static_cast<float>( nowSecs - lastControllerSecs );
        lastControllerSecs = nowSecs;

        // Active target marker (set via kb.activeTagId during FITTS) drives
        // guidance: the marker's camera-relative position IS the position
        // error (pos_target - pos_virtual), since the camera is rigid with
        // the virtual fingertip - when the marker is centered under the
        // camera, the error is zero and the target has been reached.
        //
        // The guidance target is offset "under" (toward larger image-Y) the
        // tag center by targetOffsetY - the Cal3 fingertip-offset Y value once
        // calibrated, or target.offset_default_mm before that.
        const DetectedMarker* activeMarker = nullptr;
        if ( kb.activeTagId > 0 ) {
            for ( const auto& m : markers ) {
                if ( m.id == kb.activeTagId ) {
                    activeMarker = &m;
                    break;
                }
            }
        }

        // Resolve the guidance target. Prefer the directly-detected active
        // marker (most accurate, especially up close). If it isn't visible -
        // too far for the fine target, or lost while veering off - fall back to
        // the board pose computed from whatever markers ARE visible (coarse
        // markers far away, neighbouring fine markers up close) plus the
        // target's known board location, so guidance keeps pulling back toward
        // the target instead of cutting out. The board-pose solve is the
        // expensive path, so it is recomputed only on new frames and reused.
        bool        haveTarget = false;
        cv::Point3f targetPosMm = {};
        float       targetRoll = 0.0f;
        if ( activeMarker ) {
            haveTarget = true;
            targetPosMm = activeMarker->positionMm;
            targetRoll = activeMarker->rollRad;
            fbTargetValid = false;    // direct lock re-acquired; drop stale fallback
        } else if ( kb.systemState == SystemState::FITTS && kb.activeTagId > 0 ) {
            if ( isNewFrame ) {
                fbTargetValid = fitts.EstimateTargetFromBoard(
                    markers, kb.activeTagId, fbTargetPosMm, fbTargetRoll, &fbCorners );
            }
            if ( fbTargetValid ) {
                haveTarget = true;
                targetPosMm = fbTargetPosMm;
                targetRoll = fbTargetRoll;
            }
        }

        // Once the participant has touched the screen for the current target,
        // guidance cues stop until a new target is loaded (fitts.OnNewTarget()
        // resets HasTouchSample() on the next 'r'/'m' target change).
        const bool guidanceSuppressedByTouch = ( kb.systemState == SystemState::FITTS && fitts.HasTouchSample() );

        // Hand the resolved target and calibration state to the controller.
        // pos_target, roll compensation, and the IIR setpoint filter are all
        // computed inside ControllerHandler::Update().
        controller.SetTarget(
            haveTarget,
            targetPosMm,
            targetRoll,
            cal3.IsComplete(),
            cal3.GetFinalOffset(),
            cal3.GetRollRef(),
            cfg.target.offsetDefaultMm,
            !guidanceSuppressedByTouch );

        // Feed the resolved target position to the operator telemetry panel so
        // the "Target Telemetry" readout tracks the target via the board-pose
        // estimate when the marker itself isn't directly detected.
        display.SetActiveTargetPosition( haveTarget, targetPosMm );

        // When guidance is running on the board-pose estimate (target marker not
        // directly detected), hand the projected outline to the operator view so
        // the green box / ID / guidance line still draw at the estimated spot.
        display.SetEstimatedActiveTarget( haveTarget && activeMarker == nullptr,
                                          kb.activeTagId, fbCorners );

        // Target marker centre in the camera image + its depth, for the
        // operator-view overlays. Use the directly-detected marker when present;
        // otherwise project the board-pose fallback position so the circle and
        // guiding dot still show while the target marker is dropped out.
        bool        haveTargetPx = false;
        cv::Point2i targetCenterPx = {};
        float       targetDepth = 0.0f;
        if ( activeMarker ) {
            haveTargetPx = true;
            targetCenterPx = activeMarker->centerPx;
            targetDepth = activeMarker->positionMm.z;
        } else if ( haveTarget && targetPosMm.z > 1e-3f ) {
            // positionMm is camera-frame Y-up; image Y points down, hence the
            // negation on the Y projection term.
            targetDepth = targetPosMm.z;
            targetCenterPx = cv::Point2i(
                static_cast<int>( std::round( cfg.camera.cx + cfg.camera.fx * targetPosMm.x / targetDepth ) ),
                static_cast<int>( std::round( cfg.camera.cy - cfg.camera.fy * targetPosMm.y / targetDepth ) ) );
            haveTargetPx = true;
        }

        // Operator-display cues for the new "fingerpad onto the marker centre"
        // target. The controller exposes the rolled cam->fingertip offset
        // (corrX, corrY) in camera-frame Y-up mm; intrinsics live here so the
        // pixel maths stays outside the controller.
        if ( haveTargetPx && targetDepth > 1e-3f ) {
            const auto  tgtOfs = controller.GetTargetOffsets();
            const float corrX_screen = tgtOfs.corrX;    // (R*d).x
            const float corrY_screen = tgtOfs.corrY;    // (R*d).y
            const float depth = targetDepth;

            int radiusPx = static_cast<int>( std::round(
                0.5 * ( cfg.camera.fx + cfg.camera.fy ) * cfg.target.radiusMm / depth ) );
            // Target circle = the fingerpad landing target = the marker centre.
            lastTargetCirclePx = targetCenterPx;
            lastTargetCircleRadiusPx = radiusPx;
            haveLastTargetCircle = true;

            // Green dot = guiding position ("virtual marker"): the image point the
            // marker centre must be steered onto so the fingerpad lands on it -
            // i.e. the current fingertip position = principal point + the rolled
            // offset R*d. (corrX, corrY) are camera-frame Y-up mm; image Y is down.
            cv::Point2i virtualTargetPx(
                static_cast<int>( std::round( cfg.camera.cx + cfg.camera.fx * corrX_screen / depth ) ),
                static_cast<int>( std::round( cfg.camera.cy - cfg.camera.fy * corrY_screen / depth ) ) );
            display.SetVirtualTarget( cal3.IsComplete(), virtualTargetPx );
        } else {
            display.SetVirtualTarget( false, {} );
        }

        // Circle color: red (cal3 calibrated offset) or gray (default offset,
        // not yet calibrated). Applied to both camera-side and touchscreen circles.
        cv::Scalar targetCircleColor = cal3.IsComplete() ? Colors::RedMd : Colors::GraMd;
        bool       circlesActive = ( kb.activeTagId > 0 );

        if ( circlesActive && haveLastTargetCircle ) {
            display.SetTargetCircle( true, lastTargetCirclePx, lastTargetCircleRadiusPx, targetCircleColor );
        } else {
            display.SetTargetCircle( false, {}, 0, targetCircleColor );
        }

        // Touchscreen target-offset circle: hollow circle at the calibrated
        // touch target location (marker grid-cell center + Cal3 offset) once
        // Cal3 is complete (red), or offset_default_mm below the marker center
        // before Cal3 completes (gray). Visible whenever an active target is
        // set and no touch has been recorded yet for this target.
        if ( kb.systemState == SystemState::FITTS && circlesActive && kb.fittsTargetId > 0 ) {
            // Target ring sits ON the marker centre now (the fingerpad's landing
            // target); the cal3 offset is applied in the guidance geometry, not
            // as a visible offset from the tag.
            cv::Point2i markerCenterPx = aruco.GetGridMarkerCenterPx( kb.fittsTargetId );
            int         radiusPx = static_cast<int>( std::round( cfg.target.radiusMm * cfg.touchscreen.pixelsPerMm ) );
            aruco.SetTargetOffsetCircle( true, markerCenterPx, radiusPx, targetCircleColor );
        } else {
            aruco.SetTargetOffsetCircle( false, {}, 0, targetCircleColor );
        }

        TeensyToPcPacket rxPkt = {};
        bool             hasRxPkt = serial.GetLatestPacket( rxPkt );
        if ( hasRxPkt ) {
            controller.Update( rxPkt, nowSecs, dt );
        }

        // Set home position ('Z' key) - records current encoder angles as the
        // neutral home pose, identical to what PretensionHandler does at step 3.
        if ( kb.pendingSetHomePosition ) {
            controller.SetHomePosition( rxPkt );
            keyboard.SetExternalStatus( "Home position recorded." );
            keyboard.ClearSetHomePosition();
        }

        // Pretensioning - guided state machine (see PretensionHandler.h)
        if ( kb.systemState == SystemState::PRETENSION ) {
            // Step 2/3 just began (UNSPOOL->TENSION) - default to TEN_SEL_ALL so
            // [+/-] adjusts all three motors immediately, no [a/b/c/d] needed.
            if ( pretension.ConsumeTensionPhaseEntered() ) {
                keyboard.SetInputState( InputState::TEN_SEL_ALL );
            }

            // Step 2/3 - live tension adjustments from the Tension interface,
            // applied only while ControllerHandler is in manual tension mode.
            if ( kb.pendingTensionAdjust.active ) {
                if ( controller.IsManualTensionMode() ) {
                    if ( kb.pendingTensionAdjust.isAbsolute ) {
                        controller.SetManualTension( kb.pendingTensionAdjust.motor,
                                                     kb.pendingTensionAdjust.valueN );
                    } else {
                        controller.AdjustManualTension( kb.pendingTensionAdjust.motor,
                                                        kb.pendingTensionAdjust.deltaN );
                    }
                }
                keyboard.ClearTensionAdjust();
            }

            if ( kb.pendingPretensionAdvance ) {
                pretension.Advance( rxPkt );
                keyboard.ClearPretensionAdvance();
                // Guided sequence finished (home recorded at step 3/3) - return
                // to the default IDLE state and wait for the next command. The
                // RobotState ladder keeps preload tension held (READY) since
                // pretension.IsComplete() stays true.
                if ( pretension.IsComplete() ) {
                    keyboard.SetInputState( InputState::IDLE );
                }
            }
            keyboard.SetExternalStatus( pretension.GetStatus() );
        }

        // Standalone tension adjustment - manual tension mode only, no
        // guided unspool/zero/home-recording steps (see TENSION_ADJUST entry
        // handling above).
        if ( kb.systemState == SystemState::TENSION_ADJUST ) {
            if ( kb.pendingTensionAdjust.active ) {
                if ( kb.pendingTensionAdjust.isAbsolute ) {
                    controller.SetManualTension( kb.pendingTensionAdjust.motor,
                                                 kb.pendingTensionAdjust.valueN );
                } else {
                    controller.AdjustManualTension( kb.pendingTensionAdjust.motor,
                                                    kb.pendingTensionAdjust.deltaN );
                }
                keyboard.ClearTensionAdjust();
            }
            keyboard.SetExternalStatus( pretension.GetTensionAdjustStatus() );
        }

        // Direction-dependent gain tuning - 'P' adjusts gainTune_A/B/C (the
        // custom-tuned proportional gain, seeded from gain_kP, combined with
        // K(theta) to form kP_effective); 'I' adjusts iGainTune_A/B/C (the
        // custom-tuned integral gain, seeded from gain_kI, forming kI_effective
        // for the Stage 1 endgame integrator). Both overlays work alongside
        // normal operation; neither gates output or manual tension mode.
        if ( IsIGainTuneInputState( kb.inputState ) ) {
            if ( kb.pendingGainAdjust.active ) {
                controller.AdjustIGainTune( kb.pendingGainAdjust.motor, kb.pendingGainAdjust.deltaGain );
                keyboard.ClearGainAdjust();
            }
            keyboard.SetExternalStatus( controller.GetIGainTuneStatus() );
        } else if ( IsGainTuneInputState( kb.inputState ) ) {
            if ( kb.pendingGainAdjust.active ) {
                controller.AdjustGainTune( kb.pendingGainAdjust.motor, kb.pendingGainAdjust.deltaGain );
                keyboard.ClearGainAdjust();
            }
            keyboard.SetExternalStatus( controller.GetGainTuneStatus() );
        }

        // RobotState ladder - derive PWM-output policy from serial
        // connection + tensioning completeness + guidance enable.
        // Independent of PRETENSION/TENSION_ADJUST/CAL_ROM, which manage
        // controller output/manual-tension mode directly while active.
        bool guidanceActive = controller.IsTargetActive() && controller.IsOutputEnabled() && controller.IsGuidanceOutputEnabled();

        bool inOverrideMode = ( kb.systemState == SystemState::PRETENSION ||
                                kb.systemState == SystemState::TENSION_ADJUST ||
                                kb.inputState == InputState::CAL_ROM ||
                                kb.inputState == InputState::CAL_STI );

        RobotState robotState;
        if ( !serial.IsConnected() ) {
            robotState = RobotState::DISCONNECTED;
        } else if ( guidanceActive ) {
            robotState = RobotState::GUIDING;
        } else if ( pretension.IsComplete() ) {
            robotState = RobotState::READY;
        } else {
            robotState = RobotState::IDLE;
        }

        if ( !inOverrideMode ) {
            bool holdPreload = ( robotState == RobotState::READY || robotState == RobotState::GUIDING );
            controller.SetOutputEnabled( holdPreload );
            controller.SetManualTensionMode( false );
        }

        // Flick/confirm gesture detection - armed only while READY (not
        // GUIDING/IDLE/etc); Reset() clears the state machine on every other
        // RobotState so stale velocity history can't fire a gesture right
        // after entering READY.
        if ( robotState == RobotState::READY ) {
            GestureEvent gestureEvent = gesture.Update( nowSecs );

            // In FITTS, a flick steps the active target marker up/down,
            // clamped to [1, 45] (45-marker grid). CONFIRM (circle) is
            // detected and shown but not yet bound to an action.
            if ( kb.systemState == SystemState::FITTS ) {
                if ( gestureEvent == GestureEvent::FLICK_UP ) {
                    keyboard.SetFittsTargetId( kb.fittsTargetId + 1 );
                } else if ( gestureEvent == GestureEvent::FLICK_DOWN ) {
                    keyboard.SetFittsTargetId( kb.fittsTargetId - 1 );
                }
            }
        } else {
            gesture.Reset();
        }

        // g. Serial - update the pending TX packet each new camera frame.
        //    The TX thread sends it independently at 200 Hz; packet_index is
        //    managed by the TX thread and does not need to be set here.
        if ( kb.pendingMotorTest.active ) {
            motorTestActive = true;
            motorTestMotor = kb.pendingMotorTest.motor;
            motorTestPwm = kb.pendingMotorTest.pwm;
            motorTestStart = Clock::now();
            keyboard.ClearMotorTest();
        }

        if ( isNewFrame ) {
            // Reflect the RobotState ladder to the Teensy: READY/GUIDING hold
            // preload (or full) tension, everything else is plain IDLE.
            lastTxPkt.state = ( robotState == RobotState::READY || robotState == RobotState::GUIDING )
                                  ? static_cast<uint8_t>( PcState::READY )
                                  : static_cast<uint8_t>( PcState::IDLE );
            lastTxPkt.pwm_A = 2047;
            lastTxPkt.pwm_B = 2047;
            lastTxPkt.pwm_C = 2047;

            // Controller output - only forwarded once PretensionHandler has
            // enabled it (TENSION step); otherwise PWM stays at 2047 (off).
            if ( controller.IsOutputEnabled() ) {
                lastTxPkt.pwm_A = controller.GetPwmA();
                lastTxPkt.pwm_B = controller.GetPwmB();
                lastTxPkt.pwm_C = controller.GetPwmC();
            }

            if ( motorTestActive ) {
                double elapsed = std::chrono::duration<double>( Clock::now() - motorTestStart ).count();
                if ( elapsed < 1.0 ) {
                    if ( motorTestMotor == 'A' )
                        lastTxPkt.pwm_A = motorTestPwm;
                    else if ( motorTestMotor == 'B' )
                        lastTxPkt.pwm_B = motorTestPwm;
                    else if ( motorTestMotor == 'C' )
                        lastTxPkt.pwm_C = motorTestPwm;
                    else if ( motorTestMotor == 'D' ) {
                        lastTxPkt.pwm_A = motorTestPwm;
                        lastTxPkt.pwm_B = motorTestPwm;
                        lastTxPkt.pwm_C = motorTestPwm;
                    }
                } else {
                    motorTestActive = false;
                    keyboard.SetExternalStatus( "Motor test done - back to idle." );
                }
            }

            serial.SetPendingTx( lastTxPkt );
        }

        // Assemble serial state for the display - always up to date even when
        // the panel only refreshes at 10 Hz.
        SerialState serialSt;
        serialSt.isConnected = serial.IsConnected();
        serialSt.txFrequencyHz = serial.GetTxFrequency();
        serialSt.lastTx = lastTxPkt;
        serialSt.lastTx.packet_index = serial.GetLastSentIndex();    // TX thread owns this - never set by main
        serialSt.hasRx = serial.GetLatestPacket( serialSt.lastRx );
        serialSt.robotState = robotState;

        // h. Fitts task - virtual fingertip cursor + touch error overlay,
        //    updated when in FITTS mode.
        if ( isNewFrame ) {
            if ( kb.systemState == SystemState::FITTS ) {
                fitts.Update( markers, touchState, cal3.IsComplete(),
                              cal3.GetFinalOffset(), cal3.GetRollRef() );
                display.SetTouchFingertip( fitts.HasTouchFingertip(), fitts.GetTouchFingertipPx() );

                // Freeze the active-target outline at the trial-ending touch
                // (rising edge of HasTouchSample), so a magenta reference box
                // marks the just-acquired target until the next one loads. Prefer
                // the directly-detected marker; fall back to the board estimate.
                const bool fittsTouchSample = fitts.HasTouchSample();
                if ( fittsTouchSample && !prevFittsTouchSample ) {
                    if ( activeMarker ) {
                        for ( int k = 0; k < 4; k++ ) touchedTargetCorners[k] = activeMarker->cornersPx[k];
                        touchedTargetBoxValid = true;
                    } else if ( fbTargetValid ) {
                        touchedTargetCorners = fbCorners;
                        touchedTargetBoxValid = true;
                    } else {
                        touchedTargetBoxValid = false;
                    }
                }
                prevFittsTouchSample = fittsTouchSample;
                display.SetTouchedTargetBox( touchedTargetBoxValid && fittsTouchSample, touchedTargetCorners );
                aruco.SetFittsOverlay( fitts.HasTouchSample(), fitts.GetTouchScreenPx(), kb.fittsTargetId,
                                       fitts.GetErrorLine1(), fitts.GetErrorLine2() );

                // Magenta reference outline around the target marker on the
                // touchscreen once the trial-ending touch is registered, kept
                // until the next target is selected (HasTouchSample resets then).
                aruco.SetTargetOutline( fitts.HasTouchSample(), kb.fittsTargetId );

                // Trial logging - one row per camera frame while a capture runs.
                if ( trialLogger.IsActive() ) {
                    cv::Point3f tpos;
                    cv::Vec4f   tquat;
                    cv::Point3f tdisp;
                    bool        tDetected = false;
                    if ( fitts.GetTargetFullPose( markers, kb.fittsTargetId,
                                                  cal3.IsComplete(), cal3.GetFinalOffset(),
                                                  cal3.GetRollRef(),
                                                  tpos, tquat, tdisp, tDetected ) ) {
                        const auto  tele = controller.GetTelemetry();
                        TrialSample s;
                        // Stamp with the camera capture time (CameraHandler sets
                        // frame.timestamp), not the main-loop poll time, so the
                        // sample cadence reflects the true frame interval. Write()
                        // re-zeroes to the first row, so absolute value is fine.
                        s.tSecs = frame.timestamp;
                        s.targetId = kb.fittsTargetId;
                        s.detected = tDetected ? 1 : 0;
                        s.tx = tpos.x;
                        s.ty = tpos.y;
                        s.tz = tpos.z;
                        s.dx = tdisp.x;
                        s.dy = tdisp.y;
                        s.dz = tdisp.z;
                        // Virtual fingertip = target - Δp (camera frame Y-up): the
                        // system's estimate of the fingertip point, logged per frame.
                        // s.vx = tpos.x - tdisp.x; s.vy = tpos.y - tdisp.y; Original
                        s.vx = tele.pos_virtual.x;
                        s.vy = tele.pos_virtual.y;
                        s.qx = tquat[0];
                        s.qy = tquat[1];
                        s.qz = tquat[2];
                        s.qw = tquat[3];
                        s.pwmA = tele.outputPwm.x;
                        s.pwmB = tele.outputPwm.y;
                        s.pwmC = tele.outputPwm.z;
                        trialLogger.AddSample( s );
                    }
                    // End the trial on touchscreen contact (rising edge).
                    if ( touchState.isTouched && !prevLogTouched ) {
                        std::string saved = trialLogger.FinishTrial( ( float )touchState.position.x,
                                                                     ( float )touchState.position.y,
                                                                     cfg.touchscreen.mmPerPixel );
                        if ( !saved.empty() )
                            keyboard.SetExternalStatus( "Log file saved as " + saved );
                    }
                }
                prevLogTouched = touchState.isTouched;
            } else {
                display.SetTouchFingertip( false );
                aruco.SetFittsOverlay( false, {}, 0, "", "" );
                aruco.SetTargetOutline( false, 0 );
                display.SetTouchedTargetBox( false, touchedTargetCorners );
                touchedTargetBoxValid = false;
                prevFittsTouchSample = false;
            }
        }

        // i. Operator display + telemetry - only refresh on a new camera frame
        if ( isNewFrame ) {
            display.SetControllerTelemetry( controller.GetTelemetry() );
            display.SetCal1State( kb.inputState == InputState::CAL_ROM && !cal1.IsComplete(),
                                  cal1.GetSamples(), cal1.GetBoundary() );
            display.SetCal2State( kb.inputState == InputState::CAL_STI && !cal2.IsComplete(),
                                  cal2.GetCurrentHeadingIndex() );
            display.SetGestureIndicator( gesture.IsIndicatorActive( nowSecs ), gesture.GetLastGesture() );
            display.SetLoggingStatus( trialLogger.IsPrimed(), trialLogger.IsActive() );
            display.SetArucoStats( aruco.GetDetectionHz(), aruco.GetDetectionLagMs() );

            // // --- Phase 1 detection diagnostics: once-per-second console summary
            // // Quantifies the gap between camera delivery and pose-update rate
            // // (duplicate frames = detector falling behind), plus per-stage
            // // detector timing, so detection cost can be measured on the rig.
            // // TEMPORARY - remove once Phase 1 tuning is complete.
            // {
            //     static double diagWindowStart = nowSecs;
            //     static int    diagCamFrames = 0;
            //     static int    diagPoseUpdates = 0;
            //     static double diagPrevResultTs = -1.0;

            //     diagCamFrames++;
            //     const double curResultTs = aruco.GetLatestDetectionTimestamp();
            //     if ( curResultTs != diagPrevResultTs ) {
            //         diagPoseUpdates++;
            //         diagPrevResultTs = curResultTs;
            //     }

            //     if ( kb.systemState == SystemState::FITTS ) {
                    
            //         const double win = nowSecs - diagWindowStart;
            //         if ( win >= 1.0f ) {
            //             ControllerTelemetry tele = controller.GetTelemetry();
            //             const double        camHz = diagCamFrames / win;
            //             const double        poseHz = diagPoseUpdates / win;
            //             const double        dupPct = camHz > 1e-6 ? 100.0 * ( 1.0 - poseHz / camHz ) : 0.0;
            //             char                line[256];
            //             std::snprintf( line, sizeof( line ),
            //                            "[DET] ID=%d tpos.z=%.2fmm cam=%.1fHz pose=%.1fHz (dup %.0f%%) detect=%.1fHz | "
            //                            "detectMarkers=%.1fms poseSolve=%.2fms markers=%d lag=%.0fms",
            //                            kb.activeTagId, tele.displacement.z, camHz, poseHz, dupPct, aruco.GetDetectionHz(),
            //                            aruco.GetDetectMarkersMs(), aruco.GetPoseSolveMs(),
            //                            aruco.GetMarkerCount(), aruco.GetDetectionLagMs() );
            //             std::cout << line << std::endl;

            //             diagWindowStart = nowSecs;
            //             diagCamFrames = 0;
            //             diagPoseUpdates = 0;
            //         }
            //     }
            // }

            // Pair the displayed image with the frame the current marker
            // detection was computed from, so the overlay never drifts
            // relative to the image underneath. Falls back to the live
            // frame if that source frame has already aged out of history.
            const cv::Mat* displayFrame = &frame.undistorted;
            double         resultTs = aruco.GetLatestDetectionTimestamp();
            for ( const auto& f : frameHistory ) {
                if ( f.timestamp == resultTs ) {
                    displayFrame = &f.undistorted;
                    break;
                }
            }

            display.Update( *displayFrame, markers, touchState, kb, serialSt );
        }

        // Stall probe (see iterStart): flag long iterations that drop frames.
        const double iterMs =
            std::chrono::duration<double, std::milli>( Clock::now() - iterStart ).count();
        if ( iterMs > 25.0 ) {
            std::cerr << "Main: long iteration " << std::fixed << std::setprecision( 1 )
                      << iterMs << " ms"
                      << ( trialLogger.IsActive() ? " (DROPPED frames during active trial)" : "" )
                      << " - newFrame=" << isNewFrame
                      << " fitts=" << ( kb.systemState == SystemState::FITTS )
                      << "\n";
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
