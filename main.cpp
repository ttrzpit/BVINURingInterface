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
#include "CameraHandler.h"
#include "Colors.h"
#include "Config.h"
#include "DisplayHandler.h"
#include "KeyboardHandler.h"
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

    DisplayHandler display(cfg.display,
                           cv::Point2i(static_cast<int>(cfg.camera.cx),
                                       static_cast<int>(cfg.camera.cy)),
                           cfg.telemetry);

    SerialHandler  serial(cfg.serial);

    // ---- Start background threads -------------------------------------------
    camera.start();    // Camera grab loop runs on its own thread
    serial.start();    // Serial receive loop runs on its own thread (stub)

    // ---- Show initial ArUco grid on the touchscreen -------------------------
    // This window persists for the duration of the program.
    // Call aruco.updateGridConfig(...) at any point to change the layout.
    aruco.showMarkerGrid();

    std::cout << "\nMain: Running. Press ESC to quit.\n\n";

    KeyboardHandler keyboard;

    // ---- Main loop ----------------------------------------------------------
    while (g_running) {

        // a. Keyboard — PollKey() must be called every iteration to keep all
        //    OpenCV windows responsive. The result is fed to KeyboardHandler
        //    which manages multi-character commands and the quit flag.
        int key = display.PollKey();
        keyboard.ProcessKey(key);
        const KeyboardState& kb = keyboard.GetState();
        if (kb.quitRequested) break;

        // b. Camera frame — returns the most recently completed frame.
        //    Non-blocking: if the camera thread hasn't produced a new frame yet,
        //    we get the same frame as last iteration (ready flag is still true).
        CameraFrame frame = camera.getLatestFrame();

        // c. ArUco detection — only runs when a valid frame is available
        std::vector<DetectedMarker> markers;
        if (frame.ready) {
            markers = aruco.detect(frame.gray);
        }

        // d. Touch state — drains pending X11 events, returns current state
        TouchState touchState = touch.getLatestTouch();

        // e. Operator display + telemetry panel — telemetry is populated and shown
        //    inside Update() via DisplayHandler::PopulateTelemetryPanel()
        //    activeTagId drives the green corner outline in DrawMarkerOverlays
        if (frame.ready) {
            display.Update(frame.undistorted, markers, touchState, kb);
        }

        // f. Serial (stub — uncomment when Teensy is connected)
        // serial.send("...");
        // std::string received = serial.getLatestReceived();
    }

    // ---- Clean shutdown -----------------------------------------------------
    std::cout << "\nMain: Shutting down...\n";

    camera.stop();
    serial.stop();
    cv::destroyAllWindows();

    std::cout << "Main: Done.\n";
    return 0;
}
