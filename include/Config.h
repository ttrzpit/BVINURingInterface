#pragma once

// =============================================================================
// Config.h — Runtime configuration structs and loader
//
// All tunable system parameters are defined here as plain structs and loaded
// from config.yaml at startup via cv::FileStorage (built into OpenCV — no
// extra dependencies needed).
//
// Usage:
//   Config cfg;
//   cfg.load("config.yaml");
//   // Pass const refs to each handler:
//   CameraHandler camera(cfg.camera);
//   ArucoHandler  aruco(cfg.arucoDetect, cfg.arucoDisplay, ...);
// =============================================================================

#include <array>
#include <string>

#include <opencv2/core.hpp>     // cv::Mat, cv::FileStorage


// ---- Camera -----------------------------------------------------------------

struct CameraConfig {

    // Device
    std::string device    = "/dev/video0";
    int         width     = 1600;
    int         height    = 1200;
    int         framerate = 90;
    bool        rotate180 = false;   // Flip image 180° (upside-down mount)

    // Calibration intrinsics
    double fx = 600.98172, fy = 599.86930;  // Focal lengths [px]
    double cx = 801.26194, cy = 535.69455;  // Principal point [px]
    std::array<double, 5> distortion = { 0.01561, -0.03298, 0.00020, 0.00136, 0.00561 };

    // Derived from the above — built by Config::load(), not set in config.yaml
    cv::Mat cameraMatrix;  // 3×3 intrinsic matrix
    cv::Mat distCoeffs;    // 1×5 distortion coefficients

    // Hardware controls
    int  brightness       = 0;
    int  contrast         = 0;
    int  saturation       = 32;
    int  hue              = 0;
    bool autoWhiteBalance = false;
    int  gamma            = 120;
    int  gain             = 18;
    int  sharpness        = 0;
    int  backlight        = 0;
    int  autoExposure     = 1;   // 1 = manual, 3 = aperture priority
    int  exposureLevel    = 32;
    bool autoFocus        = false;
    int  focusLevel       = 0;
    int  zoom             = 0;
};


// ---- ArUco Detection --------------------------------------------------------

struct ArucoMarkerConfig {
    float markerSizeMm = 18.0f;  // Physical side length of ring markers [mm]
    int   validIdMin   = 0;      // Ignore detected IDs below this
    int   validIdMax   = 20;     // Ignore detected IDs above this
};

// ---- ArUco Detector Algorithm Parameters ------------------------------------
// Controls the internal OpenCV ArUco detector pipeline.
// Defaults match the values tuned for the NURing high-res camera setup.

struct ArucoDetectorConfig {

    // Adaptive thresholding — converts the grayscale frame to binary before
    // searching for marker borders. Larger window sizes catch markers that are
    // farther from the camera; smaller windows are faster.
    double adaptiveThreshConstant    = 7.0;  // Added to the mean in the threshold formula
    int    adaptiveThreshWinSizeMin  = 3;    // Smallest window size [px]
    int    adaptiveThreshWinSizeMax  = 53;   // Largest window size [px]
    int    adaptiveThreshWinSizeStep = 4;    // Step between window sizes [px]

    // Marker geometry filters — reject detections that are too small, too large,
    // or have imprecise polygon fits.
    double minMarkerPerimeterRate      = 0.01;  // Minimum perimeter as fraction of frame perimeter
    double maxMarkerPerimeterRate      = 4.0;   // Maximum perimeter as fraction of frame perimeter
    double polygonalApproxAccuracyRate = 0.03;  // Corner approximation tolerance
    double minCornerDistanceRate       = 0.02;  // Minimum distance between corners (fraction of perimeter)
    int    minDistanceToBorder         = 1;     // Minimum distance from marker to frame edge [px]

    // Corner refinement — sub-pixel refinement improves 3D pose accuracy.
    // method: 0 = none, 1 = subpix (default), 2 = contour, 3 = AprilTag
    int    cornerRefinementMethod        = 1;
    int    cornerRefinementMaxIterations = 50;
    double cornerRefinementMinAccuracy   = 0.01;

    // Detect markers printed on reflective or glossy surfaces that invert the
    // black/white pattern under certain lighting conditions.
    bool detectInvertedMarker = false;

    // Perspective removal — controls the resolution of the internal bit-extraction
    // step. Higher pixel-per-cell values are more accurate but slower.
    int    perspectiveRemovePixelPerCell        = 8;
    double perspectiveRemoveIgnoredMarginPerCell = 0.13;

    // ArUco3 detection — improved algorithm that is faster and more robust for
    // small or distant markers. Requires OpenCV 4.6+.
    bool useAruco3Detection = true;
};


// ---- ArUco Display (touchscreen grid) ---------------------------------------

struct ArucoDisplayConfig {
    int   cols         = 4;     // Grid columns
    int   rows         = 2;     // Grid rows
    float markerSizeMm = 20.0f; // Marker side length [mm] — converted to px at render time
    float paddingMm    = 22.0f; // Padding on all 4 sides [mm] — converted to px at render time
    // Spacing between markers is auto-calculated from paddingMm, markerSizeMm, and screen dimensions
};


// ---- Touchscreen Monitor ----------------------------------------------------

struct TouchscreenConfig {
    int         width          = 1920;
    int         height         = 1080;
    int         xOffset        = 3440;    // X position of the touchscreen in the desktop coordinate space
    int         yOffset        = 0;       // Y position (0 when monitors are top-aligned)
    float       pixelsPerMm    = 3.6430f; // Physical pixel density of the touchscreen [px/mm]
    float       mmPerPixel     = 0.27450f;// Inverse — use whichever direction is convenient
    int         xinputDeviceId = 9;       // xinput device ID for the touchscreen
    std::string xinputOutput   = "HDMI-0";// X output name to map the touchscreen to
};


// ---- Operator Display (main monitor) ----------------------------------------

struct DisplayConfig {
    int width  = 1600;
    int height = 1070;
    int xPos   = 0;    // Window X position on the desktop
    int yPos   = 0;    // Window Y position on the desktop
};


// ---- Telemetry Panel (below the operator display) ---------------------------

struct TelemetryConfig {
    int width  = 1600;
    int height = 270;
    int cols   = 50;   // Number of grid columns (cell width = width / cols)
    int rows   = 6;    // Number of grid rows    (cell height = height / rows)
    int xPos   = 0;    // Window X position on the desktop
    int yPos   = 1100; // Window Y position — set to approx. display height + title bar
};


// ---- Controller Panel (separate tall narrow panel for controller telemetry) -

struct ControllerPanelConfig {
    int width  = 256;
    int height = 1344;
    int cols   = 16;   // Number of grid columns (cell width = width / cols)
    int rows   = 42;   // Number of grid rows    (cell height = height / rows)
    int xPos   = 1570; // Window X position on the desktop
    int yPos   = 0;    // Window Y position on the desktop
};


// ---- Serial (Teensy) --------------------------------------------------------

struct SerialConfig {
    std::string port     = "/dev/ttyACM0";   // Single full-duplex USB CDC port
    int         baudRate = 1000000;           // 1 Mbaud (nominal for USB CDC)
};


// =============================================================================
// Config — loads all of the above from config.yaml
// =============================================================================

class Config {
public:
    /**
     * @brief Load config.yaml from disk. Falls back to struct defaults if the
     *        file cannot be opened. Always returns usable config either way.
     * @param filepath  Path to the YAML config file
     * @return true if the file loaded successfully, false if using defaults
     */
    bool load(const std::string& filepath = "config.yaml");

    // Sub-configs — hand these to handlers by const reference
    CameraConfig        camera;
    ArucoMarkerConfig   arucoMarker;
    ArucoDetectorConfig arucoDetector;  // Algorithm tuning params for the OpenCV detector
    ArucoDisplayConfig  arucoDisplay;
    TouchscreenConfig   touchscreen;
    DisplayConfig       display;
    TelemetryConfig        telemetry;
    ControllerPanelConfig  controllerPanel;
    SerialConfig           serial;

private:
    // Build cameraMatrix and distCoeffs from the scalar values after loading
    void buildDerivedCameraValues();
};
