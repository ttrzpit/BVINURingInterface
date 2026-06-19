#pragma once

// =============================================================================
// Config.h - Runtime configuration structs and loader
//
// All tunable system parameters are defined here as plain structs and loaded
// from config.yaml at startup via cv::FileStorage (built into OpenCV - no
// extra dependencies needed).
//
// Usage:
//   Config cfg;
//   cfg.load("config.yaml");
//   // Pass const refs to each handler:
//   CameraHandler camera(cfg.camera);
//   ArucoHandler  aruco(cfg.arucoDetect, cfg.arucoDisplay, ...);
// =============================================================================


// ========================================
// !!!   MODIFY VALUES IN CONFIG.YAML   !!!
// !!!   MODIFY VALUES IN CONFIG.YAML   !!!
// !!!   MODIFY VALUES IN CONFIG.YAML   !!!
// !!!   MODIFY VALUES IN CONFIG.YAML   !!!
// !!!   MODIFY VALUES IN CONFIG.YAML   !!!
// ========================================


#include <array>
#include <opencv2/core.hpp>  // cv::Mat, cv::FileStorage
#include <string>

// ---- Camera -----------------------------------------------------------------

struct CameraConfig {
    // Device
    std::string device = "/dev/video0";
    int width = 1600;
    int height = 1200;
    int framerate = 90;
    bool rotate180 = false;  // Flip image 180° (upside-down mount)

    // Calibration intrinsics
    double fx = 600.98172, fy = 599.86930;  // Focal lengths [px]
    double cx = 801.26194, cy = 535.69455;  // Principal point [px]
    std::array<double, 5> distortion = {0.01561, -0.03298, 0.00020, 0.00136, 0.00561};

    // Derived from the above - built by Config::load(), not set in config.yaml
    cv::Mat cameraMatrix;  // 3×3 intrinsic matrix
    cv::Mat distCoeffs;    // 1×5 distortion coefficients

    // Hardware controls
    int brightness = 0;
    int contrast = 0;
    int saturation = 32;
    int hue = 0;
    bool autoWhiteBalance = false;
    int gamma = 120;
    int gain = 18;
    int sharpness = 0;
    int backlight = 0;
    int autoExposure = 1;  // 1 = manual, 3 = aperture priority
    int exposureLevel = 32;
    bool autoFocus = false;
    int focusLevel = 0;
    int zoom = 0;
};

// ---- ArUco Detection --------------------------------------------------------

struct ArucoMarkerConfig {
    float markerSizeMm = 20.0f;  // Physical side length of ring markers [mm]
    int validIdMin = 0;          // Ignore detected IDs below this
    int validIdMax = 45;         // Ignore detected IDs above this
};

// ---- ArUco Detector Algorithm Parameters ------------------------------------
// Controls the internal OpenCV ArUco detector pipeline.
// Defaults match the values tuned for the NURing high-res camera setup.

struct ArucoDetectorConfig {
    // Adaptive thresholding - converts the grayscale frame to binary before
    // searching for marker borders. Larger window sizes catch markers that are
    // farther from the camera; smaller windows are faster.
    double adaptiveThreshConstant = 7.0;  // Added to the mean in the threshold formula
    int adaptiveThreshWinSizeMin = 3;     // Smallest window size [px]
    int adaptiveThreshWinSizeMax = 53;    // Largest window size [px]
    int adaptiveThreshWinSizeStep = 4;    // Step between window sizes [px]

    // Marker geometry filters - reject detections that are too small, too large,
    // or have imprecise polygon fits.
    double minMarkerPerimeterRate = 0.01;       // Minimum perimeter as fraction of frame perimeter
    double maxMarkerPerimeterRate = 4.0;        // Maximum perimeter as fraction of frame perimeter
    double polygonalApproxAccuracyRate = 0.03;  // Corner approximation tolerance
    double minCornerDistanceRate = 0.02;        // Minimum distance between corners (fraction of perimeter)
    int minDistanceToBorder = 1;                // Minimum distance from marker to frame edge [px]

    // Corner refinement - sub-pixel refinement improves 3D pose accuracy.
    // method: 0 = none, 1 = subpix (default), 2 = contour, 3 = AprilTag
    int cornerRefinementMethod = 1;
    int cornerRefinementMaxIterations = 50;
    double cornerRefinementMinAccuracy = 0.01;

    // Detect markers printed on reflective or glossy surfaces that invert the
    // black/white pattern under certain lighting conditions.
    bool detectInvertedMarker = false;

    // Perspective removal - controls the resolution of the internal bit-extraction
    // step. Higher pixel-per-cell values are more accurate but slower.
    int perspectiveRemovePixelPerCell = 8;
    double perspectiveRemoveIgnoredMarginPerCell = 0.13;

    // ArUco3 detection - improved algorithm that is faster and more robust for
    // small or distant markers. Requires OpenCV 4.6+.
    bool useAruco3Detection = true;
};

// ---- ArUco Display (touchscreen grid - Fitts / study task) ------------------
// Markers are distributed evenly across the screen with equal outer padding.
// Spacing is auto-calculated to fill the usable area.

struct ArucoDisplayConfig {
    int cols = 4;                // Grid columns
    int rows = 2;                // Grid rows
    float markerSizeMm = 20.0f;  // Marker side length [mm] - converted to px at render time
    float paddingMm = 22.0f;     // Outer padding on all 4 sides [mm] - converted to px at render time
};

// ---- ArUco Calibration Grid (touchscreen grid - Cal2 / Cal3 solvePnP) -------
// Columns and rows are auto-calculated: pack markers with exactly markerPadMm
// between them inside the exclusion zone boundary. Uses DICT_4X4_250.

struct ArucoCalibrationGridConfig {
    float markerSizeMm     = 15.0f;  // Physical side length of each marker [mm]
    float markerPadMm      = 15.0f;  // Gap between adjacent markers [mm]
    float markerExclusionMm = 20.0f; // Minimum margin from screen edge to nearest marker [mm]
};

// ---- Fitts Board (multi-scale touchscreen board - FITTS pointing task) ------
// Two-level nested fiducial board with EXPLICIT placement (config.yaml
// [fitts_board_assignments]). A dense grid of small "fine" markers (subset B)
// provides close-range reference + the pointing targets; four large "coarse"
// markers (subset A) provide a wide-baseline reference that stays visible from
// far away or when the hand occludes the centre. All positions are marker
// CENTRES in screen-plane mm (origin = screen top-left). The board uses
// DICT_4X4_1000 (the fine grid alone can exceed 250 IDs). The fine grid is laid
// out from the first marker's centre, stepping by (size + pad) centre-to-centre;
// IDs run fineIdStart.. row-major, then the four coarse markers follow.

struct FittsBoardConfig {
    // Fine grid (subset B) - dense reference + pointing targets
    float fineMarkerSizeMm = 8.0f;    // Side length of each fine marker [mm]
    float finePadMm        = 8.0f;    // Gap between fine markers [mm] (pitch = size + pad)
    float fineFirstXMm     = 31.5f;   // Centre X of the first (top-left) fine marker [mm]
    float fineFirstYMm     = 36.23f;  // Centre Y of the first (top-left) fine marker [mm]
    int   fineRows         = 15;      // Number of fine-grid rows
    int   fineCols         = 27;      // Number of fine-grid columns
    int   fineIdStart      = 1;       // First fine marker ID (0 reserved for "no target")

    // Coarse perimeter (subset A) - four explicit marker CENTRES in screen mm.
    // IDs follow the fine band: fineIdStart + fineRows*fineCols + (0..3).
    float coarseMarkerSizeMm = 56.0f;
    std::array<float, 4> coarseCenterXMm = {39.5f, 487.5f, 39.5f, 487.5f};
    std::array<float, 4> coarseCenterYMm = {44.23f, 44.23f, 252.23f, 252.23f};
};

// ---- Touchscreen Monitor ----------------------------------------------------

struct TouchscreenConfig {
    int width = 1920;
    int height = 1080;
    int xOffset = 3440;                   // X position of the touchscreen in the desktop coordinate space
    int yOffset = 0;                      // Y position (0 when monitors are top-aligned)
    float pixelsPerMm = 3.6430f;          // Physical pixel density of the touchscreen [px/mm]
    float mmPerPixel = 0.27450f;          // Inverse - use whichever direction is convenient
    int xinputDeviceId = 9;               // xinput device ID for the touchscreen
    std::string xinputOutput = "HDMI-0";  // X output name to map the touchscreen to
};

// ---- Operator Display (main monitor) ----------------------------------------

struct DisplayConfig {
    int width = 1600;
    int height = 1070;
    int xPos = 0;  // Window X position on the desktop
    int yPos = 0;  // Window Y position on the desktop
};

// ---- Telemetry Panel (below the operator display) ---------------------------

struct TelemetryConfig {
    int width = 1600;
    int height = 270;
    int cols = 50;    // Number of grid columns (cell width = width / cols)
    int rows = 6;     // Number of grid rows    (cell height = height / rows)
    int xPos = 0;     // Window X position on the desktop
    int yPos = 1100;  // Window Y position - set to approx. display height + title bar
};

// ---- Controller Panel (separate tall narrow panel for controller telemetry) -

struct ControllerPanelConfig {
    int width = 256;
    int height = 1344;
    int cols = 16;    // Number of grid columns (cell width = width / cols)
    int rows = 42;    // Number of grid rows    (cell height = height / rows)
    int xPos = 1570;  // Window X position on the desktop
    int yPos = 0;     // Window Y position on the desktop
};

// ---- Calibration Stage 3 ----------------------------------------------------

struct Cal3Config {
    double holdSecs     = 0.05;  // Required touch hold duration before recording [s]
    double cooldownSecs = 2.0;   // Minimum gap between successive samples [s]
    int    maxSamples   = 10;    // Total touches required to complete calibration
};

// ---- Calibration Stage 1: Finger Active Range of Motion (AROM) --------------

struct Cal1Config {
    double recordSecs = 10.0;  // Duration of the circle-tracing recording [s]
};

// ---- Calibration Stage 2: Finger Deflection Stiffness -----------------------

struct Cal2Config {
    float  forceRampRate   = 0.5f;  // Open-loop force ramp-up rate [N/s]
    double holdSecs        = 2.0;   // Hold duration at peak force per heading [s]
    double releaseWaitSecs = 2.0;   // Wait after releasing before the next heading [s]
};

// ---- Gesture Detection (flick up/down, RobotState::READY only) -------------

struct GestureConfig {
    float  velocityThreshMmS = 80.0f;  // |vel_filtered_.y| trigger threshold [mm/s]
    int    armSamples        = 2;      // Consecutive over-threshold samples required to arm
    float  minDisplacementMm = 3.0f;   // Net |dy| required within maxWindowSecs to confirm [mm]
    double maxWindowSecs     = 0.25;   // Max time after arming to confirm displacement [s]
    double cooldownSecs      = 0.4;    // Min gap between raw flicks; also on-screen arrow duration [s]
    double doubleFlickWindowSecs = 1.5; // Max time between two same-direction raw flicks for them to register as a single FLICK_UP/FLICK_DOWN [s]

    // ---- Confirm gesture (circle) + flick false-positive rejection ---------
    float  restSpeedThreshMmS  = 15.0f;  // |vel_filtered_| below this counts as "at rest" [mm/s]
    double flickMaxMotionSecs  = 0.3;    // Max continuous time spent moving (since last at rest) for a velocity spike to still count as a flick [s] - sustained motion (a circle) exceeds this and blocks flick arming
    float  circleConfirmRad    = 5.236f; // Cumulative rotation to confirm a circle [rad] (~300 deg)
    double circleMaxWindowSecs = 1.5;    // Rolling time window for rotation accumulation [s]
    float  circleMinRadiusMm   = 5.0f;   // Min path radius from centroid - rejects jitter [mm]
    float  circleMaxRadiusMm   = 50.0f;  // Max path radius from centroid - rejects large sweeps [mm]
    double circleCooldownSecs  = 0.6;    // Min gap between confirms; also on-screen indicator duration [s]
};

// ---- Fitts Target Circle -----------------------------------------------------

struct TargetConfig {
    float radiusMm        = 5.0f;   // Drawn target circle radius [mm]
    float offsetDefaultMm = 40.0f;  // Gray circle Y offset below the tag before Cal3 completes [mm]
};

// ---- Serial (Teensy) --------------------------------------------------------

struct SerialConfig {
    std::string port = "/dev/ttyACM0";  // Single full-duplex USB CDC port
    int baudRate = 1000000;             // 1 Mbaud (nominal for USB CDC)
};

// ---- Controller (PID gains and solver parameters) ---------------------------

struct ControllerConfig {
    float gain_kP             = 1.0f;    // Proportional gain [N/mm]
    float gain_kD             = 0.0f;    // Derivative gain [N·s/mm]
    float gain_kI             = 0.0f;    // Integral gain [N/(mm·s)]
    float deflection_force_max = 5.0f;   // Maximum allowable guidance deflection force magnitude [N]
    float tension_preload_min   = 0.1f;  // T_preload_min: minimum preload tension per motor [N]
    float tension_preload_max   = 2.0f;  // T_preload_max: maximum preload tension per motor [N]
    float tension_deflection_max = 1.0f; // T_deflection_max: max per-motor tension contribution from the guidance/deflection force [N]
    float tension_output_max    = 3.0f;  // T_output_max: max total commanded tension per motor (T_preload + T_deflection) [N]
    float position_tolerance  = 1.0f;    // Deadband radius - no force inside [mm]
    float lowpass_alpha       = 0.15f;   // Velocity low-pass coefficient (0=heavy, 1=none)
    float ramp_duration_secs  = 1.0f;   // Force ramp-up duration after new target [s]
    float max_current_amps    = 1.89f;   // Amplifier max current [A]
    int   encoder_counts_per_rev = 4096; // Encoder ticks per motor revolution
};

// =============================================================================
// Config - loads all of the above from config.yaml
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

    // Sub-configs - hand these to handlers by const reference
    CameraConfig camera;
    ArucoMarkerConfig arucoMarker;
    ArucoDetectorConfig arucoDetector;  // Algorithm tuning params for the OpenCV detector
    ArucoDisplayConfig arucoDisplay;
    ArucoCalibrationGridConfig arucoCalGrid;
    FittsBoardConfig fittsBoard;
    TouchscreenConfig touchscreen;
    DisplayConfig display;
    TelemetryConfig telemetry;
    ControllerPanelConfig controllerPanel;
    SerialConfig serial;
    ControllerConfig controllerGains;
    Cal3Config cal3;
    Cal1Config cal1;
    Cal2Config cal2;
    GestureConfig gesture;
    TargetConfig target;

   private:
    // Build cameraMatrix and distCoeffs from the scalar values after loading
    void buildDerivedCameraValues();
};
