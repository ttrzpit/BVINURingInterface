#include "Config.h"

#include <iostream>

// =============================================================================
// Config.cpp - Loads config.yaml using OpenCV FileStorage
//
// cv::FileStorage is built into OpenCV - no extra packages needed.
// Reads a YAML file written in OpenCV's dialect (starts with %YAML:1.0).
//
// Each section in the YAML maps to one of the sub-config structs.
// Fields that are missing from the file keep their default values from Config.h.
// =============================================================================


bool Config::load(const std::string& filepath) {

    cv::FileStorage fs(filepath, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        std::cerr << "Config: Could not open '" << filepath
                  << "' - all parameters using defaults.\n";
        buildDerivedCameraValues();
        return false;
    }

    // ---- Camera -------------------------------------------------------------
    cv::FileNode cam = fs["camera"];
    if (!cam.empty()) {
        cam["device"]     >> camera.device;
        cam["width"]      >> camera.width;
        cam["height"]     >> camera.height;
        cam["framerate"]  >> camera.framerate;

        // FileStorage has no native bool - read as int and convert
        int rot = 0;
        cam["rotate_180"] >> rot;
        camera.rotate180 = (rot != 0);

        cam["fx"] >> camera.fx;
        cam["fy"] >> camera.fy;
        cam["cx"] >> camera.cx;
        cam["cy"] >> camera.cy;

        cv::FileNode dist = cam["distortion"];
        if (!dist.empty() && static_cast<int>(dist.size()) == 5) {
            for (int i = 0; i < 5; i++) camera.distortion[i] = static_cast<double>(dist[i]);
        }

        cam["brightness"]  >> camera.brightness;
        cam["contrast"]    >> camera.contrast;
        cam["saturation"]  >> camera.saturation;
        cam["hue"]         >> camera.hue;

        int awb = 0;
        cam["auto_white_balance"] >> awb;
        camera.autoWhiteBalance = (awb != 0);

        cam["gamma"]          >> camera.gamma;
        cam["gain"]           >> camera.gain;
        cam["sharpness"]      >> camera.sharpness;
        cam["backlight"]      >> camera.backlight;
        cam["auto_exposure"]  >> camera.autoExposure;
        cam["exposure_level"] >> camera.exposureLevel;

        int af = 0;
        cam["auto_focus"] >> af;
        camera.autoFocus = (af != 0);

        cam["focus_level"] >> camera.focusLevel;
        cam["zoom"]        >> camera.zoom;
    }

    // ---- ArUco detection ----------------------------------------------------
    cv::FileNode ad = fs["aruco_detect"];
    if (!ad.empty()) {
        ad["marker_size_mm"] >> arucoMarker.markerSizeMm;
        ad["valid_id_min"]   >> arucoMarker.validIdMin;
        ad["valid_id_max"]   >> arucoMarker.validIdMax;
    }

    // ---- ArUco detector algorithm parameters --------------------------------
    cv::FileNode adet = fs["aruco_detector"];
    if (!adet.empty()) {
        adet["adaptive_thresh_constant"]        >> arucoDetector.adaptiveThreshConstant;
        adet["adaptive_thresh_win_size_min"]    >> arucoDetector.adaptiveThreshWinSizeMin;
        adet["adaptive_thresh_win_size_max"]    >> arucoDetector.adaptiveThreshWinSizeMax;
        adet["adaptive_thresh_win_size_step"]   >> arucoDetector.adaptiveThreshWinSizeStep;
        adet["min_marker_perimeter_rate"]       >> arucoDetector.minMarkerPerimeterRate;
        adet["max_marker_perimeter_rate"]       >> arucoDetector.maxMarkerPerimeterRate;
        adet["polygonal_approx_accuracy_rate"]  >> arucoDetector.polygonalApproxAccuracyRate;
        adet["min_corner_distance_rate"]        >> arucoDetector.minCornerDistanceRate;
        adet["min_distance_to_border"]          >> arucoDetector.minDistanceToBorder;
        adet["corner_refinement_method"]        >> arucoDetector.cornerRefinementMethod;
        adet["corner_refinement_max_iterations"] >> arucoDetector.cornerRefinementMaxIterations;
        adet["corner_refinement_min_accuracy"]  >> arucoDetector.cornerRefinementMinAccuracy;

        int inv = 0;
        adet["detect_inverted_marker"] >> inv;
        arucoDetector.detectInvertedMarker = (inv != 0);

        adet["perspective_remove_pixel_per_cell"]          >> arucoDetector.perspectiveRemovePixelPerCell;
        adet["perspective_remove_ignored_margin_per_cell"] >> arucoDetector.perspectiveRemoveIgnoredMarginPerCell;

        int a3 = 1;
        adet["use_aruco3_detection"] >> a3;
        arucoDetector.useAruco3Detection = (a3 != 0);
    }

    // ---- ArUco display (Fitts grid) -----------------------------------------
    cv::FileNode adisp = fs["aruco_display"];
    if (!adisp.empty()) {
        adisp["cols"]           >> arucoDisplay.cols;
        adisp["rows"]           >> arucoDisplay.rows;
        adisp["marker_size_mm"] >> arucoDisplay.markerSizeMm;
        adisp["padding_mm"]     >> arucoDisplay.paddingMm;
    }

    // ---- ArUco calibration grid (Cal2 / Cal3) --------------------------------
    cv::FileNode acal = fs["aruco_calibration_grid"];
    if (!acal.empty()) {
        acal["marker_size_mm"]      >> arucoCalGrid.markerSizeMm;
        acal["marker_pad_mm"]       >> arucoCalGrid.markerPadMm;
        acal["marker_exclusion_mm"] >> arucoCalGrid.markerExclusionMm;
    }

    // ---- Fitts board (explicit multi-scale placement) ------------------------
    cv::FileNode fb = fs["fitts_board_assignments"];
    if (!fb.empty()) {
        fb["fine_marker_size_mm"]    >> fittsBoard.fineMarkerSizeMm;
        fb["fine_marker_pad_mm"]     >> fittsBoard.finePadMm;
        fb["fine_marker_first_x_mm"] >> fittsBoard.fineFirstXMm;
        fb["fine_marker_first_y_mm"] >> fittsBoard.fineFirstYMm;
        fb["fine_marker_rows"]       >> fittsBoard.fineRows;
        fb["fine_marker_cols"]       >> fittsBoard.fineCols;
        fb["coarse_marker_size_mm"]  >> fittsBoard.coarseMarkerSizeMm;
        fb["coarse_marker_x1"]       >> fittsBoard.coarseCenterXMm[0];
        fb["coarse_marker_y1"]       >> fittsBoard.coarseCenterYMm[0];
        fb["coarse_marker_x2"]       >> fittsBoard.coarseCenterXMm[1];
        fb["coarse_marker_y2"]       >> fittsBoard.coarseCenterYMm[1];
        fb["coarse_marker_x3"]       >> fittsBoard.coarseCenterXMm[2];
        fb["coarse_marker_y3"]       >> fittsBoard.coarseCenterYMm[2];
        fb["coarse_marker_x4"]       >> fittsBoard.coarseCenterXMm[3];
        fb["coarse_marker_y4"]       >> fittsBoard.coarseCenterYMm[3];
        if (!fb["select_border_rows"].empty()) fb["select_border_rows"] >> fittsBoard.selectBorderRows;
        if (!fb["select_border_cols"].empty()) fb["select_border_cols"] >> fittsBoard.selectBorderCols;
        if (!fb["num_distance_bands"].empty()) fb["num_distance_bands"] >> fittsBoard.numDistanceBands;
        // Random-target exclusion list (YAML sequence of marker IDs). Still
        // rendered and detected - just removed from the random target pool.
        if (!fb["random_target_excluded_ids"].empty())
            fb["random_target_excluded_ids"] >> fittsBoard.excludedTargetIds;
    }

    // ---- Accuracy Trials (manual random-target debug pool) ------------------
    cv::FileNode at = fs["accuracy_trials"];
    if (!at.empty()) {
        if (!at["random_pool"].empty())
            at["random_pool"] >> accuracyTrials.randomPool;
    }

    // ---- Touchscreen --------------------------------------------------------
    cv::FileNode ts = fs["touchscreen"];
    if (!ts.empty()) {
        ts["width"]         >> touchscreen.width;
        ts["height"]        >> touchscreen.height;
        ts["x_offset"]      >> touchscreen.xOffset;
        ts["y_offset"]      >> touchscreen.yOffset;
        ts["pixels_per_mm"]     >> touchscreen.pixelsPerMm;
        ts["mm_per_pixel"]      >> touchscreen.mmPerPixel;
        ts["xinput_device_id"]  >> touchscreen.xinputDeviceId;
        ts["xinput_output"]     >> touchscreen.xinputOutput;
    }

    // ---- Display ------------------------------------------------------------
    cv::FileNode disp = fs["display"];
    if (!disp.empty()) {
        disp["width"]  >> display.width;
        disp["height"] >> display.height;
        disp["x_pos"]  >> display.xPos;
        disp["y_pos"]  >> display.yPos;
    }

    // ---- Telemetry ----------------------------------------------------------
    cv::FileNode tel = fs["telemetry"];
    if (!tel.empty()) {
        tel["width"]  >> telemetry.width;
        tel["height"] >> telemetry.height;
        tel["cols"]   >> telemetry.cols;
        tel["rows"]   >> telemetry.rows;
        tel["x_pos"]  >> telemetry.xPos;
        tel["y_pos"]  >> telemetry.yPos;
    }

    // ---- Controller panel ---------------------------------------------------
    cv::FileNode cp = fs["controller"];
    if (!cp.empty()) {
        cp["width"]  >> controllerPanel.width;
        cp["height"] >> controllerPanel.height;
        cp["cols"]   >> controllerPanel.cols;
        cp["rows"]   >> controllerPanel.rows;
        cp["x_pos"]  >> controllerPanel.xPos;
        cp["y_pos"]  >> controllerPanel.yPos;
    }

    // ---- Serial -------------------------------------------------------------
    cv::FileNode ser = fs["serial"];
    if (!ser.empty()) {
        ser["port"]      >> serial.port;
        ser["baud_rate"] >> serial.baudRate;
    }

    // ---- Controller gains ---------------------------------------------------
    cv::FileNode cg = fs["controller_gains"];
    if (!cg.empty()) {
        cg["gain_kP"]               >> controllerGains.gain_kP;
        cg["gain_kD"]               >> controllerGains.gain_kD;
        cg["gain_kI"]               >> controllerGains.gain_kI;
        cg["deflection_force_max"]  >> controllerGains.deflection_force_max;
        cg["tension_preload_min"]   >> controllerGains.tension_preload_min;
        cg["tension_preload_max"]   >> controllerGains.tension_preload_max;
        cg["tension_deflection_max"] >> controllerGains.tension_deflection_max;
        cg["tension_output_max"]    >> controllerGains.tension_output_max;
        cg["position_tolerance"]    >> controllerGains.position_tolerance;
        cg["lowpass_alpha"]         >> controllerGains.lowpass_alpha;
        cg["ramp_duration_secs"]    >> controllerGains.ramp_duration_secs;
        cg["max_current_amps"]      >> controllerGains.max_current_amps;
        cg["encoder_counts_per_rev"] >> controllerGains.encoder_counts_per_rev;
    }

    // ---- Cal3 timing --------------------------------------------------------
    cv::FileNode c3 = fs["cal3"];
    if (!c3.empty()) {
        c3["hold_secs"]     >> cal3.holdSecs;
        c3["cooldown_secs"] >> cal3.cooldownSecs;
        c3["max_samples"]   >> cal3.maxSamples;
    }

    // ---- Cal1 timing (AROM) --------------------------------------------------
    cv::FileNode c1 = fs["cal1"];
    if (!c1.empty()) {
        c1["record_secs"] >> cal1.recordSecs;
    }

    // ---- Cal2 timing (stiffness) ----------------------------------------------
    cv::FileNode c2 = fs["cal2"];
    if (!c2.empty()) {
        c2["force_ramp_rate"]   >> cal2.forceRampRate;
        c2["hold_secs"]         >> cal2.holdSecs;
        c2["release_wait_secs"] >> cal2.releaseWaitSecs;
    }

    // ---- Fitts target circle --------------------------------------------------
    cv::FileNode tgt = fs["target"];
    if (!tgt.empty()) {
        tgt["radius_mm"]         >> target.radiusMm;
        tgt["offset_default_mm"] >> target.offsetDefaultMm;
    }

    // ---- Gesture detection (flick up/down) -----------------------------------
    cv::FileNode ges = fs["gesture"];
    if (!ges.empty()) {
        ges["velocity_thresh_mm_s"] >> gesture.velocityThreshMmS;
        ges["arm_samples"]          >> gesture.armSamples;
        ges["min_displacement_mm"]  >> gesture.minDisplacementMm;
        ges["max_window_secs"]      >> gesture.maxWindowSecs;
        ges["cooldown_secs"]        >> gesture.cooldownSecs;
        ges["double_flick_window_secs"] >> gesture.doubleFlickWindowSecs;

        ges["rest_speed_thresh_mm_s"] >> gesture.restSpeedThreshMmS;
        ges["flick_max_motion_secs"]  >> gesture.flickMaxMotionSecs;
        ges["circle_confirm_rad"]     >> gesture.circleConfirmRad;
        ges["circle_max_window_secs"] >> gesture.circleMaxWindowSecs;
        ges["circle_min_radius_mm"]   >> gesture.circleMinRadiusMm;
        ges["circle_max_radius_mm"]   >> gesture.circleMaxRadiusMm;
        ges["circle_cooldown_secs"]   >> gesture.circleCooldownSecs;
    }

    fs.release();
    buildDerivedCameraValues();

    std::cout << "Config:       Loaded '" << filepath << "'\n";
    std::cout << "Config:       Camera " << camera.width << "x" << camera.height
              << " @ " << camera.framerate << " fps\n";
    std::cout << "Config:       ArUco grid " << arucoDisplay.cols << "x"
              << arucoDisplay.rows << ", IDs [" << arucoMarker.validIdMin
              << ", " << arucoMarker.validIdMax << "]\n";
    std::cout << "Config:       Fitts board fine " << fittsBoard.fineMarkerSizeMm
              << " mm / coarse " << fittsBoard.coarseMarkerSizeMm << " mm\n";
    return true;
}


void Config::buildDerivedCameraValues() {

    // Build 3×3 camera intrinsic matrix from the scalar parameters
    camera.cameraMatrix = (cv::Mat_<double>(3, 3)
        << camera.fx, 0.0,       camera.cx,
           0.0,       camera.fy, camera.cy,
           0.0,       0.0,       1.0);

    // Build 1×5 distortion coefficient vector
    camera.distCoeffs = (cv::Mat_<double>(1, 5)
        << camera.distortion[0],
           camera.distortion[1],
           camera.distortion[2],
           camera.distortion[3],
           camera.distortion[4]);
}
