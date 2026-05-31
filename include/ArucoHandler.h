#pragma once

// =============================================================================
// ArucoHandler.h — ArUco marker detection and touchscreen grid display
//
// Responsibilities:
//
//   Detection: detect(grayFrame) runs the OpenCV ArUco detector on a grayscale
//              frame and returns a DetectedMarker for each ID within the
//              configured valid range. Each marker includes its pixel-space
//              center, corners, 3D pose, and rotation.
//
//   Display:   showMarkerGrid() renders a C×R grid of markers onto the
//              touchscreen window. Markers are numbered sequentially from ID 1,
//              left-to-right, top-to-bottom. Spacing is auto-calculated from
//              the screen dimensions, padding, and marker size.
//              Call updateGridConfig() to change the layout at runtime without
//              restarting.
// =============================================================================

#include <array>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "Config.h"


// ---- Output type ------------------------------------------------------------

/**
 * @brief Data for a single detected ArUco marker in a camera frame.
 */
struct DetectedMarker {
    int                        id;           ///< Marker ID from the dictionary
    cv::Point2i                centerPx;     ///< Pixel-space centroid
    std::array<cv::Point2f, 4> cornersPx;   ///< Four corners, clockwise from top-left
    cv::Point3f                positionMm;   ///< 3D position relative to camera [mm]
    float                      rotationDeg; ///< Rotation about the Y-axis [degrees]
};


// ---- Handler ----------------------------------------------------------------

class ArucoHandler {
public:
    /**
     * @param detectCfg    Detection tuning: marker size and valid ID range
     * @param detectorCfg  OpenCV detector algorithm parameters (thresholds, refinement, etc.)
     * @param displayCfg   Grid layout for the touchscreen
     * @param touchCfg     Touchscreen dimensions and desktop offset (for window placement)
     * @param camMatrix    3×3 camera intrinsic matrix (from Config)
     * @param distCoeffs   1×5 distortion coefficients (from Config)
     */
    ArucoHandler(const ArucoDetectConfig&   detectCfg,
                 const ArucoDetectorConfig& detectorCfg,
                 const ArucoDisplayConfig&  displayCfg,
                 const TouchscreenConfig&   touchCfg,
                 const cv::Mat&             camMatrix,
                 const cv::Mat&             distCoeffs);

    // ---- Detection ----------------------------------------------------------

    /**
     * @brief Detect ArUco markers in a grayscale camera frame.
     * @param grayFrame  Grayscale image from CameraHandler
     * @return Markers found within [validIdMin, validIdMax], empty if none
     */
    std::vector<DetectedMarker> detect(const cv::Mat& grayFrame);

    // ---- Touchscreen display ------------------------------------------------

    /**
     * @brief Open the touchscreen window and render the current marker grid.
     *        Call once at startup; the window persists until the program exits.
     */
    void showMarkerGrid();

    /**
     * @brief Change the grid layout parameters and immediately re-render.
     *        Useful for adjusting the display without restarting the program.
     */
    void updateGridConfig(int cols, int rows, float markerSizeMm, float paddingMm);

private:
    void        initDetector();        // Configure detector parameters
    void        renderGridImage();     // Build markerGridImage_ from displayCfg_
    cv::Point2i gridCellOrigin(int col, int row) const; // Top-left px of a grid cell

    // Configuration (display config is mutable — can change via updateGridConfig)
    ArucoDetectConfig   detectCfg_;
    ArucoDetectorConfig detectorCfg_;
    ArucoDisplayConfig  displayCfg_;
    TouchscreenConfig   touchCfg_;
    cv::Mat            camMatrix_;
    cv::Mat            distCoeffs_;

    // OpenCV ArUco detector
    cv::aruco::Dictionary        dictionary_;
    cv::aruco::DetectorParameters detectorParams_;
    cv::aruco::ArucoDetector     detector_;

    // Pre-built 3D corner template used for every pose estimation call
    // Avoids recomputing the same geometry every frame
    cv::Mat markerCorners3D_{ 4, 1, CV_32FC3 };

    // Rendered grid image — pushed to the touchscreen window
    cv::Mat markerGridImage_;

    static constexpr const char* TOUCHSCREEN_WIN = "ArUco Display";
};
