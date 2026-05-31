#include "ArucoHandler.h"

#include <iostream>

// =============================================================================
// ArucoHandler.cpp
//
// Detector parameters were tuned for the NURing ring marker (18mm, 4x4 dict)
// under the high-resolution camera at close range. Key knobs:
//
//   adaptiveThreshWinSizeMax = 53 — large windows catch markers far from camera
//   minMarkerPerimeterRate   = 0.01 — allows small/distant markers to be detected
//   detectInvertedMarker     = true — handles reflective/glossy marker surfaces
//   CORNER_REFINE_SUBPIX     — sub-pixel corner refinement for accurate 3D pose
//
// The 3D corner template (markerCorners3D_) is built once at init using the
// physical marker size. estimatePoseSingleMarkers() uses it per-frame.
// =============================================================================

static constexpr float RAD2DEG = 57.2958f;


// ---- Construction -----------------------------------------------------------

ArucoHandler::ArucoHandler(const ArucoDetectConfig&   detectCfg,
                             const ArucoDetectorConfig& detectorCfg,
                             const ArucoDisplayConfig&  displayCfg,
                             const TouchscreenConfig&   touchCfg,
                             const cv::Mat&             camMatrix,
                             const cv::Mat&             distCoeffs)
    : detectCfg_(detectCfg)
    , detectorCfg_(detectorCfg)
    , displayCfg_(displayCfg)
    , touchCfg_(touchCfg)
    , camMatrix_(camMatrix.clone())
    , distCoeffs_(distCoeffs.clone())
{
    initDetector();
    renderGridImage();

    std::cout << "ArucoHandler: Initialized.\n"
              << "ArucoHandler: Detection range [" << detectCfg_.validIdMin
              << ", " << detectCfg_.validIdMax << "], marker size "
              << detectCfg_.markerSizeMm << " mm\n";
}


// =============================================================================
// Detection
// =============================================================================

std::vector<DetectedMarker> ArucoHandler::detect(const cv::Mat& grayFrame) {

    std::vector<DetectedMarker>           results;
    std::vector<int>                      detectedIds;
    std::vector<std::vector<cv::Point2f>> corners;

    detector_.detectMarkers(grayFrame, corners, detectedIds);

    if (detectedIds.empty()) return results;

    for (int i = 0; i < static_cast<int>(detectedIds.size()); i++) {
        int id = detectedIds[i];

        // Discard markers outside the configured valid range
        if (id < detectCfg_.validIdMin || id > detectCfg_.validIdMax) continue;

        // Wrap this single marker's corners so estimatePoseSingleMarkers can
        // accept it — the function signature expects a vector-of-corner-vectors
        std::vector<std::vector<cv::Point2f>> singleCorner = { corners[i] };

        // Pose estimation — produces one rvec and one tvec for this marker
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
            singleCorner, detectCfg_.markerSizeMm,
            camMatrix_, distCoeffs_, rvecs, tvecs);

        // Skip if pose estimation returned no result (degenerate corner geometry)
        if (tvecs.empty()) continue;

        DetectedMarker marker;
        marker.id = id;

        // Pixel-space centroid — average of the four corner coordinates
        const auto& c = corners[i];
        marker.centerPx = cv::Point2i(
            static_cast<int>((c[0].x + c[1].x + c[2].x + c[3].x) / 4.0f),
            static_cast<int>((c[0].y + c[1].y + c[2].y + c[3].y) / 4.0f));

        // Store corners
        for (int k = 0; k < 4; k++) marker.cornersPx[k] = c[k];

        // 3D position in millimetres, camera-relative.
        // Y is negated so that positive Y points upward in world space
        // (OpenCV's camera Y axis points downward by default).
        marker.positionMm = cv::Point3f(
            static_cast<float>( tvecs[0][0]),
            static_cast<float>(-tvecs[0][1]),
            static_cast<float>( tvecs[0][2]));

        // Rotation about the Y axis (most informative for a ring-worn marker)
        marker.rotationDeg = static_cast<float>(rvecs[0][1]) * RAD2DEG;

        results.push_back(marker);
    }

    return results;
}


// =============================================================================
// Touchscreen display
// =============================================================================

void ArucoHandler::showMarkerGrid() {
    // Create a normal (non-fullscreen) window, position it at the touchscreen's
    // desktop offset, and size it to exactly fill that monitor.
    // Using WINDOW_NORMAL instead of fullscreen is more reliable across
    // different Linux window managers.
    cv::namedWindow(TOUCHSCREEN_WIN, cv::WINDOW_NORMAL);
    cv::moveWindow(TOUCHSCREEN_WIN, touchCfg_.xOffset, touchCfg_.yOffset);
    cv::resizeWindow(TOUCHSCREEN_WIN, touchCfg_.width, touchCfg_.height);
    cv::imshow(TOUCHSCREEN_WIN, markerGridImage_);
    cv::waitKey(1);  // Flush window event queue so the window actually appears

    std::cout << "ArucoHandler: Touchscreen grid displayed ("
              << displayCfg_.cols << " cols x " << displayCfg_.rows << " rows)\n";
}

void ArucoHandler::updateGridConfig(int cols, int rows, int markerSizePx, int paddingPx) {
    displayCfg_.cols         = cols;
    displayCfg_.rows         = rows;
    displayCfg_.markerSizePx = markerSizePx;
    displayCfg_.paddingPx    = paddingPx;
    renderGridImage();
    showMarkerGrid();
}


// =============================================================================
// Private
// =============================================================================

void ArucoHandler::renderGridImage() {

    const int screenW = touchCfg_.width;
    const int screenH = touchCfg_.height;
    const int cols    = displayCfg_.cols;
    const int rows    = displayCfg_.rows;
    const int sz      = displayCfg_.markerSizePx;
    const int pad     = displayCfg_.paddingPx;

    // Calculate equal spacing so markers fill the usable area evenly.
    // Usable area = screen minus padding on both sides.
    // If there is only one column/row, spacing is irrelevant (no gaps needed).
    float spacingX = (cols > 1)
        ? static_cast<float>(screenW - 2 * pad - cols * sz) / (cols - 1)
        : 0.0f;
    float spacingY = (rows > 1)
        ? static_cast<float>(screenH - 2 * pad - rows * sz) / (rows - 1)
        : 0.0f;

    // Black background — grayscale image so marker images paste in cleanly
    markerGridImage_ = cv::Mat::zeros(screenH, screenW, CV_8UC1);

    // Place markers left-to-right, top-to-bottom, IDs starting at 1
    int markerID = 1;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            cv::Point2i origin = gridCellOrigin(c, r);

            cv::Mat markerImg;
            // borderBits=1 adds one quiet-zone cell (standard for ArUco)
            cv::aruco::generateImageMarker(dictionary_, markerID, sz, markerImg, 1);
            markerImg.copyTo(markerGridImage_(cv::Rect(origin.x, origin.y, sz, sz)));

            markerID++;
        }
    }

    std::cout << "ArucoHandler: Grid rendered — "
              << (markerID - 1) << " markers, spacing "
              << static_cast<int>(spacingX) << "x"
              << static_cast<int>(spacingY) << " px\n";
}

cv::Point2i ArucoHandler::gridCellOrigin(int col, int row) const {
    const int   sz  = displayCfg_.markerSizePx;
    const int   pad = displayCfg_.paddingPx;

    float spacingX = (displayCfg_.cols > 1)
        ? static_cast<float>(touchCfg_.width  - 2 * pad - displayCfg_.cols * sz) / (displayCfg_.cols - 1)
        : 0.0f;
    float spacingY = (displayCfg_.rows > 1)
        ? static_cast<float>(touchCfg_.height - 2 * pad - displayCfg_.rows * sz) / (displayCfg_.rows - 1)
        : 0.0f;

    return cv::Point2i(
        static_cast<int>(pad + col * (sz + spacingX)),
        static_cast<int>(pad + row * (sz + spacingY)));
}

void ArucoHandler::initDetector() {

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    // All values come from config.yaml [aruco_detector] — edit there, not here.
    detectorParams_.adaptiveThreshConstant        = detectorCfg_.adaptiveThreshConstant;
    detectorParams_.adaptiveThreshWinSizeMin      = detectorCfg_.adaptiveThreshWinSizeMin;
    detectorParams_.adaptiveThreshWinSizeMax      = detectorCfg_.adaptiveThreshWinSizeMax;
    detectorParams_.adaptiveThreshWinSizeStep     = detectorCfg_.adaptiveThreshWinSizeStep;
    detectorParams_.minMarkerPerimeterRate        = detectorCfg_.minMarkerPerimeterRate;
    detectorParams_.maxMarkerPerimeterRate        = detectorCfg_.maxMarkerPerimeterRate;
    detectorParams_.polygonalApproxAccuracyRate   = detectorCfg_.polygonalApproxAccuracyRate;
    detectorParams_.minCornerDistanceRate         = detectorCfg_.minCornerDistanceRate;
    detectorParams_.minDistanceToBorder           = detectorCfg_.minDistanceToBorder;
    detectorParams_.cornerRefinementMethod        =
        static_cast<cv::aruco::CornerRefineMethod>(detectorCfg_.cornerRefinementMethod);
    detectorParams_.cornerRefinementMaxIterations = detectorCfg_.cornerRefinementMaxIterations;
    detectorParams_.cornerRefinementMinAccuracy   = detectorCfg_.cornerRefinementMinAccuracy;
    detectorParams_.detectInvertedMarker          = detectorCfg_.detectInvertedMarker;

    detector_ = cv::aruco::ArucoDetector(dictionary_, detectorParams_);

    // Pre-build the 3D marker corner template in the marker's local frame.
    // Origin is the marker center; corners are at ±half in X and Y.
    float half = detectCfg_.markerSizeMm / 2.0f;
    markerCorners3D_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-half,  half, 0.0f);
    markerCorners3D_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f( half,  half, 0.0f);
    markerCorners3D_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f( half, -half, 0.0f);
    markerCorners3D_.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-half, -half, 0.0f);

    std::cout << "ArucoHandler: Detector initialized (DICT_4X4_50).\n";
}
