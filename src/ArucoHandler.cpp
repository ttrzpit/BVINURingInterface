#include "ArucoHandler.h"

#include <iomanip>
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
    // Fullscreen sequence on Linux — order matters.
    // The window must be created, shown, and moved to the target monitor
    // BEFORE requesting fullscreen. If fullscreen is set first, the WM may
    // fullscreen the window on the primary display instead of the touchscreen.

    cv::namedWindow(TOUCHSCREEN_WIN, cv::WINDOW_NORMAL);

    // Show the image first so the window physically exists in the WM
    cv::imshow(TOUCHSCREEN_WIN, markerGridImage_);
    cv::waitKey(1);

    // Move to the touchscreen monitor, then give the WM a moment to process it
    cv::moveWindow(TOUCHSCREEN_WIN, touchCfg_.xOffset, touchCfg_.yOffset);
    cv::waitKey(1);

    // Now request fullscreen — the WM will fullscreen it on whichever monitor
    // the window is currently on (the touchscreen, after the move above)
    cv::setWindowProperty(TOUCHSCREEN_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::waitKey(1);

    std::cout << "ArucoHandler: Touchscreen grid displayed fullscreen ("
              << displayCfg_.cols << " cols x " << displayCfg_.rows << " rows)\n";
}

void ArucoHandler::updateGridConfig(int cols, int rows, float markerSizeMm, float paddingMm) {
    displayCfg_.cols         = cols;
    displayCfg_.rows         = rows;
    displayCfg_.markerSizeMm = markerSizeMm;
    displayCfg_.paddingMm    = paddingMm;
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

    // Convert mm values to pixels using the touchscreen's physical pixel density
    const int sz  = static_cast<int>(std::round(displayCfg_.markerSizeMm * touchCfg_.pixelsPerMm));
    const int pad = static_cast<int>(std::round(displayCfg_.paddingMm    * touchCfg_.pixelsPerMm));

    // Space markers evenly within the usable area (screen minus equal padding on all 4 sides)
    float spacingX = (cols > 1)
        ? static_cast<float>(screenW - 2 * pad - cols * sz) / (cols - 1)
        : 0.0f;
    float spacingY = (rows > 1)
        ? static_cast<float>(screenH - 2 * pad - rows * sz) / (rows - 1)
        : 0.0f;

    // White background
    markerGridImage_ = cv::Mat(screenH, screenW, CV_8UC1, cv::Scalar(255));

    // Place markers left-to-right, top-to-bottom, IDs starting at 1
    int markerID = 1;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            cv::Point2i origin = gridCellOrigin(c, r);
            cv::Mat markerImg;
            cv::aruco::generateImageMarker(dictionary_, markerID, sz, markerImg, 1);
            markerImg.copyTo(markerGridImage_(cv::Rect(origin.x, origin.y, sz, sz)));
            markerID++;
        }
    }

    // Derived values for the console report
    float c2cX_mm = (sz + spacingX) * touchCfg_.mmPerPixel;
    float c2cY_mm = (sz + spacingY) * touchCfg_.mmPerPixel;
    float gapX_mm = spacingX        * touchCfg_.mmPerPixel;
    float gapY_mm = spacingY        * touchCfg_.mmPerPixel;

    std::cout << std::fixed << std::setprecision(2)
              << "ArucoHandler: Grid rendered — "
              << cols << " cols x " << rows << " rows = " << (markerID - 1) << " markers\n"
              << "  Marker size:              " << displayCfg_.markerSizeMm       << " mm  |  " << sz                          << " px\n"
              << "  H spacing (center-ctr):   " << c2cX_mm                       << " mm  |  " << static_cast<int>(sz + spacingX) << " px\n"
              << "  H spacing (edge-edge):    " << gapX_mm                        << " mm  |  " << static_cast<int>(spacingX)     << " px\n"
              << "  V spacing (center-ctr):   " << c2cY_mm                       << " mm  |  " << static_cast<int>(sz + spacingY) << " px\n"
              << "  V spacing (edge-edge):    " << gapY_mm                        << " mm  |  " << static_cast<int>(spacingY)     << " px\n"
              << "  Padding (all sides):      " << displayCfg_.paddingMm          << " mm  |  " << pad                           << " px\n"
              << std::defaultfloat;
}

cv::Point2i ArucoHandler::gridCellOrigin(int col, int row) const {
    const int sz  = static_cast<int>(std::round(displayCfg_.markerSizeMm * touchCfg_.pixelsPerMm));
    const int pad = static_cast<int>(std::round(displayCfg_.paddingMm    * touchCfg_.pixelsPerMm));

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
    detectorParams_.detectInvertedMarker                  = detectorCfg_.detectInvertedMarker;
    detectorParams_.perspectiveRemovePixelPerCell         = detectorCfg_.perspectiveRemovePixelPerCell;
    detectorParams_.perspectiveRemoveIgnoredMarginPerCell = detectorCfg_.perspectiveRemoveIgnoredMarginPerCell;
    detectorParams_.useAruco3Detection                    = detectorCfg_.useAruco3Detection;

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
