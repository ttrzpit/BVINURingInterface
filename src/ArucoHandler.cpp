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


ArucoHandler::~ArucoHandler() { Stop(); }


// =============================================================================
// Detection thread — lifecycle and I/O
// =============================================================================

void ArucoHandler::Start() {
    detectRunning_ = true;
    detectThread_  = std::thread(&ArucoHandler::DetectLoop, this);
    std::cout << "ArucoHandler: Detection thread started.\n";
}

void ArucoHandler::Stop() {
    detectRunning_ = false;
    frameCv_.notify_all();   // Wake the thread so it can check the exit flag
    if (detectThread_.joinable()) detectThread_.join();
    std::cout << "ArucoHandler: Detection thread stopped.\n";
}

void ArucoHandler::SubmitFrame(const cv::Mat& grayFrame) {
    {
        std::lock_guard<std::mutex> lock(frameMutex_);
        // Ref-counted assignment — no pixel data copied.
        // The camera handler allocates a fresh buffer each frame, so the buffer
        // referenced here is safe to read even after the main loop moves on.
        pendingFrame_ = grayFrame;
        frameReady_   = true;
    }
    frameCv_.notify_one();
}

std::vector<DetectedMarker> ArucoHandler::GetLatestDetection() {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return latestResult_;   // ref-counted Mat copies inside, cheap
}

void ArucoHandler::DetectLoop() {
    while (detectRunning_) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(frameMutex_);
            // Sleep until a new frame arrives or Stop() signals exit
            frameCv_.wait(lock, [this]{ return frameReady_ || !detectRunning_; });
            if (!detectRunning_) break;

            // Move the frame into a local variable before releasing the lock so
            // the main loop can submit the next frame immediately — the two
            // threads never touch the same buffer at the same time.
            frame       = std::move(pendingFrame_);
            frameReady_ = false;
        }

        auto result = RunDetection(frame);

        {
            std::lock_guard<std::mutex> lock(resultMutex_);
            latestResult_ = std::move(result);
        }
    }
}


// =============================================================================
// Detection implementation (was detect())
// =============================================================================

std::vector<DetectedMarker> ArucoHandler::RunDetection(const cv::Mat& grayFrame) {

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

void ArucoHandler::SetGridVisible(bool visible) {
    if (visible == gridVisible_) return;   // No change — avoid recreating the window
    gridVisible_ = visible;

    if (visible) {
        // Fullscreen sequence on Linux — order matters.
        // Create and show first, then move, then request fullscreen.
        cv::namedWindow(TOUCHSCREEN_WIN, cv::WINDOW_NORMAL);
        cv::imshow(TOUCHSCREEN_WIN, markerGridImage_);
        cv::waitKey(1);
        cv::moveWindow(TOUCHSCREEN_WIN, touchCfg_.xOffset, touchCfg_.yOffset);
        cv::waitKey(1);
        cv::setWindowProperty(TOUCHSCREEN_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::waitKey(1);
        std::cout << "ArucoHandler: Grid shown ("
                  << displayCfg_.cols << "x" << displayCfg_.rows << ")\n";
    } else {
        cv::destroyWindow(TOUCHSCREEN_WIN);
        std::cout << "ArucoHandler: Grid hidden.\n";
    }
}

void ArucoHandler::ShowSingleMarker(int id) {
    if (id < 1) return;

    // White background — same dimensions as the full grid image
    cv::Mat img(touchCfg_.height, touchCfg_.width, CV_8UC1, cv::Scalar(255));

    // Marker size in pixels — same calculation used by renderGridImage()
    const int sz = static_cast<int>(std::round(displayCfg_.markerSizeMm * touchCfg_.pixelsPerMm));

    // Grid position for this marker ID (1-based, left-to-right top-to-bottom)
    const int col = (id - 1) % displayCfg_.cols;
    const int row = (id - 1) / displayCfg_.cols;
    cv::Point2i origin = gridCellOrigin(col, row);

    cv::Mat markerImg;
    cv::aruco::generateImageMarker(dictionary_, id, sz, markerImg, 1);
    markerImg.copyTo(img(cv::Rect(origin.x, origin.y, sz, sz)));

    cv::imshow(TOUCHSCREEN_WIN, img);
    cv::waitKey(1);

    std::cout << "ArucoHandler: Fitts target → marker " << id << "\n";
}

void ArucoHandler::updateGridConfig(int cols, int rows, float markerSizeMm, float paddingMm) {
    displayCfg_.cols         = cols;
    displayCfg_.rows         = rows;
    displayCfg_.markerSizeMm = markerSizeMm;
    displayCfg_.paddingMm    = paddingMm;
    renderGridImage();
    // Re-show only if currently visible (caller is responsible for visibility)
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
