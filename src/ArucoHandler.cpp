#include "ArucoHandler.h"

#include "Colors.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>

// =============================================================================
// ArucoHandler.cpp
//
// Detector parameters were tuned for the NURing ring marker (18mm, 4x4 dict)
// under the high-resolution camera at close range. Key knobs:
//
//   adaptiveThreshWinSizeMax = 53 - large windows catch markers far from camera
//   minMarkerPerimeterRate   = 0.01 - allows small/distant markers to be detected
//   detectInvertedMarker     = true - handles reflective/glossy marker surfaces
//   CORNER_REFINE_SUBPIX     - sub-pixel corner refinement for accurate 3D pose
// =============================================================================

static constexpr float RAD2DEG = 57.2958f;

// ---- Construction -----------------------------------------------------------

ArucoHandler::ArucoHandler(const ArucoMarkerConfig& markerCfg,
                           const ArucoDetectorConfig& detectorCfg,
                           const ArucoDisplayConfig& displayCfg,
                           const ArucoCalibrationGridConfig& calGridCfg,
                           const FittsBoardConfig& fittsBoardCfg,
                           const TouchscreenConfig& touchCfg,
                           const cv::Mat& camMatrix,
                           const cv::Mat& distCoeffs)
    : detectCfg_(markerCfg), detectorCfg_(detectorCfg), displayCfg_(displayCfg),
      calGridCfg_(calGridCfg), touchCfg_(touchCfg),
      fittsLayout_(fittsBoardCfg, touchCfg),
      camMatrix_(camMatrix.clone()), distCoeffs_(distCoeffs.clone()),
      calGridDictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000)) {
    initDetector();
    renderGridImage();
    renderCalibrationGridImage();
    renderFittsBoardImage();

    std::cout << "ArucoHandler: Initialized.\n"
              << "ArucoHandler: Detection range [" << detectCfg_.validIdMin
              << ", " << detectCfg_.validIdMax << "], marker size "
              << detectCfg_.markerSizeMm << " mm\n";
}

ArucoHandler::~ArucoHandler() { Stop(); }

// =============================================================================
// Detection thread - lifecycle and I/O
// =============================================================================

void ArucoHandler::Start() {
    detectRunning_ = true;
    detectThread_ = std::thread(&ArucoHandler::DetectLoop, this);
    std::cout << "ArucoHandler: Detection thread started.\n";
}

void ArucoHandler::Stop() {
    detectRunning_ = false;
    frameCv_.notify_all();  // Wake the thread so it can check the exit flag
    if (detectThread_.joinable()) detectThread_.join();
    std::cout << "ArucoHandler: Detection thread stopped.\n";
}

void ArucoHandler::SubmitFrame(const cv::Mat& grayFrame, double timestamp) {
    {
        std::lock_guard<std::mutex> lock(frameMutex_);
        // Ref-counted assignment - no pixel data copied.
        // The camera handler allocates a fresh buffer each frame, so the buffer
        // referenced here is safe to read even after the main loop moves on.
        pendingFrame_ = grayFrame;
        pendingTimestamp_ = timestamp;
        frameReady_ = true;
    }
    frameCv_.notify_one();
}

std::vector<DetectedMarker> ArucoHandler::GetLatestDetection() {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return latestResult_;  // ref-counted Mat copies inside, cheap
}

void ArucoHandler::DetectLoop() {
    // Rolling 1-second Hz counter (mirrors the pattern in DisplayHandler).
    int    freqCount       = 0;
    double freqWindowStart = 0.0;
    bool   freqInit        = false;

    while (detectRunning_) {
        double  submitTimestamp;
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(frameMutex_);
            // Sleep until a new frame arrives or Stop() signals exit.
            frameCv_.wait(lock, [this] { return frameReady_ || !detectRunning_; });
            if (!detectRunning_) break;

            // Move the frame into a local variable before releasing the lock so
            // the main loop can submit the next frame immediately - the two
            // threads never touch the same buffer at the same time.
            frame           = std::move(pendingFrame_);
            submitTimestamp = pendingTimestamp_;
            frameReady_     = false;
        }

        // Wall time before detection — used for Hz window and lag.
        const double tStart =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();

        if (!freqInit) { freqWindowStart = tStart; freqInit = true; }

        auto result = RunDetection(frame);

        const double tEnd =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();

        {
            std::lock_guard<std::mutex> lock(resultMutex_);
            latestResult_ = std::move(result);
        }
        resultTimestamp_ = submitTimestamp;

        // --- Hz measurement --------------------------------------------------
        freqCount++;
        const double elapsed = tEnd - freqWindowStart;
        if (elapsed >= 1.0) {
            detectionHz_ = static_cast<float>(freqCount / elapsed);
            freqCount        = 0;
            freqWindowStart  = tEnd;
        }

        // --- Lag measurement -------------------------------------------------
        // After RunDetection() finishes, if pendingFrame_ already holds a newer
        // frame, the gap between that frame's capture time and the one we just
        // processed is the detection lag — i.e. how far behind the camera the
        // detector has fallen.
        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            if (frameReady_ && pendingTimestamp_ > submitTimestamp)
                detectionLagMs_ = static_cast<float>((pendingTimestamp_ - submitTimestamp) * 1000.0);
            else
                detectionLagMs_ = 0.0f;   // caught up
        }
    }
}

// =============================================================================
// Detection implementation (was detect())
// =============================================================================

std::vector<DetectedMarker> ArucoHandler::RunDetection(const cv::Mat& grayFrame) {
    std::vector<DetectedMarker> results;
    std::vector<int> detectedIds;
    std::vector<std::vector<cv::Point2f>> corners;

    const cv::aruco::ArucoDetector& det =
        useCalDetector_.load() ? calDetector_ : detector_;

    // Phase 1 diagnostic: time the detectMarkers() call in isolation - it is the
    // dominant per-frame cost and scales with the number of markers decoded.
    const auto detT0 = std::chrono::steady_clock::now();
    det.detectMarkers(grayFrame, corners, detectedIds);
    const auto detT1 = std::chrono::steady_clock::now();
    detectMarkersMs_.store(std::chrono::duration<float, std::milli>(detT1 - detT0).count());
    markerCount_.store(static_cast<int>(detectedIds.size()));

    // Reset per-frame pose-solve time; set below only if the active target is solved.
    float poseSolveMs = 0.0f;

    if (detectedIds.empty()) { poseSolveMs_.store(0.0f); return results; }

    const int idMin = activeValidIdMin_.load();
    const int idMax = activeValidIdMax_.load();

    const int activeId = activeTagId_.load();

    for (int i = 0; i < static_cast<int>(detectedIds.size()); i++) {
        int id = detectedIds[i];

        // Discard markers outside the active valid range
        if (id < idMin || id > idMax) continue;

        const bool isActiveTarget = (id == activeId);

        // Subpixel-refine ONLY the active target's corners. Global refinement is
        // disabled (see initDetector) to keep detectMarkers() fast on the dense
        // board; the target is the one marker whose 3D pose (and thus precise
        // corners) is consumed. Guard against the cornerSubPix window reaching
        // outside the image - markers are only kept ~min_distance_to_border px
        // from the edge, which is less than the refine window radius. Refining
        // before centerPx/cornersPx/rollRad are read below feeds them all the
        // refined corners.
        if (isActiveTarget && refineActiveTargetCorners_ && !grayFrame.empty()) {
            constexpr int win    = 5;        // cornerSubPix half-window (OpenCV aruco default)
            constexpr int margin = win + 1;
            bool          safe   = true;
            for (const auto& pt : corners[i]) {
                if (pt.x < margin || pt.y < margin ||
                    pt.x >= grayFrame.cols - margin || pt.y >= grayFrame.rows - margin) {
                    safe = false;
                    break;
                }
            }
            if (safe) {
                cv::cornerSubPix(
                    grayFrame, corners[i],
                    cv::Size(win, win), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                                     detectorCfg_.cornerRefinementMaxIterations,
                                     detectorCfg_.cornerRefinementMinAccuracy));
            }
        }

        DetectedMarker marker;
        marker.id = id;

        // Pixel-space centroid - average of the four corner coordinates
        const auto& c = corners[i];
        marker.centerPx = cv::Point2i(
            static_cast<int>((c[0].x + c[1].x + c[2].x + c[3].x) / 4.0f),
            static_cast<int>((c[0].y + c[1].y + c[2].y + c[3].y) / 4.0f));

        // Store corners
        for (int k = 0; k < 4; k++) marker.cornersPx[k] = c[k];

        // 3D pose (positionMm / rotationDeg) is only ever consumed for the
        // active guidance target, so only solve it for that one marker. On the
        // dense Fitts board this avoids ~120 wasted solvePnP calls per frame
        // (which slowed detection throughput and made the overlay refresh choppy).
        // All other markers still carry centerPx / corners / rollRad below.
        if (isActiveTarget) {
            // Physical marker size for pose. On the multi-scale Fitts board the
            // coarse and fine markers have different sizes, so resolve per-ID
            // from the layout (rendered pixel extent → mm); otherwise use the
            // single configured ring-marker size. A wrong size would scale this
            // marker's depth, breaking guidance.
            float markerSizeMm = detectCfg_.markerSizeMm;
            if (useFittsBoardSizes_.load()) {
                if (const FittsMarker* fm = fittsLayout_.Find(id)) {
                    markerSizeMm = fm->sizePx * touchCfg_.mmPerPixel;
                }
            }

            std::vector<std::vector<cv::Point2f>> singleCorner = {corners[i]};
            std::vector<cv::Vec3d> rvecs, tvecs;
            const auto poseT0 = std::chrono::steady_clock::now();
            cv::aruco::estimatePoseSingleMarkers(
                singleCorner, markerSizeMm, camMatrix_, distCoeffs_, rvecs, tvecs);
            const auto poseT1 = std::chrono::steady_clock::now();
            poseSolveMs += std::chrono::duration<float, std::milli>(poseT1 - poseT0).count();

            if (!tvecs.empty()) {
                // 3D position in millimetres, camera-relative. Y is negated so
                // positive Y points upward in world space (OpenCV's camera Y
                // axis points downward by default).
                marker.positionMm = cv::Point3f(
                    static_cast<float>(tvecs[0][0]),
                    static_cast<float>(-tvecs[0][1]),
                    static_cast<float>(tvecs[0][2]));
                marker.rotationDeg = static_cast<float>(rvecs[0][1]) * RAD2DEG;
            }
        }

        // In-plane roll from the marker's top edge in the image (top-left ->
        // top-right corner). Sub-pixel corners make this stable, unlike rvec[2]
        // (the Rodrigues Z-component), which flips between the two planar-pose
        // solutions and makes the roll jump erratically. All screen markers are
        // axis-aligned, so this angle measures the camera's roll directly. Used
        // to compute the roll delta vs Cal3's rollReference_ so the fingertip
        // offset can be rotated to match the current finger roll.
        marker.rollRad = std::atan2(c[1].y - c[0].y, c[1].x - c[0].x);

        results.push_back(marker);
    }

    poseSolveMs_.store(poseSolveMs);
    return results;
}

// =============================================================================
// Touchscreen display
// =============================================================================

void ArucoHandler::ShowBlankTouchscreen() {
    if (!gridVisible_) {
        // Open and position the window using the same fullscreen sequence.
        // The 50 ms wait after moveWindow gives the X11 window manager time
        // to register the new position *before* fullscreen is requested -
        // otherwise the WM can record the pre-move (wrong-display) geometry
        // as the "restore" position, so un-fullscreening later (SetGridVisible(false))
        // snaps the window back to the wrong display and steals input focus.
        cv::namedWindow(TOUCHSCREEN_WIN, cv::WINDOW_NORMAL);
        cv::waitKey(1);
        cv::moveWindow(TOUCHSCREEN_WIN, touchCfg_.xOffset, touchCfg_.yOffset);
        cv::waitKey(50);
        cv::setWindowProperty(TOUCHSCREEN_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::waitKey(50);
        gridVisible_ = true;
    }
    // Solid white - participant sees a clean screen before the first target is chosen
    cv::Mat blank(touchCfg_.height, touchCfg_.width, CV_8UC1, cv::Scalar(255));
    cv::imshow(TOUCHSCREEN_WIN, blank);
    cv::waitKey(1);

    // Reset the Fitts overlay base image and state - no overlay until a touch
    // is recorded against the next target.
    cv::cvtColor(blank, singleMarkerImage_, cv::COLOR_GRAY2BGR);
    fittsOverlayVisible_ = false;
    fittsTouchPx_        = {};
    fittsOverlayTargetId_ = 0;
    fittsLine1_.clear();
    fittsLine2_.clear();
    targetCircleVisible_ = false;

    std::cout << "ArucoHandler: Touchscreen blank (FITTS ready - press 'r' for first target)\n";
}

void ArucoHandler::SetGridVisible(bool visible) {
    if (visible == gridVisible_) return;  // No change - avoid recreating the window
    gridVisible_ = visible;

    if (visible) {
        // Fullscreen sequence on Linux - order matters.
        // Create and show first, then move, then request fullscreen. The
        // 50 ms waits around moveWindow/setWindowProperty give the X11 WM
        // time to register each step before the next - see ShowBlankTouchscreen().
        cv::namedWindow(TOUCHSCREEN_WIN, cv::WINDOW_NORMAL);
        cv::imshow(TOUCHSCREEN_WIN, markerGridImage_);
        cv::waitKey(1);
        cv::moveWindow(TOUCHSCREEN_WIN, touchCfg_.xOffset, touchCfg_.yOffset);
        cv::waitKey(50);
        cv::setWindowProperty(TOUCHSCREEN_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::waitKey(50);
        std::cout << "ArucoHandler: Grid shown ("
                  << displayCfg_.cols << "x" << displayCfg_.rows << ")\n";
    } else {
        // Un-fullscreen before destroying - skipping this step causes a heap
        // corruption crash in cv::Mat::deallocate on Linux/X11 when the window
        // manager still holds a reference to the fullscreen surface.
        cv::setWindowProperty(TOUCHSCREEN_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
        cv::waitKey(50);
        cv::destroyWindow(TOUCHSCREEN_WIN);
        cv::waitKey(50);
        std::cout << "ArucoHandler: Grid hidden.\n";
    }
}

void ArucoHandler::ShowSingleMarker(int id) {
    if (id < 1) return;

    // White background - same dimensions as the full grid image
    cv::Mat img(touchCfg_.height, touchCfg_.width, CV_8UC1, cv::Scalar(255));

    // Marker size in pixels - same calculation used by renderGridImage()
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

    // New target - reset the Fitts overlay base image and clear any previous
    // touch sample until the next contact is recorded.
    cv::cvtColor(img, singleMarkerImage_, cv::COLOR_GRAY2BGR);
    fittsOverlayVisible_ = false;
    fittsTouchPx_        = {};
    fittsOverlayTargetId_ = 0;
    fittsLine1_.clear();
    fittsLine2_.clear();
    targetCircleVisible_ = false;

    std::cout << "ArucoHandler: Fitts target → marker " << id << "\n";
}

void ArucoHandler::SetFittsOverlay(bool visible, cv::Point2i touchPx, int targetId,
                                   const std::string& line1, const std::string& line2) {
    if (visible == fittsOverlayVisible_ && touchPx == fittsTouchPx_ &&
        targetId == fittsOverlayTargetId_ &&
        line1 == fittsLine1_ && line2 == fittsLine2_) {
        return;  // No change - avoid redundant redraw
    }

    fittsOverlayVisible_  = visible;
    fittsTouchPx_         = touchPx;
    fittsOverlayTargetId_ = targetId;
    fittsLine1_           = line1;
    fittsLine2_           = line2;

    RedrawTouchscreenOverlay();
}

void ArucoHandler::SetTargetOffsetCircle(bool visible, cv::Point2i centerPx, int radiusPx, cv::Scalar color) {
    bool colorChanged = color[0] != targetCircleColor_[0] || color[1] != targetCircleColor_[1] ||
                        color[2] != targetCircleColor_[2];
    if (visible == targetCircleVisible_ && centerPx == targetCirclePx_ &&
        radiusPx == targetCircleRadiusPx_ && !colorChanged) {
        return;  // No change - avoid redundant redraw
    }

    targetCircleVisible_  = visible;
    targetCirclePx_       = centerPx;
    targetCircleRadiusPx_ = radiusPx;
    targetCircleColor_    = color;

    RedrawTouchscreenOverlay();
}

void ArucoHandler::SetTargetOutline(bool visible, int targetId) {
    if (visible == targetOutlineVisible_ && targetId == targetOutlineId_) {
        return;  // No change - avoid redundant redraw
    }
    targetOutlineVisible_ = visible;
    targetOutlineId_      = targetId;

    RedrawTouchscreenOverlay();
}

void ArucoHandler::RedrawTouchscreenOverlay() {
    if (singleMarkerImage_.empty()) return;

    cv::Mat img = singleMarkerImage_.clone();

    if (targetCircleVisible_) {
        // cv::circle(img, targetCirclePx_, targetCircleRadiusPx_, targetCircleColor_, 2);  // hollow
    }

    // Magenta reference outline around the just-touched target marker.
    if (targetOutlineVisible_) {
        if (const FittsMarker* fm = fittsLayout_.Find(targetOutlineId_)) {
            cv::rectangle(img, cv::Rect(fm->xPx, fm->yPx, fm->sizePx, fm->sizePx),
                          Colors::MagMd, 3);
        }
    }

    if (fittsOverlayVisible_) {
        // Error triangle: from the touch point (fingertip endpoint) to the
        // target marker centre. The right-angle corner sits at the touch X /
        // target Y, so the green leg is the vertical error and the red leg is
        // the horizontal error; the black hypotenuse is the direct error.
        const cv::Point2i target = GetGridMarkerCenterPx(fittsOverlayTargetId_);
        if (target != cv::Point2i{}) {
            const cv::Point2i corner(fittsTouchPx_.x, target.y);
            cv::line(img, fittsTouchPx_, corner, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);  // vertical error (green)
            cv::line(img, corner, target,        cv::Scalar(0, 0, 255), 2, cv::LINE_AA);  // horizontal error (red)
            cv::line(img, fittsTouchPx_, target,  cv::Scalar(0, 0, 0),   2, cv::LINE_AA);  // direct error (black)
        }

        int touchRadiusPx = static_cast<int>(std::round(2.5f * touchCfg_.pixelsPerMm));
        cv::circle(img, fittsTouchPx_, touchRadiusPx, cv::Scalar(255, 0, 0), -1);  // blue filled, 2.5mm radius

        // Endpoint-error readout along the bottom of the screen, under the markers.
        cv::putText(img, fittsLine1_, cv::Point2i(300, img.rows - 60), cv::FONT_HERSHEY_SIMPLEX,
                    0.7, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(img, fittsLine2_, cv::Point2i(300, img.rows - 20), cv::FONT_HERSHEY_SIMPLEX,
                    0.7, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    }

    cv::imshow(TOUCHSCREEN_WIN, img);
    cv::waitKey(1);
}

cv::Point2i ArucoHandler::GetGridMarkerCenterPx(int id) const {
    const FittsMarker* m = fittsLayout_.Find(id);
    if (!m) return {};
    return cv::Point2i(m->xPx + m->sizePx / 2, m->yPx + m->sizePx / 2);
}

void ArucoHandler::updateGridConfig(int cols, int rows, float markerSizeMm, float paddingMm) {
    displayCfg_.cols = cols;
    displayCfg_.rows = rows;
    displayCfg_.markerSizeMm = markerSizeMm;
    displayCfg_.paddingMm = paddingMm;
    renderGridImage();
    // Re-show only if currently visible (caller is responsible for visibility)
}

void ArucoHandler::SetCalibrationGridVisible(bool visible) {
    if (visible == calGridVisible_) return;
    calGridVisible_ = visible;

    if (visible) {
        // Fullscreen sequence - must match SetGridVisible order exactly.
        // create → show → waitKey → move → waitKey(50) → fullscreen → waitKey(50)
        cv::namedWindow(TOUCHSCREEN_WIN, cv::WINDOW_NORMAL);
        cv::imshow(TOUCHSCREEN_WIN, calGridImage_);
        cv::waitKey(1);
        cv::moveWindow(TOUCHSCREEN_WIN, touchCfg_.xOffset, touchCfg_.yOffset);
        cv::waitKey(50);
        cv::setWindowProperty(TOUCHSCREEN_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::waitKey(50);
        std::cout << "ArucoHandler: Calibration grid shown ("
                  << calGridSize_.width << "x" << calGridSize_.height << " markers)\n";
    } else {
        cv::setWindowProperty(TOUCHSCREEN_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
        cv::waitKey(50);
        cv::destroyWindow(TOUCHSCREEN_WIN);
        cv::waitKey(50);
        std::cout << "ArucoHandler: Calibration grid hidden\n";
    }
}

void ArucoHandler::SetCalibrationDetection(bool calibration) {
    if (calibration) {
        activeValidIdMin_.store(0);
        activeValidIdMax_.store(999);
        useCalDetector_.store(true);
        std::cout << "ArucoHandler: Detection → DICT_4X4_1000 (IDs 0–999)\n";
    } else {
        useCalDetector_.store(false);
        activeValidIdMin_.store(detectCfg_.validIdMin);
        activeValidIdMax_.store(detectCfg_.validIdMax);
        std::cout << "ArucoHandler: Detection → DICT_4X4_50 (IDs "
                  << detectCfg_.validIdMin << "–" << detectCfg_.validIdMax << ")\n";
    }
}

void ArucoHandler::SetFittsBoardVisible(bool visible) {
    if (visible == fittsBoardVisible_) return;  // No change - avoid recreating the window
    fittsBoardVisible_ = visible;

    if (visible) {
        // Fullscreen sequence - must match SetGridVisible order exactly.
        cv::namedWindow(TOUCHSCREEN_WIN, cv::WINDOW_NORMAL);
        cv::imshow(TOUCHSCREEN_WIN, fittsBoardImage_);
        cv::waitKey(1);
        cv::moveWindow(TOUCHSCREEN_WIN, touchCfg_.xOffset, touchCfg_.yOffset);
        cv::waitKey(50);
        cv::setWindowProperty(TOUCHSCREEN_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::waitKey(50);

        // Seed the overlay base image so the target-offset circle and Fitts
        // touch overlay draw on top of the board (RedrawTouchscreenOverlay).
        cv::cvtColor(fittsBoardImage_, singleMarkerImage_, cv::COLOR_GRAY2BGR);
        fittsOverlayVisible_ = false;
        fittsTouchPx_        = {};
        fittsOverlayTargetId_ = 0;
        fittsLine1_.clear();
        fittsLine2_.clear();
        targetCircleVisible_ = false;

        std::cout << "ArucoHandler: Fitts board shown ("
                  << fittsLayout_.FineCount() << " fine targets + 4 coarse)\n";
    } else {
        cv::setWindowProperty(TOUCHSCREEN_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
        cv::waitKey(50);
        cv::destroyWindow(TOUCHSCREEN_WIN);
        cv::waitKey(50);
        std::cout << "ArucoHandler: Fitts board hidden\n";
    }
}

void ArucoHandler::SetFittsBoardDetection(bool fitts) {
    if (fitts) {
        useCalDetector_.store(true);          // DICT_4X4_1000 (coarse + fine bands)
        useFittsBoardSizes_.store(true);      // per-ID physical size from layout
        activeValidIdMin_.store(0);
        activeValidIdMax_.store(fittsLayout_.MaxId());
        std::cout << "ArucoHandler: Detection → Fitts board (DICT_4X4_1000, IDs 0–"
                  << fittsLayout_.MaxId() << ", per-ID sizing)\n";
    } else {
        useFittsBoardSizes_.store(false);
        useCalDetector_.store(false);
        activeValidIdMin_.store(detectCfg_.validIdMin);
        activeValidIdMax_.store(detectCfg_.validIdMax);
        std::cout << "ArucoHandler: Detection → DICT_4X4_50 (IDs "
                  << detectCfg_.validIdMin << "–" << detectCfg_.validIdMax << ")\n";
    }
}

// =============================================================================
// Private
// =============================================================================

void ArucoHandler::renderFittsBoardImage() {
    fittsBoardImage_ = cv::Mat(touchCfg_.height, touchCfg_.width, CV_8UC1, cv::Scalar(255));

    for (const auto& m : fittsLayout_.Markers()) {
        cv::Mat markerImg;
        cv::aruco::generateImageMarker(calGridDictionary_, m.id, m.sizePx, markerImg, 1);
        markerImg.copyTo(fittsBoardImage_(cv::Rect(m.xPx, m.yPx, m.sizePx, m.sizePx)));
    }

    const float fineSzMm   = (fittsLayout_.FineCount() > 0)
        ? fittsLayout_.Markers().back().sizePx * touchCfg_.mmPerPixel : 0.f;
    std::cout << std::fixed << std::setprecision(1)
              << "ArucoHandler: Fitts board rendered (DICT_4X4_1000) - "
              << fittsLayout_.FineCount() << " fine targets (IDs "
              << fittsLayout_.FineIdMin() << "–" << fittsLayout_.FineIdMax() << ", "
              << fineSzMm << " mm) + 4 coarse perimeter markers\n"
              << std::defaultfloat;
}

void ArucoHandler::renderCalibrationGridImage() {
    const int screenW    = touchCfg_.width;
    const int screenH    = touchCfg_.height;

    // Convert mm config values to pixels
    const int sz  = static_cast<int>(std::round(calGridCfg_.markerSizeMm      * touchCfg_.pixelsPerMm));
    const int gap = static_cast<int>(std::round(calGridCfg_.markerPadMm       * touchCfg_.pixelsPerMm));
    const int exc = static_cast<int>(std::round(calGridCfg_.markerExclusionMm * touchCfg_.pixelsPerMm));

    // Available area inside the exclusion boundary
    const int availW = screenW - 2 * exc;
    const int availH = screenH - 2 * exc;

    // Auto-calculate cols/rows: pack markers with fixed gap between them
    // n markers need: n*sz + (n-1)*gap <= avail  →  n <= (avail + gap) / (sz + gap)
    const int cols = std::max(1, (availW + gap) / (sz + gap));
    const int rows = std::max(1, (availH + gap) / (sz + gap));
    calGridSize_ = cv::Size(cols, rows);

    // White background
    calGridImage_ = cv::Mat(screenH, screenW, CV_8UC1, cv::Scalar(255));

    // Place markers: origin of marker (c,r) is at (exc + c*(sz+gap), exc + r*(sz+gap))
    int markerID = 0;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int originX = exc + c * (sz + gap);
            int originY = exc + r * (sz + gap);
            cv::Mat markerImg;
            cv::aruco::generateImageMarker(calGridDictionary_, markerID, sz, markerImg, 1);
            markerImg.copyTo(calGridImage_(cv::Rect(originX, originY, sz, sz)));
            markerID++;
        }
    }

    // Derived values for the console report
    float szMm  = calGridCfg_.markerSizeMm;
    float gapMm = calGridCfg_.markerPadMm;
    float excMm = calGridCfg_.markerExclusionMm;

    std::cout << std::fixed << std::setprecision(1)
              << "ArucoHandler: Calibration grid rendered (DICT_4X4_1000) - "
              << cols << " cols x " << rows << " rows = " << markerID << " markers\n"
              << "  Marker size:     " << szMm  << " mm  |  " << sz  << " px\n"
              << "  Marker gap:      " << gapMm << " mm  |  " << gap << " px\n"
              << "  Exclusion zone:  " << excMm << " mm  |  " << exc << " px\n"
              << std::defaultfloat;
}

void ArucoHandler::renderGridImage() {
    const int screenW = touchCfg_.width;
    const int screenH = touchCfg_.height;
    const int cols = displayCfg_.cols;
    const int rows = displayCfg_.rows;

    // Convert mm values to pixels using the touchscreen's physical pixel density
    const int sz = static_cast<int>(std::round(displayCfg_.markerSizeMm * touchCfg_.pixelsPerMm));
    const int pad = static_cast<int>(std::round(displayCfg_.paddingMm * touchCfg_.pixelsPerMm));

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
    float gapX_mm = spacingX * touchCfg_.mmPerPixel;
    float gapY_mm = spacingY * touchCfg_.mmPerPixel;

    std::cout << std::fixed << std::setprecision(2)
              << "ArucoHandler: Grid rendered - "
              << cols << " cols x " << rows << " rows = " << (markerID - 1) << " markers\n"
              << "  Marker size:              " << displayCfg_.markerSizeMm << " mm  |  " << sz << " px\n"
              << "  H spacing (center-ctr):   " << c2cX_mm << " mm  |  " << static_cast<int>(sz + spacingX) << " px\n"
              << "  H spacing (edge-edge):    " << gapX_mm << " mm  |  " << static_cast<int>(spacingX) << " px\n"
              << "  V spacing (center-ctr):   " << c2cY_mm << " mm  |  " << static_cast<int>(sz + spacingY) << " px\n"
              << "  V spacing (edge-edge):    " << gapY_mm << " mm  |  " << static_cast<int>(spacingY) << " px\n"
              << "  Padding (all sides):      " << displayCfg_.paddingMm << " mm  |  " << pad << " px\n"
              << std::defaultfloat;
}

cv::Point2i ArucoHandler::gridCellOrigin(int col, int row) const {
    const int sz = static_cast<int>(std::round(displayCfg_.markerSizeMm * touchCfg_.pixelsPerMm));
    const int pad = static_cast<int>(std::round(displayCfg_.paddingMm * touchCfg_.pixelsPerMm));

    float spacingX = (displayCfg_.cols > 1)
                         ? static_cast<float>(touchCfg_.width - 2 * pad - displayCfg_.cols * sz) / (displayCfg_.cols - 1)
                         : 0.0f;
    float spacingY = (displayCfg_.rows > 1)
                         ? static_cast<float>(touchCfg_.height - 2 * pad - displayCfg_.rows * sz) / (displayCfg_.rows - 1)
                         : 0.0f;

    return cv::Point2i(
        static_cast<int>(pad + col * (sz + spacingX)),
        static_cast<int>(pad + row * (sz + spacingY)));
}

void ArucoHandler::initDetector() {
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);

    // All values come from config.yaml [aruco_detector] - edit there, not here.
    detectorParams_.adaptiveThreshConstant = detectorCfg_.adaptiveThreshConstant;
    detectorParams_.adaptiveThreshWinSizeMin = detectorCfg_.adaptiveThreshWinSizeMin;
    detectorParams_.adaptiveThreshWinSizeMax = detectorCfg_.adaptiveThreshWinSizeMax;
    detectorParams_.adaptiveThreshWinSizeStep = detectorCfg_.adaptiveThreshWinSizeStep;
    detectorParams_.minMarkerPerimeterRate = detectorCfg_.minMarkerPerimeterRate;
    detectorParams_.maxMarkerPerimeterRate = detectorCfg_.maxMarkerPerimeterRate;
    detectorParams_.polygonalApproxAccuracyRate = detectorCfg_.polygonalApproxAccuracyRate;
    detectorParams_.minCornerDistanceRate = detectorCfg_.minCornerDistanceRate;
    detectorParams_.minDistanceToBorder = detectorCfg_.minDistanceToBorder;
    // Corner refinement is the dominant per-marker cost inside detectMarkers(), and
    // on the dense Fitts board (450+ markers) refining every marker is what makes
    // the detector fall behind at the mid-range distance where the most markers
    // decode at once. Only the active guidance target actually needs subpixel
    // corners (for its 3D pose); the board homography fits hundreds of corners by
    // least squares and is unaffected by per-marker subpixel precision. So the
    // detectors are built with NO global refinement, and RunDetection() refines
    // only the active target's corners (see refineActiveTargetCorners_).
    detectorParams_.cornerRefinementMethod        = cv::aruco::CORNER_REFINE_NONE;
    detectorParams_.cornerRefinementMaxIterations = detectorCfg_.cornerRefinementMaxIterations;
    detectorParams_.cornerRefinementMinAccuracy   = detectorCfg_.cornerRefinementMinAccuracy;
    refineActiveTargetCorners_ =
        ( detectorCfg_.cornerRefinementMethod ==
          static_cast<int>( cv::aruco::CORNER_REFINE_SUBPIX ) );
    detectorParams_.detectInvertedMarker = detectorCfg_.detectInvertedMarker;
    detectorParams_.perspectiveRemovePixelPerCell = detectorCfg_.perspectiveRemovePixelPerCell;
    detectorParams_.perspectiveRemoveIgnoredMarginPerCell = detectorCfg_.perspectiveRemoveIgnoredMarginPerCell;
    detectorParams_.errorCorrectionRate = detectorCfg_.errorCorrectionRate;
    detectorParams_.useAruco3Detection = detectorCfg_.useAruco3Detection;

    detector_    = cv::aruco::ArucoDetector(dictionary_,       detectorParams_);
    calDetector_ = cv::aruco::ArucoDetector(calGridDictionary_, detectorParams_);

    // Seed the active ID range from config - matches the Fitts/default mode
    activeValidIdMin_.store(detectCfg_.validIdMin);
    activeValidIdMax_.store(detectCfg_.validIdMax);

    std::cout << "ArucoHandler: Detector initialized (DICT_4X4_1000).\n";
}
