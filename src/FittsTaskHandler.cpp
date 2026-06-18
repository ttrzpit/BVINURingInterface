#include "FittsTaskHandler.h"

#include <cmath>
#include <iomanip>
#include <sstream>

// =============================================================================
// FittsTaskHandler.cpp
// =============================================================================

FittsTaskHandler::FittsTaskHandler(const ArucoDisplayConfig& displayCfg,
                                   const TouchscreenConfig&  touchCfg,
                                   const CameraConfig&       camCfg)
    : displayCfg_(displayCfg), touchCfg_(touchCfg), camCfg_(camCfg) {}

void FittsTaskHandler::Reset() {
    targetId_    = 0;
    wasTouched_  = false;
    ftValid_     = false;
    sampleValid_ = false;
    errorLine1_.clear();
    errorLine2_.clear();
}

void FittsTaskHandler::OnNewTarget(int targetId) {
    targetId_     = targetId;
    sampleValid_  = false;
    touchFtValid_ = false;
    errorLine1_.clear();
    errorLine2_.clear();
}

// =============================================================================
// Private - multi-tag solvePnP (mirrors ArucoHandler::renderGridImage layout)
// =============================================================================

bool FittsTaskHandler::ComputeArucoPose(const std::vector<DetectedMarker>& markers,
                                        cv::Vec3d& rvecOut, cv::Vec3d& tvecOut) const {
    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;

    const float sz_px  = std::round(displayCfg_.markerSizeMm * touchCfg_.pixelsPerMm);
    const float pad_px = std::round(displayCfg_.paddingMm    * touchCfg_.pixelsPerMm);
    const float sz_mm  = displayCfg_.markerSizeMm;
    const float spX_px = (displayCfg_.cols > 1)
        ? (touchCfg_.width  - 2.f*pad_px - displayCfg_.cols*sz_px) / (displayCfg_.cols - 1) : 0.f;
    const float spY_px = (displayCfg_.rows > 1)
        ? (touchCfg_.height - 2.f*pad_px - displayCfg_.rows*sz_px) / (displayCfg_.rows - 1) : 0.f;

    for (const auto& m : markers) {
        if (m.id < 1 || m.id > displayCfg_.cols * displayCfg_.rows) continue;
        const int col = (m.id - 1) % displayCfg_.cols;
        const int row = (m.id - 1) / displayCfg_.cols;
        const float ox = (pad_px + col * (sz_px + spX_px)) * touchCfg_.mmPerPixel;
        const float oy = (pad_px + row * (sz_px + spY_px)) * touchCfg_.mmPerPixel;
        objPts.push_back({ox,        oy,        0.f});
        objPts.push_back({ox+sz_mm,  oy,        0.f});
        objPts.push_back({ox+sz_mm,  oy+sz_mm,  0.f});
        objPts.push_back({ox,        oy+sz_mm,  0.f});
        for (int k = 0; k < 4; k++)
            imgPts.push_back({m.cornersPx[k].x, m.cornersPx[k].y});
    }

    if (static_cast<int>(objPts.size()) < 4) return false;

    return cv::solvePnP(objPts, imgPts, camCfg_.cameraMatrix, camCfg_.distCoeffs,
                        rvecOut, tvecOut, false, cv::SOLVEPNP_ITERATIVE);
}

cv::Point2f FittsTaskHandler::TargetCenterPx(const cv::Vec3d& rvec, const cv::Vec3d& tvec) const {
    const float sz_px  = std::round(displayCfg_.markerSizeMm * touchCfg_.pixelsPerMm);
    const float pad_px = std::round(displayCfg_.paddingMm    * touchCfg_.pixelsPerMm);
    const float sz_mm  = displayCfg_.markerSizeMm;
    const float spX_px = (displayCfg_.cols > 1)
        ? (touchCfg_.width  - 2.f*pad_px - displayCfg_.cols*sz_px) / (displayCfg_.cols - 1) : 0.f;
    const float spY_px = (displayCfg_.rows > 1)
        ? (touchCfg_.height - 2.f*pad_px - displayCfg_.rows*sz_px) / (displayCfg_.rows - 1) : 0.f;

    const int   col = (targetId_ - 1) % displayCfg_.cols;
    const int   row = (targetId_ - 1) / displayCfg_.cols;
    const float ox  = (pad_px + col * (sz_px + spX_px)) * touchCfg_.mmPerPixel;
    const float oy  = (pad_px + row * (sz_px + spY_px)) * touchCfg_.mmPerPixel;

    std::vector<cv::Point3f> pts3d = {cv::Point3f(ox + sz_mm * 0.5f, oy + sz_mm * 0.5f, 0.f)};
    std::vector<cv::Point2f> pts2d;
    cv::projectPoints(pts3d, rvec, tvec, camCfg_.cameraMatrix, camCfg_.distCoeffs, pts2d);
    return pts2d[0];
}

cv::Point2i FittsTaskHandler::VirtualFingertipPx(const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                                                 cv::Point3f d, float rollRefRad) const {
    // Roll-corrected XY offset (negated: rvec[2]'s rotation sense is opposite
    // the screen's X-right/Y-down frame)
    const float  deltaRoll = rollRefRad - static_cast<float>(rvec[2]);
    const double cosR = std::cos(deltaRoll), sinR = std::sin(deltaRoll);
    const double dx = cosR * d.x - sinR * d.y;
    const double dy = sinR * d.x + cosR * d.y;

    // Depth of the ArUco plane from the camera (mm) - scales the mm offset to
    // pixels so the cursor sits on the same plane as the detected tags
    const double depth = tvec[2];
    const double dxPx = camCfg_.fx * dx / depth;
    const double dyPx = camCfg_.fy * dy / depth;

    return cv::Point2i(static_cast<int>(camCfg_.cx + dxPx),
                       static_cast<int>(camCfg_.cy + dyPx));
}

// =============================================================================
// Update - called once per main loop iteration while in FITTS state
// =============================================================================

void FittsTaskHandler::Update(const std::vector<DetectedMarker>& markers,
                              const TouchState&                  touch,
                              bool                               cal3Complete,
                              cv::Point3f                        cal3Offset,
                              float                              cal3RollRef) {
    cv::Vec3d rvec, tvec;
    bool havePose = ComputeArucoPose(markers, rvec, tvec) && tvec[2] > 1e-6;

    // Live virtual fingertip cursor - updates every frame
    if (havePose && cal3Complete) {
        ftPx_    = VirtualFingertipPx(rvec, tvec, cal3Offset, cal3RollRef);
        ftValid_ = true;
    } else {
        ftValid_ = false;
    }

    // Touch sample - recorded on the rising edge while a target is active
    bool isTouchedNow = touch.isTouched;
    if (isTouchedNow && !wasTouched_ && targetId_ > 0 ) {
        const cv::Point2f targetPx = TargetCenterPx(rvec, tvec);

        // Touch position is already in touchscreen-local pixels - drawn
        // directly on the touchscreen display (ArucoHandler::SetFittsOverlay)
        touchScreenPx_ = touch.position;

        // Freeze the virtual fingertip's current position - drawn as a
        // persistent marker on the operator display (DisplayHandler) until
        // the next target is selected.
        touchFtPx_    = ftPx_;
        touchFtValid_ = ftValid_;

        const double depth   = tvec[2];
        const double mmPerPx = depth / ((camCfg_.fx + camCfg_.fy) * 0.5);

        // Camera principal point ↔ target marker error
        const double camDx   = camCfg_.cx - targetPx.x;
        const double camDy   = camCfg_.cy - targetPx.y;
        const double camErrPx = std::sqrt(camDx * camDx + camDy * camDy);

        std::ostringstream l1;
        l1 << std::fixed << std::setprecision(1)
           << "Cam-Tag err: " << camErrPx << " px / " << (camErrPx * mmPerPx) << " mm";
        errorLine1_ = l1.str();

        std::ostringstream l2;
        if (ftValid_) {
            const double ftDx   = ftPx_.x - targetPx.x;
            const double ftDy   = ftPx_.y - targetPx.y;
            const double ftErrPx = std::sqrt(ftDx * ftDx + ftDy * ftDy);
            l2 << std::fixed << std::setprecision(1)
               << "Tip-Tag err: " << ftErrPx << " px / " << (ftErrPx * mmPerPx) << " mm";
        } else {
            l2 << "Tip-Tag err: N/A (Cal3 incomplete)";
        }
        errorLine2_ = l2.str();

        sampleValid_ = true;
    }
    wasTouched_ = isTouchedNow;
}
