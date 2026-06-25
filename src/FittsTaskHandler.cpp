#include "FittsTaskHandler.h"

#include <cmath>
#include <iomanip>
#include <sstream>

// =============================================================================
// FittsTaskHandler.cpp
// =============================================================================

FittsTaskHandler::FittsTaskHandler(const FittsBoardConfig&   boardCfg,
                                   const TouchscreenConfig&  touchCfg,
                                   const CameraConfig&       camCfg)
    : touchCfg_(touchCfg), camCfg_(camCfg), layout_(boardCfg, touchCfg) {}

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
// Private - unified multi-scale solvePnP over the Fitts board
//
// Every marker (coarse or fine) lives on the same screen plane, so a single
// solvePnP over whatever is currently visible recovers the camera pose. Far
// away only the coarse markers contribute; up close only the fine ones; in the
// overlap band both do, naturally weighted by corner count - so the near/far
// handoff is automatic, with no explicit blending. Each marker's object points
// are built from its own rendered extent (FittsBoardLayout), so the two marker
// sizes stay metrically consistent.
// =============================================================================

bool FittsTaskHandler::ComputeArucoPose(const std::vector<DetectedMarker>& markers,
                                        cv::Vec3d& rvecOut, cv::Vec3d& tvecOut) const {
    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;

    for (const auto& m : markers) {
        const FittsMarker* fm = layout_.Find(m.id);
        if (!fm) continue;
        const float ox = fm->xPx * touchCfg_.mmPerPixel;
        const float oy = fm->yPx * touchCfg_.mmPerPixel;
        const float sz = fm->sizePx * touchCfg_.mmPerPixel;
        objPts.push_back({ox,      oy,      0.f});
        objPts.push_back({ox + sz, oy,      0.f});
        objPts.push_back({ox + sz, oy + sz, 0.f});
        objPts.push_back({ox,      oy + sz, 0.f});
        for (int k = 0; k < 4; k++)
            imgPts.push_back({m.cornersPx[k].x, m.cornersPx[k].y});
    }

    if (static_cast<int>(objPts.size()) < 4) return false;

    return cv::solvePnP(objPts, imgPts, camCfg_.cameraMatrix, camCfg_.distCoeffs,
                        rvecOut, tvecOut, false, cv::SOLVEPNP_ITERATIVE);
}

bool FittsTaskHandler::EstimateTargetFromBoard(const std::vector<DetectedMarker>& markers,
                                               int targetId,
                                               cv::Point3f& posOut, float& rollOut,
                                               std::array<cv::Point2f, 4>* cornersPxOut) const {
    const FittsMarker* fm = layout_.Find(targetId);
    if (!fm) return false;

    const float mmpp = touchCfg_.mmPerPixel;

    // Build board(mm) <-> image(px) correspondences from every visible board
    // marker, then fit a homography. The whole board is planar, so the mapping
    // from board-plane coordinates to the image is exactly a homography. Unlike
    // solvePnP on a planar target - which has a two-fold pose ambiguity that
    // flips between two tilted solutions every frame when the perspective is
    // weak (i.e. far away), throwing the projection all over the screen - a
    // homography is a single, unique 2D->2D map, so the estimate is stable.
    std::vector<cv::Point2f> srcMm, dstPx;
    srcMm.reserve(markers.size() * 4);
    dstPx.reserve(markers.size() * 4);
    for (const auto& m : markers) {
        const FittsMarker* v = layout_.Find(m.id);
        if (!v) continue;
        const float x0 = v->xPx * mmpp, y0 = v->yPx * mmpp;
        const float x1 = (v->xPx + v->sizePx) * mmpp, y1 = (v->yPx + v->sizePx) * mmpp;
        srcMm.push_back({x0, y0}); srcMm.push_back({x1, y0});
        srcMm.push_back({x1, y1}); srcMm.push_back({x0, y1});
        for (int k = 0; k < 4; k++) dstPx.push_back(m.cornersPx[k]);
    }
    if (srcMm.size() < 4) return false;

    cv::Mat H = cv::findHomography(srcMm, dstPx, cv::RANSAC, 3.0);
    if (H.empty()) return false;

    // Map the target marker's corners + centre (board mm) into the image (px).
    const float tx0 = fm->xPx * mmpp, ty0 = fm->yPx * mmpp;
    const float tx1 = (fm->xPx + fm->sizePx) * mmpp, ty1 = (fm->yPx + fm->sizePx) * mmpp;
    std::vector<cv::Point2f> tBoard = {
        {tx0, ty0}, {tx1, ty0}, {tx1, ty1}, {tx0, ty1},
        {(tx0 + tx1) * 0.5f, (ty0 + ty1) * 0.5f}};
    std::vector<cv::Point2f> tImg;
    cv::perspectiveTransform(tBoard, tImg, H);

    if (cornersPxOut)
        for (int k = 0; k < 4; k++) (*cornersPxOut)[k] = tImg[k];
    const cv::Point2f centerPx = tImg[4];

    // Depth from apparent marker scale: Z = f * sizeMm / edgePx, averaged over
    // visible markers. This uses only apparent size (not tilt), so it is also
    // free of the planar-pose ambiguity and stays stable far away.
    const double f = 0.5 * (camCfg_.fx + camCfg_.fy);
    double sumDepth = 0.0;
    int    nd = 0;
    for (const auto& m : markers) {
        const FittsMarker* v = layout_.Find(m.id);
        if (!v) continue;
        double edge = 0.0;
        for (int k = 0; k < 4; k++) {
            const cv::Point2f d = m.cornersPx[(k + 1) & 3] - m.cornersPx[k];
            edge += std::sqrt(d.x * d.x + d.y * d.y);
        }
        edge *= 0.25;
        if (edge > 1e-3) { sumDepth += f * (v->sizePx * mmpp) / edge; nd++; }
    }
    if (nd == 0) return false;
    const double depth = sumDepth / nd;

    // Camera-relative target position from the image ray + depth (Y-up, to match
    // DetectedMarker::positionMm). Stable because both centre px (homography) and
    // depth (scale) are ambiguity-free.
    const double X = (centerPx.x - camCfg_.cx) / camCfg_.fx * depth;
    const double Y = (centerPx.y - camCfg_.cy) / camCfg_.fy * depth;
    posOut = cv::Point3f(static_cast<float>( X),
                         static_cast<float>(-Y),
                         static_cast<float>( depth));

    // Camera roll = circular mean of every visible board marker's image roll
    // (all coplanar and axis-aligned, so each measures the same camera roll).
    float sumSin = 0.f, sumCos = 0.f;
    int   nr = 0;
    for (const auto& m : markers) {
        if (layout_.Find(m.id)) {
            sumSin += std::sin(m.rollRad);
            sumCos += std::cos(m.rollRad);
            nr++;
        }
    }
    rollOut = (nr > 0) ? std::atan2(sumSin, sumCos) : 0.f;
    return true;
}

bool FittsTaskHandler::GetTargetFullPose(const std::vector<DetectedMarker>& markers,
                                         int targetId,
                                         bool cal3Complete, cv::Point3f cal3Offset,
                                         float cal3RollRef,
                                         cv::Point3f& posMmOut, cv::Vec4f& quatXyzwOut,
                                         cv::Point3f& dispMmOut, bool& detectedOut) const {
    const FittsMarker* fm = layout_.Find(targetId);
    if (!fm) return false;

    detectedOut = false;
    for (const auto& m : markers)
        if (m.id == targetId) { detectedOut = true; break; }

    // --- Clean target trajectory (camera frame, Y-up) ------------------------
    // Use the ambiguity-free homography + apparent-scale estimator rather than
    // solvePnP, whose planar-pose ambiguity throws the depth +/-50 mm when the
    // board is far away. This is the same estimate that drives live guidance.
    float rollRad = 0.0f;
    if (!EstimateTargetFromBoard(markers, targetId, posMmOut, rollRad)) return false;

    // --- Fingertip-compensated displacement Δp = target - fingertip ----------
    // Uses the FULL 3D Cal3 offset (incl. its Z standoff), so dz -> 0 when the
    // fingertip reaches the target plane. Cal3 offset = fingertip - camera in the
    // screen-world frame (X right, Y DOWN, Z INTO the screen / away from camera).
    // The camera-frame depth axis points the same way as screen-world +Z (toward
    // the screen), so Z carries over unchanged and cal3Offset.z is the positive
    // standoff -> dz = tz - standoff < tz. Only Y flips (screen Y-down -> camera
    // Y-up). Then roll about +Z by (roll_now - roll_ref) to match
    // ControllerHandler::BuildRollCorrection.
    if (cal3Complete) {
        const cv::Point3f dFull(cal3Offset.x, -cal3Offset.y, cal3Offset.z);
        const float dRoll = rollRad - cal3RollRef;
        const float c = std::cos(dRoll), s = std::sin(dRoll);
        const cv::Point3f fingertip(c * dFull.x - s * dFull.y,
                                    s * dFull.x + c * dFull.y,
                                    dFull.z);
        dispMmOut = posMmOut - fingertip;
    } else {
        dispMmOut = posMmOut;    // no compensation available
    }

    // --- Orientation (nice-to-have, OpenCV Y-down) ---------------------------
    // board->camera rotation from solvePnP, as a quaternion. May be noisy far
    // away (planar-pose ambiguity); position above is unaffected by it.
    cv::Vec3d rvec, tvec;
    if (!ComputeArucoPose(markers, rvec, tvec) || tvec[2] <= 1e-6) {
        quatXyzwOut = cv::Vec4f(0.f, 0.f, 0.f, 1.f);
        return true;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Rotation matrix (board->camera) -> quaternion (x,y,z,w).
    const double m00 = R.at<double>(0,0), m01 = R.at<double>(0,1), m02 = R.at<double>(0,2);
    const double m10 = R.at<double>(1,0), m11 = R.at<double>(1,1), m12 = R.at<double>(1,2);
    const double m20 = R.at<double>(2,0), m21 = R.at<double>(2,1), m22 = R.at<double>(2,2);
    const double tr = m00 + m11 + m22;
    double qw, qx, qy, qz;
    if (tr > 0.0) {
        double s = std::sqrt(tr + 1.0) * 2.0;
        qw = 0.25 * s; qx = (m21 - m12) / s; qy = (m02 - m20) / s; qz = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        qw = (m21 - m12) / s; qx = 0.25 * s; qy = (m01 + m10) / s; qz = (m02 + m20) / s;
    } else if (m11 > m22) {
        double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = 0.25 * s; qz = (m12 + m21) / s;
    } else {
        double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = 0.25 * s;
    }
    quatXyzwOut = cv::Vec4f(static_cast<float>(qx), static_cast<float>(qy),
                            static_cast<float>(qz), static_cast<float>(qw));
    return true;
}

cv::Point2f FittsTaskHandler::TargetCenterPx(const cv::Vec3d& rvec, const cv::Vec3d& tvec) const {
    const FittsMarker* fm = layout_.Find(targetId_);
    if (!fm) return {};
    const float cx = (fm->xPx + fm->sizePx * 0.5f) * touchCfg_.mmPerPixel;
    const float cy = (fm->yPx + fm->sizePx * 0.5f) * touchCfg_.mmPerPixel;

    std::vector<cv::Point3f> pts3d = {cv::Point3f(cx, cy, 0.f)};
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
