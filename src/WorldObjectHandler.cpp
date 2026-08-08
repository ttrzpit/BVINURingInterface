#include "WorldObjectHandler.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

// =============================================================================
// WorldObjectHandler.cpp - clean global-PnP object localisation (no filtering)
//
// See the header for the model. The finite/NaN guards are mandatory: degenerate
// solvePnP results can be non-finite, and because every comparison with NaN is
// false such values slip past range checks and corrupt OpenCV's drawing buffers
// (the crash class the repo fought in cv::findContours). Every pose is
// finite-checked before use and every projected point is rejected if non-finite
// before it is drawn.
// =============================================================================

// solvePnPRansac reprojection-error inlier threshold [px] for the world pose - a
// misdetected / mis-decoded world marker's centre lands outside this and is
// rejected. (RANSAC is only the fallback path for a non-coplanar marker set;
// the ground-plane board uses the deterministic SolvePlanarWorldPose.)
static constexpr float kRansacReprojPx = 4.0f;

// Deterministic outlier gate [px] for SolvePlanarWorldPose: a marker whose MEAN
// corner reprojection error against the first full-set solve exceeds this is
// dropped (once) and the pose re-solved. Catches a biased marker_positions
// entry / bent board region the same way every frame - unlike RANSAC's random
// inlier subsets, which made the pose JUMP when a biased marker sat near the
// threshold.
static constexpr double kWorldMarkerReprojGatePx = 3.0;

// Frames a TRAINED object coasts on its last rendered pose when the world pose
// drops out (~0.25 s at the detection rate), instead of snapping to its own
// live marker pose. After this it hides until the world pose returns.
static constexpr int kCoastFrames = 20;

// Consecutive frames whose measured horizontal displacement from the trained
// anchor must exceed objects.contact_move_threshold_mm before CONTACT latches.
// The latch is permanent for the trial, and the threshold is small (a few mm),
// so a single noisy solo-IPPE pose of the 40 mm object marker must not be able
// to set it; three frames (~40 ms) costs nothing perceptually.
static constexpr int kContactConfirmFrames = 3;

// ---- File-local helpers -----------------------------------------------------

static bool finite3(const cv::Vec3d& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

static cv::Matx33d toMatx33(const cv::Mat& m) {
    cv::Mat md;
    m.convertTo(md, CV_64F);
    cv::Matx33d r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r(i, j) = md.at<double>(i, j);
    return r;
}

// Project a marker-frame point through (R|t) with a pinhole model (image already
// undistorted). False if behind the camera or non-finite. `!(Xc[2] > 1.0)`
// rejects NaN as well as depths <= 1 mm.
static bool projectMarkerPt(const cv::Matx33d& R, const cv::Vec3d& t,
                            const cv::Point3f& P, double fx, double fy,
                            double cx, double cy, cv::Point2f& out) {
    const cv::Vec3d Xc = R * cv::Vec3d(P.x, P.y, P.z) + t;
    if (!(Xc[2] > 1.0)) return false;
    const double ix = fx * Xc[0] / Xc[2] + cx;
    const double iy = fy * Xc[1] / Xc[2] + cy;
    if (!std::isfinite(ix) || !std::isfinite(iy)) return false;
    out = cv::Point2f(static_cast<float>(ix), static_cast<float>(iy));
    return true;
}

// Nearest proper rotation to an averaged rotation matrix (Kabsch/SVD):
// R = U V^T from SVD(M); flip the last column if it came out a reflection. The
// mean of rotation matrices is not itself a rotation, so a training burst's
// averaged anchor rotation is projected back onto SO(3) with this.
static cv::Matx33d Orthonormalize(const cv::Matx33d& M) {
    cv::Mat w, U, Vt;
    cv::SVDecomp(cv::Mat(M), w, U, Vt);
    cv::Mat R = U * Vt;
    if (cv::determinant(R) < 0) {
        U.col(2) *= -1.0;
        R = U * Vt;
    }
    cv::Matx33d r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r(i, j) = R.at<double>(i, j);
    return r;
}

// Reject the mirrored planar-PnP solution. The world board is a single ground
// plane (y = 0), and PnP on coplanar points has a second, reflected solution
// that places the camera below that plane; the real camera is always above it
// (world +Y). Camera position in the world frame = -R^T t.
static bool CameraAboveFloor(const cv::Matx33d& Rwc, const cv::Vec3d& twc) {
    const cv::Vec3d camWorld = -(Rwc.t() * twc);
    return camWorld[1] > 0.0;
}

// Solve a square marker's pose (marker -> camera) from its 4 detected corners.
static bool solveSquare(const std::array<cv::Point2f, 4>& px, float halfMm,
                        const cv::Mat& K, const cv::Mat& D,
                        cv::Matx33d& R, cv::Vec3d& t) {
    const std::vector<cv::Point3f> obj = {
        {-halfMm, halfMm, 0.f}, {halfMm, halfMm, 0.f}, {halfMm, -halfMm, 0.f}, {-halfMm, -halfMm, 0.f}
    };
    const std::vector<cv::Point2f> img(px.begin(), px.end());
    cv::Vec3d rvec, tvec;
    if (!cv::solvePnP(obj, img, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE))
        return false;
    if (!finite3(rvec) || !finite3(tvec)) return false;
    cv::Mat Rm;
    cv::Rodrigues(rvec, Rm);
    R = toMatx33(Rm);
    t = tvec;
    return true;
}

// ---- Construction -----------------------------------------------------------

WorldObjectHandler::WorldObjectHandler(const ObjectWorldConfig& objCfg,
                                       const CameraConfig&      camCfg)
    : objCfg_(objCfg), camCfg_(camCfg),
      rollOffsetRad_(objCfg.rollOffsetDeg * static_cast<float>(CV_PI / 180.0)) {
    for (const auto& [id, def] : objCfg_.targetObjects) {
        ObjectRuntime rt;
        rt.def   = &def;
        rt.edges = BuildEdges(def);
        objects_[id] = std::move(rt);
    }

    // World-board corner geometry from the configured CENTRES + marker size.
    // Plane from the centre itself (y = 0 -> ground plane spanning X/Z - the
    // whole current board; else legacy wall at z = 0 spanning X/Y); orientation
    // from the mounting convention: ground markers lie face-up with the printed
    // top edge toward the z = 0 row (marker +Y = world -Z, face up: +Z = world
    // +Y), wall markers are upright (marker +Y = world +Y).
    // Corner order matches ArUco detection order (TL, TR, BR, BL).
    const float h = objCfg_.worldMarkerSizeMm * 0.5f;
    const cv::Vec3d cornersMarker[4] = {
        {-h, h, 0.0}, {h, h, 0.0}, {h, -h, 0.0}, {-h, -h, 0.0}
    };
    for (const auto& [id, c] : objCfg_.markerPositions) {
        WorldMarkerGeom g;
        const bool onFloor = std::abs(c.y) < 1e-4f;
        g.Rmw = onFloor ? cv::Matx33d(1, 0, 0,
                                      0, 0, 1,
                                      0, -1, 0)
                        : cv::Matx33d::eye();
        g.centerWorld = cv::Vec3d(c.x, c.y, c.z);
        for (int k = 0; k < 4; ++k) {
            const cv::Vec3d w = g.Rmw * cornersMarker[k] + g.centerWorld;
            g.cornersWorld[k] = cv::Point3f(static_cast<float>(w[0]),
                                            static_cast<float>(w[1]),
                                            static_cast<float>(w[2]));
        }
        worldGeom_[id] = g;
    }

    // Ring base<-second seed transform from the configured fold angle: the two
    // marker planes meet at the base marker's printed top edge (the y = +h
    // line), with the second marker folded toward +Z by relationshipAngleDeg.
    // X_base = R * X_second + t, so the second marker's centre sits at
    // (0, h(1+cos a), h sin a) in the base frame. Replaced by the live-learned
    // transform the first time both markers are seen together (Update()).
    const RingMarkerConfig& ring = objCfg_.ring;
    if (ring.baseMarker >= 0 && ring.markerSizeMm > 0.0f) {
        const double a = ring.relationshipAngleDeg * CV_PI / 180.0;
        const double h = ring.markerSizeMm * 0.5;
        ringRelR_ = cv::Matx33d(1, 0, 0,
                                0, std::cos(a), -std::sin(a),
                                0, std::sin(a),  std::cos(a));
        ringRelT_ = cv::Vec3d(0.0, h * (1.0 + std::cos(a)), h * std::sin(a));

        // Arrow-frame constant factor (see GetCamYupToArrowR): M maps arrow
        // axes into marker axes (X_a = X_m, Y_a = Z_m, Z_a = -Y_m), so M^T
        // takes a marker-frame vector into arrow coordinates.
        //   HANDEDNESS FIX (Sx): the per-frame camera Y-up -> Y-down flip
        //   (diag(1,-1,1)) is a REFLECTION, while M^T, Rring^T and Rz are all
        //   proper rotations - so without correction the camYup->arrow map has
        //   det -1 (left-handed) and exactly one lateral axis reads BACKWARDS
        //   for every mount. This is mount-independent (the reflection is
        //   always present). Sx = diag(-1,1,1) mirrors the arrow X axis to
        //   restore a right-handed frame with physical +X = right / +Y = up,
        //   matching the FITTS-validated controller sign convention (Y was
        //   already correct on the rig at trim 0, X was inverted -> flip X).
        //   Rz then applies the config yaw trim for any residual ROTATIONAL
        //   mount misalignment, on top of the corrected right-handed frame.
        const double g = ring.arrowYawTrimDeg * CV_PI / 180.0;
        const cv::Matx33d Rz(std::cos(g), -std::sin(g), 0.0,
                             std::sin(g),  std::cos(g), 0.0,
                             0.0,          0.0,         1.0);
        const cv::Matx33d Mt(1,  0, 0,
                             0,  0, 1,
                             0, -1, 0);
        const cv::Matx33d Sx(-1, 0, 0,
                              0, 1, 0,
                              0, 0, 1);
        arrowTrimM_ = Rz * Sx * Mt;
    }
}

void WorldObjectHandler::Reset() {
    activeId_    = 0;
    scanning_    = true;    // every OBJECTS session starts in the scan phase
    poseOk_      = false;
    worldMarkerCount_ = 0;
    hasTarget_   = false;
    targetLive_  = false;
    targetPosMm_ = {};
    targetRoll_  = 0.0f;
    overshooting_ = false;
    overshootMm_  = 0.0f;
    overlays_.clear();
    worldOutlines_.clear();
    knownLayoutOutlines_.clear();
    hasRingFingertip_ = false;
    ringOverlay_      = RingOverlay{};
    ringFromSecond_   = false;
    ringCoasting_     = false;
    lastRingFingertip_ = {};
    hasLastRing_       = false;
    ringCoastLeft_     = 0;
    training_         = false;
    trainFramesLeft_  = 0;
    lastTrainedCount_ = 0;
    trainAcc_.clear();
    presenceScanning_    = false;
    presenceFramesLeft_  = 0;
    presenceResultReady_ = false;
    presenceSeen_.clear();
    presentIds_.clear();
    worldScanning_       = false;
    worldScanFramesLeft_ = 0;
    worldScanAcc_.clear();
    worldMaskQuads_.clear();
    probeActive_      = false;
    probeFramesLeft_  = 0;
    probeStats_.clear();
    worldReprojErr_.clear();
    worldIds_.clear();
    poseFailCount_ = 0;
    // ringRelR_/ringRelT_ (the learned base<-second transform) are deliberately
    // KEPT: the ring mount is rigid, so the transform survives OBJECTS re-entry.
    for (auto& [id, rt] : objects_) {
        rt.anchor         = ObjectAnchor{};
        rt.hasLast        = false;
        rt.coastLeft      = 0;
        rt.contactRun     = 0;
        rt.contactLatched = false;
    }
}

void WorldObjectHandler::StartCornerJitterProbe() {
    probeActive_     = true;
    probeFramesLeft_ = kJitterProbeFrames;
    probeStats_.clear();
    std::cout << "[JITTER] probe started - hold the camera rigidly still ("
              << kJitterProbeFrames << " detection frames)...\n";
}

void WorldObjectHandler::StartTraining() {
    training_        = true;
    trainFramesLeft_ = std::max(1, objCfg_.trainFrames);
    trainAcc_.clear();
    // A burst re-measures the anchors, so any CONTACT latch (which only means
    // "displaced from the OLD anchor") is stale - clear it and re-arm.
    for (auto& [id, rt] : objects_) {
        rt.contactRun     = 0;
        rt.contactLatched = false;
    }
}

void WorldObjectHandler::StartWorldScan() {
    // Clear the old mask up front: the operator pressed 'w' because the current
    // boxes are stale (or absent), and seeing the bare markers again is the
    // feedback that a re-scan is running.
    worldScanning_       = true;
    worldScanFramesLeft_ = std::max(1, objCfg_.trainFrames);
    worldScanAcc_.clear();
    worldMaskQuads_.clear();
}

void WorldObjectHandler::StartPresenceScan() {
    presenceScanning_    = true;
    presenceFramesLeft_  = std::max(1, objCfg_.presenceScanFrames);
    presenceResultReady_ = false;
    presenceSeen_.clear();
    presentIds_.clear();
}

void WorldObjectHandler::UntrainAll() {
    training_        = false;
    trainFramesLeft_ = 0;
    trainAcc_.clear();
    for (auto& [id, rt] : objects_) {
        rt.anchor         = ObjectAnchor{};
        rt.hasLast        = false;
        rt.coastLeft      = 0;
        rt.contactRun     = 0;
        rt.contactLatched = false;
    }
}

int WorldObjectHandler::FinishScan() {
    scanning_ = false;
    return ScannedCount();
}

bool WorldObjectHandler::IsObjectScanned(int id) const {
    const auto it = objects_.find(id);
    return it != objects_.end() && it->second.anchor.has;
}

int WorldObjectHandler::ScannedCount() const {
    int n = 0;
    for (const auto& [id, rt] : objects_)
        if (rt.anchor.has) ++n;
    return n;
}

std::string WorldObjectHandler::GetScanStatus() const {
    if (training_)
        return "TRAINING - hold the camera steady... " + std::to_string(trainFramesLeft_);
    std::string names;
    int         n = 0;
    for (const auto& [id, rt] : objects_) {
        if (!rt.anchor.has) continue;
        if (n++) names += ", ";
        names += rt.def->name;
    }
    std::string s = "SCANNING - trained " + std::to_string(n) + "/" +
                    std::to_string(static_cast<int>(objects_.size()));
    if (n) s += " (" + names + ")";
    // World mask state rides along: until 'w' has run, the operator's first step
    // is to scan the BLANK workspace, so advertise it here.
    s += worldMaskQuads_.empty()
             ? " - [w] mask world markers (workspace clear), [t] train visible, [Enter] finish"
             : " [world mask " + std::to_string(worldMaskQuads_.size()) +
                   "] - [t] train visible, [Enter] finish";
    return s;
}

void WorldObjectHandler::OnNewTarget(int objectMarkerId) {
    activeId_   = objectMarkerId;
    hasTarget_  = false;
    targetLive_ = false;
    overshooting_ = false;   // live cue, but don't carry a stale one into the new trial
    overshootMm_  = 0.0f;
    // Held anchors are kept across target changes. CONTACT latches are per-trial,
    // so they all re-arm here (including the object just guided to, whose object
    // may have been carried away and put back).
    for (auto& [id, rt] : objects_) {
        rt.contactRun     = 0;
        rt.contactLatched = false;
    }
}

bool WorldObjectHandler::HasActiveAnchor() const {
    const auto it = objects_.find(activeId_);
    return it != objects_.end() && it->second.anchor.has;
}

// =============================================================================
// Deterministic planar world solve (ground-plane board)
// =============================================================================

bool WorldObjectHandler::SolvePlanarWorldPose(const std::vector<cv::Point3f>& worldPts,
                                              const std::vector<cv::Point2f>& imgPts) {
    const double fx = camCfg_.fx, fy = camCfg_.fy, cx = camCfg_.cx, cy = camCfg_.cy;
    const cv::Mat& K = camCfg_.cameraMatrix;
    const cv::Mat& D = camCfg_.distCoeffs;

    // Mean reprojection error [px] of a pose over a point set (infinity if any
    // point fails to project - degenerate pose).
    auto MeanErr = [&](const cv::Matx33d& R, const cv::Vec3d& t,
                       const std::vector<cv::Point3f>& wp,
                       const std::vector<cv::Point2f>& ip) {
        double e = 0.0;
        for (size_t i = 0; i < wp.size(); ++i) {
            cv::Point2f p;
            if (!projectMarkerPt(R, t, wp[i], fx, fy, cx, cy, p))
                return std::numeric_limits<double>::infinity();
            e += cv::norm(p - ip[i]);
        }
        return e / static_cast<double>(wp.size());
    };

    // IPPE returns BOTH planar solutions explicitly; keep the best one that
    // puts the camera above the floor. The mirrored solution is discarded by
    // the gate rather than by whichever one an iterative solver happens to
    // converge to. Returns the mean error (infinity = no acceptable solution).
    auto SolveIppe = [&](const std::vector<cv::Point3f>& wp,
                         const std::vector<cv::Point2f>& ip,
                         cv::Matx33d& bestR, cv::Vec3d& bestT) {
        std::vector<cv::Mat> rvecs, tvecs;
        cv::solvePnPGeneric(wp, ip, K, D, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
        double best = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < rvecs.size(); ++i) {
            const cv::Vec3d rv(rvecs[i].at<double>(0), rvecs[i].at<double>(1), rvecs[i].at<double>(2));
            const cv::Vec3d tv(tvecs[i].at<double>(0), tvecs[i].at<double>(1), tvecs[i].at<double>(2));
            if (!finite3(rv) || !finite3(tv)) continue;
            cv::Mat Rm; cv::Rodrigues(rv, Rm);
            const cv::Matx33d R = toMatx33(Rm);
            if (!CameraAboveFloor(R, tv)) continue;
            const double e = MeanErr(R, tv, wp, ip);
            if (e < best) { best = e; bestR = R; bestT = tv; }
        }
        return best;
    };

    try {
        cv::Matx33d R; cv::Vec3d t;
        if (!std::isfinite(SolveIppe(worldPts, imgPts, R, t)))
            return true;   // handled: genuinely no valid pose this frame

        // One deterministic outlier pass: drop whole markers (groups of 4
        // corners) whose mean corner error exceeds the gate, then re-solve on
        // the kept set. Same input -> same kept set -> same pose, every frame.
        std::vector<cv::Point3f> keptW;
        std::vector<cv::Point2f> keptI;
        const size_t nMk    = worldPts.size() / 4;
        size_t       keptMk = 0;
        for (size_t m = 0; m < nMk; ++m) {
            double em = 0.0;
            bool   ok = true;
            for (int k = 0; k < 4 && ok; ++k) {
                cv::Point2f p;
                ok = projectMarkerPt(R, t, worldPts[4 * m + k], fx, fy, cx, cy, p);
                if (ok) em += cv::norm(p - imgPts[4 * m + k]);
            }
            if (ok && em / 4.0 <= kWorldMarkerReprojGatePx) {
                for (int k = 0; k < 4; ++k) {
                    keptW.push_back(worldPts[4 * m + k]);
                    keptI.push_back(imgPts[4 * m + k]);
                }
                ++keptMk;
            }
        }
        if (keptMk >= 2) {
            if (keptMk < nMk) {   // something was dropped - re-solve without it
                cv::Matx33d R2; cv::Vec3d t2;
                if (std::isfinite(SolveIppe(keptW, keptI, R2, t2))) { R = R2; t = t2; }
            }
        } else {
            keptW = worldPts;     // gate rejected nearly everything - keep all
            keptI = imgPts;
        }

        // Levenberg-Marquardt polish over the kept set, then the final gates.
        cv::Mat rvm; cv::Rodrigues(cv::Mat(R), rvm);
        cv::Vec3d rv(rvm.at<double>(0), rvm.at<double>(1), rvm.at<double>(2));
        cv::Vec3d tv = t;
        cv::solvePnPRefineLM(keptW, keptI, K, D, rv, tv);
        if (finite3(rv) && finite3(tv)) {
            cv::Mat Rm; cv::Rodrigues(rv, Rm);
            const cv::Matx33d Rf = toMatx33(Rm);
            if (CameraAboveFloor(Rf, tv)) {
                worldR_ = Rf;
                worldT_ = tv;
                poseOk_ = true;
            }
        }
        return true;
    } catch (const cv::Exception&) {
        return false;   // solver threw - caller falls back to the RANSAC path
    }
}

// =============================================================================
// Wireframe edges (marker frame) - ported from ArUcoTest objectEdges()
// =============================================================================

std::vector<std::pair<cv::Point3f, cv::Point3f>>
WorldObjectHandler::BuildEdges(const ObjectDef& o) {
    std::vector<std::pair<cv::Point3f, cv::Point3f>> e;

    if (o.type == ObjectType::Cylinder && o.cylGeneral) {
        if (o.radiusMm <= 0.f || o.heightMm <= 0.f) return e;
        const float R = o.radiusMm;
        const int   N = 36;
        cv::Vec3f w(o.baseNormal.x, o.baseNormal.y, o.baseNormal.z);
        const float wn = static_cast<float>(cv::norm(w));
        if (wn < 1e-6f) return e;
        w /= wn;
        const cv::Vec3f ref = (std::abs(w[2]) < 0.9f) ? cv::Vec3f(0, 0, 1) : cv::Vec3f(1, 0, 0);
        cv::Vec3f u = w.cross(ref); u /= static_cast<float>(cv::norm(u));
        const cv::Vec3f v = w.cross(u);
        const cv::Vec3f B(o.baseOrigin.x, o.baseOrigin.y, o.baseOrigin.z);
        const cv::Vec3f T = B + w * o.heightMm;
        auto ring = [&](const cv::Vec3f& c, double a) {
            const cv::Vec3f p = c + R * (static_cast<float>(std::cos(a)) * u +
                                         static_cast<float>(std::sin(a)) * v);
            return cv::Point3f(p[0], p[1], p[2]);
        };
        for (int i = 0; i < N; ++i) {
            const double a = 2.0 * CV_PI * i / N;
            const double b = 2.0 * CV_PI * (i + 1) / N;
            e.emplace_back(ring(B, a), ring(B, b));
            e.emplace_back(ring(T, a), ring(T, b));
        }
        for (double a : { 0.0, CV_PI })
            e.emplace_back(ring(B, a), ring(T, a));
    } else if (o.type == ObjectType::Cylinder) {
        if (o.radiusMm <= 0.f || o.heightMm <= 0.f) return e;
        const float R  = o.radiusMm;
        const float yb = -o.markerCenterHeightMm;
        const float yt = o.heightMm - o.markerCenterHeightMm;
        const int   N  = 36;
        auto surf = [&](double a, float y) {
            return cv::Point3f(R * static_cast<float>(std::sin(a)), y,
                               R * static_cast<float>(std::cos(a) - 1.0));
        };
        for (int i = 0; i < N; ++i) {
            const double a = 2.0 * CV_PI * i / N;
            const double b = 2.0 * CV_PI * (i + 1) / N;
            e.emplace_back(surf(a, yb), surf(b, yb));
            e.emplace_back(surf(a, yt), surf(b, yt));
        }
        for (double a : { CV_PI * 0.5, CV_PI * 1.5 })
            e.emplace_back(surf(a, yb), surf(a, yt));
    } else if (o.points.size() == 8) {
        static const int E[12][2] = {
            {0, 1}, {1, 3}, {3, 2}, {2, 0},
            {4, 5}, {5, 7}, {7, 6}, {6, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}
        };
        for (const auto& ed : E) e.emplace_back(o.points[ed[0]], o.points[ed[1]]);
    }
    return e;
}

// =============================================================================
// Update - once per main loop iteration while in OBJECTS state
// =============================================================================

void WorldObjectHandler::Update(const std::vector<DetectedMarker>& markers) {
    hasTarget_  = false;
    targetLive_ = false;
    hasEffectiveWorldR_ = false;
    hasRingFingertip_ = false;
    ringFromSecond_   = false;
    ringCoasting_     = false;
    overlays_.clear();
    worldOutlines_.clear();
    knownLayoutOutlines_.clear();
    ringOverlay_ = RingOverlay{};
    worldMarkerCount_ = 0;
    poseOk_ = false;
    worldReprojErr_.clear();
    worldIds_.clear();

    const double fx = camCfg_.fx, fy = camCfg_.fy, cx = camCfg_.cx, cy = camCfg_.cy;
    const cv::Mat& K = camCfg_.cameraMatrix;
    const cv::Mat& D = camCfg_.distCoeffs;

    // --- 1. Gather detections -------------------------------------------------
    // World markers -> (world-frame corner <-> detected corner) correspondences
    // for the world pose. Object markers -> their own live pose (solved from
    // their 4 corners). Ring markers -> corners, resolved to the fingertip in
    // step 2b.
    struct OPose { cv::Matx33d R; cv::Vec3d t; float roll; };
    std::vector<cv::Point3f> worldObj;   // world-frame marker corners (config geometry)
    std::vector<cv::Point2f> worldImg;   // detected corners
    const WorldMarkerGeom*   soloGeom = nullptr;   // last world marker seen (1-marker path)
    const DetectedMarker*    soloMk   = nullptr;
    std::map<int, OPose>     objPoses;   // object id -> live pose

    const RingMarkerConfig& ring = objCfg_.ring;
    const bool ringEnabled = (ring.baseMarker >= 0 && ring.markerSizeMm > 0.0f);
    bool haveBase = false, haveSecond = false;
    std::array<cv::Point2f, 4> baseCorners{}, secondCorners{};

    for (const auto& m : markers) {
        const auto wIt = worldGeom_.find(m.id);
        if (wIt != worldGeom_.end()) {                        // world board marker
            for (int k = 0; k < 4; ++k) {
                worldObj.push_back(wIt->second.cornersWorld[k]);
                worldImg.push_back(m.cornersPx[k]);
            }
            worldIds_.push_back(m.id);   // parallel to each group of 4 corners
            soloGeom = &wIt->second;
            soloMk   = &m;
            worldOutlines_.push_back(m.cornersPx);
            ++worldMarkerCount_;
            // Mask scan ('w'): accumulate this marker's raw image corners. No
            // pose needed - the mask is frozen pixels, not world geometry.
            if (worldScanning_) {
                WorldScanAccum& acc = worldScanAcc_[m.id];
                for (int k = 0; k < 4; ++k)
                    acc.sum[k] += cv::Point2d(m.cornersPx[k]);
                acc.n++;
            }
            continue;
        }
        if (ringEnabled && (m.id == ring.baseMarker || m.id == ring.secondMarker)) {
            if (m.id == ring.baseMarker) { haveBase   = true; baseCorners   = m.cornersPx; }
            else                         { haveSecond = true; secondCorners = m.cornersPx; }
            ringOverlay_.outlines.push_back(m.cornersPx);     // cyan outline
            continue;
        }
        const auto oIt = objects_.find(m.id);
        if (oIt != objects_.end()) {                                          // object marker
            // Presence re-scan counts the raw DETECTION (no pose needed) -
            // presence only asks whether the printed marker is still there.
            if (presenceScanning_) ++presenceSeen_[m.id];
            if (oIt->second.def->markerSizeMm > 0.f) {
                cv::Matx33d R; cv::Vec3d t;
                if (solveSquare(m.cornersPx, oIt->second.def->markerSizeMm * 0.5f, K, D, R, t) && t[2] > 1.0)
                    objPoses[m.id] = { R, t, m.rollRad };
            }
        }
    }

    // --- 2. World -> camera pose (corner correspondences, config geometry) -----
    // Corners give 4 points per marker, so ONE world marker is already enough
    // for a pose (the old centre-based solve needed >= 4 markers).
    if (worldMarkerCount_ >= 2) {
        // Multi-marker, coplanar board (all corners on y = 0): DETERMINISTIC
        // IPPE solve - both planar solutions examined explicitly, mirror
        // rejected by the above-the-floor gate, one deterministic outlier pass,
        // LM polish (SolvePlanarWorldPose). solvePnPRansac remains ONLY as the
        // fallback for a non-coplanar set (legacy wall markers) or an IPPE
        // failure: its RANDOM inlier subsets made the pose jump frame-to-frame
        // whenever a biased marker sat near the threshold - the coherent
        // "trained wireframes jump together" symptom.
        bool planar = true;
        for (const auto& p : worldObj)
            if (std::abs(p.y) > 1e-3f) { planar = false; break; }

        if (!planar || !SolvePlanarWorldPose(worldObj, worldImg)) {
            cv::Vec3d rvec, tvec;
            std::vector<int> inliers;
            try {
                if (cv::solvePnPRansac(worldObj, worldImg, K, D, rvec, tvec, false, 100,
                                       kRansacReprojPx, 0.99, inliers)
                    && finite3(rvec) && finite3(tvec)) {
                    // LM polish over the RANSAC inliers.
                    if (inliers.size() >= 4) {
                        std::vector<cv::Point3f> inObj; inObj.reserve(inliers.size());
                        std::vector<cv::Point2f> inImg; inImg.reserve(inliers.size());
                        for (const int idx : inliers) {
                            inObj.push_back(worldObj[idx]);
                            inImg.push_back(worldImg[idx]);
                        }
                        cv::solvePnPRefineLM(inObj, inImg, K, D, rvec, tvec);
                    }
                    if (finite3(rvec) && finite3(tvec)) {
                        cv::Mat Rm; cv::Rodrigues(rvec, Rm);
                        const cv::Matx33d Rwc = toMatx33(Rm);
                        if (CameraAboveFloor(Rwc, tvec)) {
                            worldR_ = Rwc;
                            worldT_ = tvec;
                            poseOk_ = true;
                        }
                    }
                }
            } catch (const cv::Exception&) {
                poseOk_ = false;   // degenerate correspondence set - treat as no pose
            }
        }
    } else if (worldMarkerCount_ == 1) {
        // Single marker: its own IPPE square pose composed with its known world
        // placement. R_world->cam = R_marker->cam * R_marker->world^T. No RANSAC
        // is possible with 4 points, so gate on the reprojection error instead
        // (plus the same above-the-floor gate as the multi-marker path).
        cv::Matx33d Rmc; cv::Vec3d tmc;
        if (solveSquare(soloMk->cornersPx, objCfg_.worldMarkerSizeMm * 0.5f, K, D, Rmc, tmc)
            && tmc[2] > 1.0) {
            const cv::Matx33d Rwc = Rmc * soloGeom->Rmw.t();
            const cv::Vec3d   twc = tmc - Rwc * soloGeom->centerWorld;
            double errPx = 0.0;
            int    nProj = 0;
            for (int k = 0; k < 4; ++k) {
                cv::Point2f p;
                if (projectMarkerPt(Rwc, twc, soloGeom->cornersWorld[k], fx, fy, cx, cy, p)) {
                    errPx += cv::norm(p - soloMk->cornersPx[k]);
                    ++nProj;
                }
            }
            if (nProj == 4 && errPx / 4.0 <= kRansacReprojPx && finite3(twc)
                && CameraAboveFloor(Rwc, twc)) {
                worldR_ = Rwc;
                worldT_ = twc;
                poseOk_ = true;
            }
        }
    }

    // --- 2a'. Pose diagnostics ---------------------------------------------------
    // Count frames where world markers were seen but no pose was accepted
    // (exposes intermittent solve failures), and compute each detected marker's
    // mean corner reprojection error against the accepted pose - a marker
    // consistently above its neighbours has a biased marker_positions entry
    // (bias the corner-std probe cannot see). Consumed by the 'D' probe below
    // and GetWorldReprojErrors().
    if (worldMarkerCount_ >= 1 && !poseOk_) ++poseFailCount_;
    if (poseOk_) {
        for (size_t m = 0; m < worldIds_.size(); ++m) {
            double em = 0.0;
            bool   ok = true;
            for (int k = 0; k < 4 && ok; ++k) {
                cv::Point2f p;
                ok = projectMarkerPt(worldR_, worldT_, worldObj[4 * m + k], fx, fy, cx, cy, p);
                if (ok) em += cv::norm(p - worldImg[4 * m + k]);
            }
            if (ok) worldReprojErr_[worldIds_[m]] = em / 4.0;
        }
    }

    // --- 2a''. Corner-jitter probe ('D') ----------------------------------------
    // Accumulates the RAW detected corners of the probe world markers plus their
    // per-frame reprojection error, and when the window completes prints two
    // copy/paste rows: per-marker corner std [px] (= sqrt(mean over the 4
    // corners of (var_x + var_y)); random noise) and per-marker mean
    // reprojection error [px] (bias - a high outlier = bad marker_positions
    // entry). "nan" = marker seen < 2 frames / never with a pose.
    if (probeActive_) {
        for (const auto& m : markers) {
            for (const int pid : kJitterProbeIds) {
                if (m.id != pid) continue;
                CornerStat& st = probeStats_[m.id];
                for (int k = 0; k < 4; ++k) {
                    const double x = m.cornersPx[k].x, y = m.cornersPx[k].y;
                    st.s[2 * k]      += x;
                    st.ss[2 * k]     += x * x;
                    st.s[2 * k + 1]  += y;
                    st.ss[2 * k + 1] += y * y;
                }
                st.n++;
                if (const auto eIt = worldReprojErr_.find(pid); eIt != worldReprojErr_.end()) {
                    st.esum += eIt->second;
                    st.en++;
                }
                break;
            }
        }
        if (--probeFramesLeft_ <= 0) {
            probeActive_ = false;
            std::ostringstream ids, ns, vals, errs;
            bool first = true;
            for (const int pid : kJitterProbeIds) {
                if (!first) { ids << ","; ns << ","; vals << ", "; errs << ", "; }
                first = false;
                ids << pid;
                const auto it = probeStats_.find(pid);
                if (it == probeStats_.end()) {
                    ns << 0;
                    vals << "nan";
                    errs << "nan";
                    continue;
                }
                const CornerStat& st = it->second;
                ns << st.n;
                if (st.n < 2) {
                    vals << "nan";
                } else {
                    double var = 0.0;   // sum of the 8 per-coordinate variances
                    for (int c = 0; c < 8; ++c) {
                        const double mean = st.s[c] / st.n;
                        var += std::max(0.0, st.ss[c] / st.n - mean * mean);
                    }
                    vals << std::fixed << std::setprecision(3) << std::sqrt(var / 4.0);
                }
                if (st.en < 1) errs << "nan";
                else           errs << std::fixed << std::setprecision(3) << (st.esum / st.en);
            }
            std::cout << "[JITTER] corner std [px] over " << kJitterProbeFrames
                      << " frames, ids " << ids.str() << " (samples " << ns.str() << "):\n"
                      << "[JITTER] " << vals.str() << "\n"
                      << "[JITTER] mean reprojection error vs world pose [px] "
                         "(bias check - one high outlier = bad marker_positions entry):\n"
                      << "[JITTER] " << errs.str() << "\n";
        }
    }

    // --- 2a. Known-layout reprojection (diagnostic, config show_known_layout) --
    // Expected outline of EVERY configured world marker through the solved pose;
    // squares that miss the printed markers reveal a wrong marker size /
    // position / mounting convention at a glance. Only squares fully inside the
    // frame are kept (mirrors the ArUcoTest overlay).
    if (objCfg_.showKnownLayout && poseOk_) {
        const float W = static_cast<float>(camCfg_.width);
        const float H = static_cast<float>(camCfg_.height);
        for (const auto& [id, g] : worldGeom_) {
            std::array<cv::Point2f, 4> q{};
            bool ok = true;
            for (int k = 0; k < 4 && ok; ++k)
                ok = projectMarkerPt(worldR_, worldT_, g.cornersWorld[k], fx, fy, cx, cy, q[k])
                     && q[k].x >= 0.f && q[k].x < W && q[k].y >= 0.f && q[k].y < H;
            if (ok) knownLayoutOutlines_.push_back(q);
        }
    }

    // --- 2b. Ring fingertip (live-measured hand tracking) ----------------------
    // Base marker pose directly when visible; otherwise reconstructed from the
    // second marker via the base<-second rigid transform (learned live whenever
    // both are seen together, seeded from config relationship_angle until then).
    // The fingertip (fingertip_offset in the base marker frame) feeds
    // ControllerHandler::SetFingertipOffsetOverride() in main.cpp, replacing the
    // rigid Cal3 camera->fingertip offset while available.
    if (ringEnabled && (haveBase || haveSecond)) {
        const float rh = ring.markerSizeMm * 0.5f;
        cv::Matx33d Rbase, Rsec; cv::Vec3d tbase, tsec;
        const bool basePose = haveBase &&
            solveSquare(baseCorners, rh, K, D, Rbase, tbase) && tbase[2] > 1.0;
        const bool secPose  = haveSecond &&
            solveSquare(secondCorners, rh, K, D, Rsec, tsec) && tsec[2] > 1.0;

        // Learn/refresh the base<-second transform (X_base = R*X_sec + t) from
        // the two live poses: R = Rb^T*Rs, t = Rb^T*(ts - tb). Absorbs mount
        // build tolerance that the config-derived seed cannot capture.
        if (basePose && secPose) {
            ringRelR_       = Rbase.t() * Rsec;
            ringRelT_       = Rbase.t() * (tsec - tbase);
            ringRelLearned_ = true;
        }

        cv::Matx33d Rring; cv::Vec3d tring;
        bool havePose = false, fromSecond = false;
        if (basePose) {
            Rring = Rbase; tring = tbase; havePose = true;
        } else if (secPose) {
            // R_base->cam = R_sec->cam * R^T;  t_base->cam = t_sec - R_base->cam * t.
            Rring      = Rsec * ringRelR_.t();
            tring      = tsec - Rring * ringRelT_;
            havePose   = finite3(tring) && tring[2] > 1.0;
            fromSecond = true;
        }

        if (havePose) {
            const cv::Point3f fo = ring.fingertipOffsetMm;
            const cv::Vec3d   Xc = Rring * cv::Vec3d(fo.x, fo.y, fo.z) + tring;
            if (finite3(Xc) && Xc[2] > 1.0) {
                ringFingertipCamYup_ = cv::Point3f(static_cast<float>(Xc[0]),
                                                   static_cast<float>(-Xc[1]),   // Y-down -> Y-up
                                                   static_cast<float>(Xc[2]));
                hasRingFingertip_ = true;
            }

            // Overlay arrow: head at the fingertip, tail back along the marker
            // Y axis at marker-centre height (the ArUcoTest arrow geometry).
            // The fingertip pixel (error-vector line anchor) only needs the
            // tip, so it is set independently of the tail projection.
            cv::Point2f tipPx, tailPx;
            if (hasRingFingertip_ &&
                projectMarkerPt(Rring, tring, fo, fx, fy, cx, cy, tipPx)) {
                ringOverlay_.fingertipPx    = tipPx;
                ringOverlay_.hasFingertipPx = true;
                if (projectMarkerPt(Rring, tring, {fo.x, 0.f, fo.z}, fx, fy, cx, cy, tailPx)) {
                    ringOverlay_.arrowTip  = tipPx;
                    ringOverlay_.arrowTail = tailPx;
                    ringOverlay_.hasArrow  = true;

                    // Pointing ray: kRayLenMm beyond the fingertip along the
                    // arrow direction (tail -> tip = marker Y through fo.y).
                    // Projected in 3D so it foreshortens correctly when the
                    // finger tilts toward/away from the overhead camera.
                    if (std::abs(fo.y) > 1e-3f) {
                        constexpr float kRayLenMm = 400.0f;
                        const float     s = 1.0f + kRayLenMm / std::abs(fo.y);
                        const cv::Point3f rayEndMarker(fo.x, fo.y * s, fo.z);
                        cv::Point2f rayPx;
                        if (projectMarkerPt(Rring, tring, rayEndMarker, fx, fy, cx, cy, rayPx)) {
                            ringOverlay_.rayEnd = rayPx;
                            ringOverlay_.hasRay = true;
                        }
                    }
                }
            }

            // Ground-plane hit: where the finger's pointing ray strikes the
            // world board (world y = 0), so the operator can see the aim point
            // in physical space. The ray is the finger axis through the
            // fingertip (marker origin fo, direction +Y, tail -> tip). Both the
            // ring pose (Rring/tring) and the world pose (worldR_/worldT_) are
            // in the same OpenCV camera frame, so the intersection is solved
            // there directly and projected with the same pinhole model.
            if (poseOk_ && std::abs(fo.y) > 1e-3f) {
                const cv::Vec3d P   = Rring * cv::Vec3d(fo.x, fo.y, fo.z) + tring;  // fingertip
                const cv::Vec3d dir = Rring * cv::Vec3d(0.0, fo.y, 0.0);            // pointing dir
                // Ground plane in camera coords: normal = world +Y (worldR_
                // column 1); the world origin maps to worldT_, a point on it.
                const cv::Vec3d n(worldR_(0, 1), worldR_(1, 1), worldR_(2, 1));
                const double    denom = n.dot(dir);
                if (std::abs(denom) > 1e-9) {
                    const double u = -n.dot(P - worldT_) / denom;
                    if (u > 0.0) {   // ahead of the fingertip along the pointing dir
                        const cv::Vec3d X = P + u * dir;
                        cv::Point2f     gpx;
                        if (X[2] > 1.0 &&
                            projectMarkerPt(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0),
                                            cv::Point3f(static_cast<float>(X[0]),
                                                        static_cast<float>(X[1]),
                                                        static_cast<float>(X[2])),
                                            fx, fy, cx, cy, gpx)) {
                            ringOverlay_.groundHit    = gpx;
                            ringOverlay_.hasGroundHit = true;
                        }
                    }
                }
            }
            ringOverlay_.visible    = hasRingFingertip_;
            ringOverlay_.fromSecond = fromSecond;
            ringFromSecond_         = fromSecond;

            // Arrow-frame error rotation for this frame's ring pose:
            // cam(Y-up) -> arrow = trim * R_marker->cam^T * diag(1,-1,1); the
            // diag flips the caller's Y-up camera vector into the OpenCV
            // Y-down frame Rring lives in (right-multiplying negates col 1).
            cv::Matx33d A = arrowTrimM_ * Rring.t();
            A(0, 1) = -A(0, 1);
            A(1, 1) = -A(1, 1);
            A(2, 1) = -A(2, 1);
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    camYupToArrowR_(i, j) = static_cast<float>(A(i, j));
        }
    }

    // Ring-fingertip coast: hold the last measured fingertip for a short window
    // when BOTH ring markers drop out (same kCoastFrames policy as trained
    // objects), so a one-frame detection hiccup doesn't pulse guidance off/on.
    // After the window the fingertip goes invalid - guidance must CUT, never
    // fall back to a camera-co-located offset model (the camera is overhead,
    // not on the ring). The coast holds POSITION only; a fast-moving hand makes
    // the held value stale, which is why the window is short.
    if (hasRingFingertip_) {
        lastRingFingertip_ = ringFingertipCamYup_;
        lastArrowR_        = camYupToArrowR_;
        hasLastRing_       = true;
        ringCoastLeft_     = kCoastFrames;
    } else if (ringEnabled && hasLastRing_ && ringCoastLeft_ > 0) {
        --ringCoastLeft_;
        ringFingertipCamYup_ = lastRingFingertip_;
        camYupToArrowR_      = lastArrowR_;
        hasRingFingertip_    = true;
        ringCoasting_        = true;
        // Reproject the held fingertip so the operator-view error-vector line
        // keeps its anchor through the coast (Y-up camera frame -> image
        // Y-down, same pinhole model as everywhere else).
        if (lastRingFingertip_.z > 1.0f) {
            const double px = fx * lastRingFingertip_.x / lastRingFingertip_.z + cx;
            const double py = cy - fy * lastRingFingertip_.y / lastRingFingertip_.z;
            if (std::isfinite(px) && std::isfinite(py)) {
                ringOverlay_.fingertipPx    = cv::Point2f(static_cast<float>(px),
                                                          static_cast<float>(py));
                ringOverlay_.hasFingertipPx = true;
            }
        }
    }

    // --- 2c. Training burst ('t') ----------------------------------------------
    // Accumulate a marker->world sample per visible object while a burst is
    // active. Frames without a world pose contribute nothing but still count
    // down, so a burst is always time-bounded. On expiry, average each sampled
    // object's pose (translation mean, rotation SVD-orthonormalized) into a
    // LOCKED anchor. This is the ONLY place anchors are created.
    if (training_) {
        if (poseOk_) {
            for (const auto& [id, op] : objPoses) {
                TrainAccum& acc = trainAcc_[id];
                acc.Rsum += worldR_.t() * op.R;
                acc.tsum += worldR_.t() * (op.t - worldT_);
                acc.n++;
            }
        }
        if (--trainFramesLeft_ <= 0) {
            training_         = false;
            trainFramesLeft_  = 0;
            lastTrainedCount_ = 0;
            for (const auto& [id, acc] : trainAcc_) {
                if (acc.n <= 0) continue;
                const auto oIt = objects_.find(id);
                if (oIt == objects_.end()) continue;
                ObjectAnchor& a = oIt->second.anchor;
                a.R   = Orthonormalize(acc.Rsum * (1.0 / acc.n));
                a.t   = acc.tsum * (1.0 / acc.n);
                a.has = true;
                ++lastTrainedCount_;
            }
            trainAcc_.clear();
        }
    }

    // --- 2c2. World-marker mask burst ('w') -------------------------------------
    // Countdown decrements once per Update() (detection frame) like the training
    // burst. On expiry every world marker seen on >= kWorldScanMinSeenFrames
    // frames becomes one averaged, padded pixel quad; DisplayHandler fills those
    // solid white for the rest of the run (see the header's mask notes).
    if (worldScanning_) {
        if (--worldScanFramesLeft_ <= 0) {
            worldScanning_       = false;
            worldScanFramesLeft_ = 0;
            worldMaskQuads_.clear();
            for (const auto& [id, acc] : worldScanAcc_) {
                if (acc.n < kWorldScanMinSeenFrames) continue;
                std::array<cv::Point2f, 4> quad{};
                cv::Point2d                c(0.0, 0.0);
                for (int k = 0; k < 4; ++k) {
                    quad[k] = cv::Point2f(static_cast<float>(acc.sum[k].x / acc.n),
                                          static_cast<float>(acc.sum[k].y / acc.n));
                    c += cv::Point2d(quad[k]);
                }
                c *= 0.25;
                // Push each corner outward from the quad centre so the fill also
                // covers the marker's outer anti-aliased edge.
                for (int k = 0; k < 4; ++k)
                    quad[k] = cv::Point2f(quad[k]) +
                              (cv::Point2f(quad[k]) - cv::Point2f(c)) * kWorldMaskPadFrac;
                worldMaskQuads_.push_back(quad);
            }
            worldScanAcc_.clear();
        }
    }

    // --- 2d. Presence re-scan ('r') ---------------------------------------------
    // Countdown decrements once per Update() (detection frame) like the training
    // burst, so a scan is always time-bounded. On expiry, TRAINED objects whose
    // marker was detected on >= kPresenceMinSeenFrames frames become the present
    // set; main.cpp consumes it for the random pick (removed objects drop out).
    if (presenceScanning_) {
        if (--presenceFramesLeft_ <= 0) {
            presenceScanning_   = false;
            presenceFramesLeft_ = 0;
            presentIds_.clear();
            for (const auto& [id, n] : presenceSeen_) {
                if (n < kPresenceMinSeenFrames) continue;
                if (!IsObjectScanned(id)) continue;   // only trained objects can guide
                presentIds_.push_back(id);
            }
            presenceSeen_.clear();
            presenceResultReady_ = true;
        }
    }

    // --- 3. Per object: trained anchor (locked) or live preview; overlay + target
    for (auto& [id, rt] : objects_) {
        const ObjectDef& obj = *rt.def;
        const auto oIt = objPoses.find(id);
        const bool live    = (oIt != objPoses.end());
        const bool trained = rt.anchor.has;

        // Pose used this frame. A TRAINED object renders and guides from its
        // locked anchor through the current world pose - the live marker only
        // recolors the wireframe. If the world pose drops out it COASTS on the
        // last rendered pose for a short window (then hides); it NEVER falls
        // back to its own live marker pose - switching reference systems makes
        // the wireframe snap, since the small marker's solo IPPE pose is far
        // noisier and tilt-ambiguous. An UNTRAINED object draws a live preview
        // while its marker is visible (aim before pressing 't') but never guides.
        cv::Matx33d Ruse; cv::Vec3d tuse;
        bool have       = false;
        bool fromAnchor = false;

        if (trained && poseOk_) {
            Ruse = worldR_ * rt.anchor.R;
            tuse = worldR_ * rt.anchor.t + worldT_;
            have       = true;
            fromAnchor = true;
            rt.lastR     = Ruse;
            rt.lastT     = tuse;
            rt.hasLast   = true;
            rt.coastLeft = kCoastFrames;
        } else if (trained && rt.hasLast && rt.coastLeft > 0) {
            Ruse = rt.lastR;
            tuse = rt.lastT;
            --rt.coastLeft;
            have       = true;
            fromAnchor = true;
        } else if (!trained && live) {
            Ruse = oIt->second.R;
            tuse = oIt->second.t;
            have = true;
        }

        if (!have) continue;
        if (!(tuse[2] > 1.0)) continue;

        // ---- Contact detection (retrieval task) -------------------------------
        // While guidance is running to THIS object, compare its own live marker
        // pose against the LOCKED trained anchor: the anchor is where the object
        // sat when 't' was pressed, so displacement past the threshold means it
        // has since been moved - the participant reached it. The live pose is
        // mapped into the world frame exactly as the training burst does
        // (worldR_^T * (t - worldT_)) so both sides of the comparison are in the
        // same frame, and world Y (height) is dropped: a slide across the board
        // triggers, a perfectly vertical lift alone does not. Needs the object's
        // OWN marker plus a world pose in the same frame - the reaching hand is
        // exactly what hides that marker, so the test simply resumes when it
        // reappears. kContactConfirmFrames consecutive frames are required
        // because the latch is permanent for the trial.
        if (!scanning_ && id == activeId_ && trained && obj.hasTarget &&
            !rt.contactLatched && live && poseOk_ &&
            objCfg_.contactMoveThresholdMm > 0.0f) {   // <= 0 disables the cue
            const cv::Vec3d tw = worldR_.t() * (oIt->second.t - worldT_);
            const double    dx = tw[0] - rt.anchor.t[0];
            const double    dz = tw[2] - rt.anchor.t[2];
            const double    dHoriz = std::sqrt(dx * dx + dz * dz);
            if (std::isfinite(dHoriz) && dHoriz > objCfg_.contactMoveThresholdMm) {
                if (++rt.contactRun >= kContactConfirmFrames) rt.contactLatched = true;
            } else {
                rt.contactRun = 0;
            }
        }

        // ---- Overlay geometry (green = marker seen, yellow = held anchor) -----
        ObjectOverlay ov;
        ov.id       = id;
        ov.name     = obj.name;
        ov.visible  = live;
        ov.anchored = fromAnchor && !live;
        ov.trained  = trained;
        ov.active   = (id == activeId_);
        ov.contact  = rt.contactLatched;
        ov.worldRefCount = poseOk_ ? worldMarkerCount_ : 0;

        for (const auto& [A, B] : rt.edges) {
            cv::Point2f a, b;
            if (projectMarkerPt(Ruse, tuse, A, fx, fy, cx, cy, a) &&
                projectMarkerPt(Ruse, tuse, B, fx, fy, cx, cy, b))
                ov.edges.emplace_back(a, b);
        }

        if (!live) {
            const float mh = obj.markerSizeMm * 0.5f;
            const cv::Point3f mc[4] = {
                {-mh, mh, 0.f}, {mh, mh, 0.f}, {mh, -mh, 0.f}, {-mh, -mh, 0.f}
            };
            bool ok = true;
            for (int k = 0; k < 4 && ok; ++k)
                ok = projectMarkerPt(Ruse, tuse, mc[k], fx, fy, cx, cy, ov.outline[k]);
            ov.hasOutline = ok;
        }

        if (obj.type == ObjectType::Cylinder && obj.cylGeneral) {
            const cv::Point3f b = obj.baseOrigin;
            const float       L = obj.markerSizeMm;
            if (projectMarkerPt(Ruse, tuse, b, fx, fy, cx, cy, ov.gizmoO) &&
                projectMarkerPt(Ruse, tuse, {b.x + L, b.y, b.z}, fx, fy, cx, cy, ov.gizmoX) &&
                projectMarkerPt(Ruse, tuse, {b.x, b.y + L, b.z}, fx, fy, cx, cy, ov.gizmoY) &&
                projectMarkerPt(Ruse, tuse, {b.x, b.y, b.z + L}, fx, fy, cx, cy, ov.gizmoZ))
                ov.hasGizmo = true;
        }

        if (obj.hasTarget) {
            cv::Point2f tpx;
            if (projectMarkerPt(Ruse, tuse, obj.targetPoint, fx, fy, cx, cy, tpx)) {
                ov.targetDot    = tpx;
                ov.hasTargetDot = true;
            }
        }

        cv::Point2f lbl;
        if (projectMarkerPt(Ruse, tuse, {0.f, 0.f, 0.f}, fx, fy, cx, cy, lbl)) {
            ov.labelPos = lbl;
            ov.hasLabel = true;
        }

        overlays_.push_back(std::move(ov));

        // ---- Active-object guidance target -----------------------------------
        // TRAINED objects only (an untrained preview never guides), and
        // suppressed during the scan phase until the operator confirms the scan
        // (FinishScan()). Resolved through the anchor + world pose; during a
        // brief world-pose dropout it rides the coasted pose above.
        if (!scanning_ && id == activeId_ && obj.hasTarget && trained) {
            const cv::Vec3d Xc =
                Ruse * cv::Vec3d(obj.targetPoint.x, obj.targetPoint.y, obj.targetPoint.z) + tuse;
            if (finite3(Xc) && Xc[2] > 1.0) {
                targetPosMm_ = cv::Point3f(static_cast<float>(Xc[0]),
                                           static_cast<float>(-Xc[1]),   // Y-down -> Y-up
                                           static_cast<float>(Xc[2]));
                hasTarget_  = true;
                targetLive_ = live;

                // Effective world->camera rotation for the full-pose fingertip
                // path (main.cpp rotates the Cal3 offset by this). Prefer the
                // world-board pose; during a coast, reconstruct it from the
                // coasted anchor-based pose (anchor.R = R_object->world):
                // R_world->cam = R_object->cam * R_object->world^T.
                if ( poseOk_ ) {
                    effectiveWorldR_    = worldR_;
                    hasEffectiveWorldR_ = true;
                } else if ( rt.anchor.has ) {
                    effectiveWorldR_    = Ruse * rt.anchor.R.t();
                    hasEffectiveWorldR_ = true;
                }

                // Roll used to orient the Cal3 offset: take the CAMERA roll from
                // the world-board pose (image angle of world +X) - this is
                // mounting-independent, so every object orients the offset the same
                // way, unlike the object marker's own roll which bakes in however
                // its sticker is oriented. A constant world-board-vs-touchscreen
                // reference offset is trimmed by rollOffsetRad_ (roll_offset_deg).
                // Fall back to the object marker's top-edge roll only when no world
                // board is visible this frame (live object, no world pose).
                if (poseOk_) {
                    targetRoll_ = static_cast<float>(std::atan2(worldR_(1, 0), worldR_(0, 0)))
                                  + rollOffsetRad_;
                } else {
                    const float mh = obj.markerSizeMm * 0.5f;
                    cv::Point2f c0, c1;
                    if (projectMarkerPt(Ruse, tuse, {-mh, mh, 0.f}, fx, fy, cx, cy, c0) &&
                        projectMarkerPt(Ruse, tuse, { mh, mh, 0.f}, fx, fy, cx, cy, c1))
                        targetRoll_ = std::atan2(c1.y - c0.y, c1.x - c0.x);
                    else
                        targetRoll_ = 0.0f;
                }
            }
        }
    }

    // Both halves of the guidance error vector are final now - test for overshoot.
    UpdateOvershoot();
}

// =============================================================================
// UpdateOvershoot - has the fingertip reached PAST the active object?
// =============================================================================

void WorldObjectHandler::UpdateOvershoot() {
    overshooting_ = false;
    overshootMm_  = 0.0f;

    // Needs both halves of the error vector; either missing means guidance is
    // cut this frame and there is nothing meaningful to test.
    if (!hasTarget_ || !hasRingFingertip_) return;

    // Δp = target - fingertip (camera frame, Y-up) rotated into the ring's arrow
    // frame: z is the reach still to go along the finger's pointing direction, so
    // -z is how far the fingertip has already travelled PAST the target. Lateral
    // deviation (x/y) is deliberately not part of the test - it is the aiming
    // error the guidance is already correcting, not a reach-distance measure.
    const cv::Vec3f dCam(targetPosMm_.x - ringFingertipCamYup_.x,
                         targetPosMm_.y - ringFingertipCamYup_.y,
                         targetPosMm_.z - ringFingertipCamYup_.z);
    const cv::Vec3f dArrow = camYupToArrowR_ * dCam;
    if (!std::isfinite(dArrow[2])) return;

    overshootMm_ = -dArrow[2];
    // A non-positive threshold disables the cue (see config overshoot_threshold_mm).
    overshooting_ = objCfg_.overshootThresholdMm > 0.0f &&
                    overshootMm_ > objCfg_.overshootThresholdMm;
}
