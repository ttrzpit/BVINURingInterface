#include "WorldObjectHandler.h"

#include <cmath>

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
// rejected.
static constexpr float kRansacReprojPx = 4.0f;

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
    // Plane from the centre itself (y = 0 -> floor spanning X/Z, else wall at
    // z = 0 spanning X/Y); orientation from the mounting convention: wall
    // markers are upright (marker +Y = world +Y), floor markers lie with their
    // top edge toward the wall (marker +Y = world -Z, face up: +Z = world +Y).
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
    overlays_.clear();
    worldOutlines_.clear();
    for (auto& [id, rt] : objects_) rt.anchor = ObjectAnchor{};
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
    std::string names;
    int         n = 0;
    for (const auto& [id, rt] : objects_) {
        if (!rt.anchor.has) continue;
        if (n++) names += ", ";
        names += rt.def->name;
    }
    std::string s = "SCANNING - mapped " + std::to_string(n) + "/" +
                    std::to_string(static_cast<int>(objects_.size()));
    if (n) s += " (" + names + ")";
    s += " - [Enter] to finish";
    return s;
}

void WorldObjectHandler::OnNewTarget(int objectMarkerId) {
    activeId_   = objectMarkerId;
    hasTarget_  = false;
    targetLive_ = false;
    // Held anchors are kept across target changes.
}

bool WorldObjectHandler::HasActiveAnchor() const {
    const auto it = objects_.find(activeId_);
    return it != objects_.end() && it->second.anchor.has;
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
    overlays_.clear();
    worldOutlines_.clear();
    worldMarkerCount_ = 0;
    poseOk_ = false;

    const double fx = camCfg_.fx, fy = camCfg_.fy, cx = camCfg_.cx, cy = camCfg_.cy;
    const cv::Mat& K = camCfg_.cameraMatrix;
    const cv::Mat& D = camCfg_.distCoeffs;

    // --- 1. Gather detections -------------------------------------------------
    // World markers -> (world-frame corner <-> detected corner) correspondences
    // for the world pose. Object markers -> their own live pose (solved from
    // their 4 corners).
    struct OPose { cv::Matx33d R; cv::Vec3d t; float roll; };
    std::vector<cv::Point3f> worldObj;   // world-frame marker corners (config geometry)
    std::vector<cv::Point2f> worldImg;   // detected corners
    const WorldMarkerGeom*   soloGeom = nullptr;   // last world marker seen (1-marker path)
    const DetectedMarker*    soloMk   = nullptr;
    std::map<int, OPose>     objPoses;   // object id -> live pose

    for (const auto& m : markers) {
        const auto wIt = worldGeom_.find(m.id);
        if (wIt != worldGeom_.end()) {                        // world board marker
            for (int k = 0; k < 4; ++k) {
                worldObj.push_back(wIt->second.cornersWorld[k]);
                worldImg.push_back(m.cornersPx[k]);
            }
            soloGeom = &wIt->second;
            soloMk   = &m;
            worldOutlines_.push_back(m.cornersPx);
            ++worldMarkerCount_;
            continue;
        }
        const auto oIt = objects_.find(m.id);
        if (oIt != objects_.end() && oIt->second.def->markerSizeMm > 0.f) {   // object marker
            cv::Matx33d R; cv::Vec3d t;
            if (solveSquare(m.cornersPx, oIt->second.def->markerSizeMm * 0.5f, K, D, R, t) && t[2] > 1.0)
                objPoses[m.id] = { R, t, m.rollRad };
        }
    }

    // --- 2. World -> camera pose (corner correspondences, config geometry) -----
    // Corners give 4 points per marker, so ONE world marker is already enough
    // for a pose (the old centre-based solve needed >= 4 markers).
    if (worldMarkerCount_ >= 2) {
        // Multi-marker: RANSAC keeps the misdetected-marker rejection.
        cv::Vec3d rvec, tvec;
        try {
            if (cv::solvePnPRansac(worldObj, worldImg, K, D, rvec, tvec, false, 100, kRansacReprojPx)
                && finite3(rvec) && finite3(tvec)) {
                cv::Mat Rm; cv::Rodrigues(rvec, Rm);
                worldR_ = toMatx33(Rm);
                worldT_ = tvec;
                poseOk_ = true;
            }
        } catch (const cv::Exception&) {
            poseOk_ = false;   // degenerate correspondence set - treat as no pose
        }
    } else if (worldMarkerCount_ == 1) {
        // Single marker: its own IPPE square pose composed with its known world
        // placement. R_world->cam = R_marker->cam * R_marker->world^T. No RANSAC
        // is possible with 4 points, so gate on the reprojection error instead.
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
            if (nProj == 4 && errPx / 4.0 <= kRansacReprojPx && finite3(twc)) {
                worldR_ = Rwc;
                worldT_ = twc;
                poseOk_ = true;
            }
        }
    }

    // --- 3. Per object: live pose (instant) or held anchor; overlay + target ---
    for (auto& [id, rt] : objects_) {
        const ObjectDef& obj = *rt.def;
        const auto oIt = objPoses.find(id);
        const bool live = (oIt != objPoses.end());

        cv::Matx33d Ruse; cv::Vec3d tuse;
        bool have = false;

        if (live) {
            // Use the object marker's own pose directly - tracks the marker with
            // zero lag. Refresh the world-frame anchor from this single frame (no
            // averaging) so it can hold position once the marker is occluded.
            Ruse = oIt->second.R;
            tuse = oIt->second.t;
            have = true;
            if (poseOk_) {
                rt.anchor.R   = worldR_.t() * Ruse;
                rt.anchor.t   = worldR_.t() * (tuse - worldT_);
                rt.anchor.has = true;
            }
        } else if (poseOk_ && rt.anchor.has) {
            // Occluded: hold the last-seen world pose, reprojected via the current
            // (steady) world pose.
            Ruse = worldR_ * rt.anchor.R;
            tuse = worldR_ * rt.anchor.t + worldT_;
            have = true;
        }

        if (!have) continue;
        if (!(tuse[2] > 1.0)) continue;

        // ---- Overlay geometry (green = marker seen, yellow = held anchor) -----
        ObjectOverlay ov;
        ov.id       = id;
        ov.name     = obj.name;
        ov.visible  = live;
        ov.anchored = !live;
        ov.active   = (id == activeId_);
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
        // Suppressed during the scan phase: anchors are being collected, no
        // guidance until the operator confirms the scan (FinishScan()).
        if (!scanning_ && id == activeId_ && obj.hasTarget) {
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
                // world-board pose; if it isn't solved this frame but the object
                // is live with a stored anchor (anchor.R = R_object->world),
                // reconstruct R_world->cam = R_object->cam * R_object->world^T.
                if ( poseOk_ ) {
                    effectiveWorldR_    = worldR_;
                    hasEffectiveWorldR_ = true;
                } else if ( live && rt.anchor.has ) {
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
}
