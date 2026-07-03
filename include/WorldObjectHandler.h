#pragma once

// =============================================================================
// WorldObjectHandler.h - OBJECTS mode: clean global-PnP object localisation
//
// Mirrors FittsTaskHandler's shape for the OBJECTS ('O') task. Physical objects
// (ids 50-90), each tagged by an ArUco marker, are guided to; a physical ArUco
// "world board" (ids 1-36, DICT_6X6_100, positions in config.yaml) provides a
// persistent reference so an object stays localized when its own marker is
// occluded by the reaching hand.
//
// LOCALISATION MODEL - global PnP, NO filtering (zero lag):
//   * World->camera pose: solved each frame from the detected world markers'
//     CORNERS <-> their config-derived world corner positions (centre +
//     world_marker_size_mm + mounting convention: wall markers upright with the
//     top edge along +Y, floor markers with the top edge toward the wall, -Z).
//     With >= 2 markers a solvePnPRansac rejects a misdetected marker; with a
//     SINGLE marker its own IPPE square pose is composed with its known world
//     placement (reprojection-gated), so ONE visible world marker is enough.
//     (Config accuracy matters: ~1 mm positions give sub-mm/steady pose;
//     several-mm errors reintroduce jitter.)
//   * Object pose: when the object marker is VISIBLE its own pose (solved from
//     its 4 corners) is used DIRECTLY - it tracks the marker instantly, with no
//     averaging or smoothing. Each visible frame also refreshes a single-frame
//     "anchor" (object pose expressed in the world frame). When the marker is
//     OCCLUDED the object is reconstructed from that held anchor via the current
//     world pose, so it stays put while the reaching hand covers it.
//
//   There is NO temporal filtering anywhere: a visible object never lags the
//   marker, and an occluded object holds its last-seen world pose (steady because
//   the world pose is steady, not because anything is smoothed).
//
// SCAN/TRAINING PHASE: entering OBJECTS starts in a scanning phase (Reset()).
// The operator sweeps the camera over the field; every object marker seen in the
// same frame as a valid world pose gets its pose (and so config target_point_mm)
// mapped into the world frame via the anchor mechanism. Guidance targets stay
// disabled until FinishScan() (ENTER), after which target selection is meant to
// pick among the scanned objects - their target persists off any single world
// marker even if the object marker never reappears. Anchors keep refreshing
// whenever an object marker is re-seen (a moved object re-maps automatically).
//
// All PnP runs on the MAIN thread from the corners ArucoHandler already returns
// (single detection thread; no extra threads / VideoCapture). Frame convention
// (marker frame, mm): origin = marker centre, +X right, +Y up, +Z out of the
// face. Camera/DetectedMarker are Y-up, OpenCV pose is Y-down, so Y is negated
// at the boundary (GetTargetPosMm()).
// =============================================================================

#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "ArucoHandler.h"   // DetectedMarker
#include "Config.h"         // ObjectWorldConfig, ObjectDef, CameraConfig


// Precomputed overlay geometry for one object, projected into the camera image
// this frame. Consumed by DisplayHandler. All points are operator-view pixels.
struct ObjectOverlay {
    int         id = -1;
    std::string name;
    bool        visible  = false;   // object marker directly detected this frame
    bool        anchored = false;   // reconstructed from the world board (marker not seen)
    bool        active   = false;   // this object is the current guidance target
    int         worldRefCount = 0;  // world markers backing this frame's world pose
                                    // (target-dot confidence colour; 0 = no world pose)

    std::vector<std::pair<cv::Point2f, cv::Point2f>> edges;   // wireframe

    bool                       hasOutline = false;            // marker square (when anchored)
    std::array<cv::Point2f, 4> outline{};

    bool        hasGizmo = false;                             // base-origin XYZ (general cyl)
    cv::Point2f gizmoO{}, gizmoX{}, gizmoY{}, gizmoZ{};

    bool        hasTargetDot = false;                         // guidance-target dot
    cv::Point2f targetDot{};

    bool        hasLabel = false;                             // name label anchor
    cv::Point2f labelPos{};
};


class WorldObjectHandler {
public:
    WorldObjectHandler(const ObjectWorldConfig& objCfg,
                       const CameraConfig&      camCfg);

    /** @brief Call when entering OBJECTS state - clears held anchors and target,
     *         and (re)starts the scan/training phase. */
    void Reset();

    // ---- Scan/training phase (see the header comment) ------------------------
    bool IsScanning() const { return scanning_; }
    /** @brief End the scan phase (ENTER). @return number of objects mapped. */
    int  FinishScan();
    /** @brief True once `id` has a world-frame anchor (was seen together with a
     *         valid world pose), i.e. its target survives occlusion. */
    bool IsObjectScanned( int id ) const;
    int  ScannedCount() const;
    /** @brief One-line scan progress for the Output row / HUD. */
    std::string GetScanStatus() const;

    /** @brief Call when a new object marker is selected ('m' / 'r'). Held anchors
     *         of all objects are kept. */
    void OnNewTarget(int objectMarkerId);

    /** @brief Process one loop iteration while in OBJECTS state: solve the world
     *         pose, use each visible object's live pose (refreshing its anchor),
     *         reconstruct occluded objects from their held anchor, build overlays. */
    void Update(const std::vector<DetectedMarker>& markers);

    // ---- Guidance target (for ControllerHandler::SetTarget) -----------------
    bool        HasTarget()      const { return hasTarget_; }
    cv::Point3f GetTargetPosMm() const { return targetPosMm_; }   // camera frame, Y-up
    float       GetTargetRoll()  const { return targetRoll_; }
    bool        TargetIsLive()   const { return targetLive_; }    // object marker itself seen
    int         GetActiveObjectId() const { return activeId_; }

    // ---- Diagnostics (HUD / console) ----------------------------------------
    int  GetWorldMarkerCount() const { return worldMarkerCount_; }   // world markers seen this frame
    bool HasWorldPose()        const { return poseOk_; }             // world->cam solved this frame
    bool HasActiveAnchor()     const;                                // active object has a held anchor

    // ---- Effective world->camera rotation (OBJECTS full-pose fingertip) -----
    // The best available world->camera rotation for the active target this frame,
    // used by main.cpp to rotate the Cal3 fingertip offset (via the rig-alignment
    // R_screen->world) instead of the legacy scalar roll. Prefers the world-board
    // pose; if the board isn't solved this frame but the object marker is live and
    // has a stored world anchor, it is reconstructed from the object's own pose and
    // that anchor. Invalid (false) when neither is available.
    bool        HasEffectiveWorldR() const { return hasEffectiveWorldR_; }
    cv::Matx33d GetEffectiveWorldR() const { return effectiveWorldR_; }

    // ---- Overlay geometry (for DisplayHandler) ------------------------------
    const std::vector<ObjectOverlay>& GetOverlays() const { return overlays_; }
    /** @brief Detected world-board marker outlines (4 image-px corners each) this
     *         frame, for the faint blue world-marker overlay. */
    const std::vector<std::array<cv::Point2f, 4>>& GetWorldOutlines() const { return worldOutlines_; }

private:
    // Last-seen object pose expressed in the world frame (object -> world). A
    // single frame's estimate (no averaging), refreshed every visible frame and
    // held while the marker is occluded.
    struct ObjectAnchor {
        cv::Matx33d R = cv::Matx33d::eye();
        cv::Vec3d   t;
        bool        has = false;
    };

    struct ObjectRuntime {
        const ObjectDef*                                 def = nullptr;
        std::vector<std::pair<cv::Point3f, cv::Point3f>> edges;   // marker frame
        ObjectAnchor                                     anchor;
    };

    // Precomputed world-frame geometry for one world-board marker: its 4 corner
    // positions (detection order TL,TR,BR,BL) and its marker->world rotation,
    // derived from the config centre + world_marker_size_mm + the mounting
    // convention (wall upright / floor top edge toward the wall).
    struct WorldMarkerGeom {
        std::array<cv::Point3f, 4> cornersWorld{};
        cv::Matx33d                Rmw = cv::Matx33d::eye();   // marker -> world
        cv::Vec3d                  centerWorld;
    };

    static std::vector<std::pair<cv::Point3f, cv::Point3f>> BuildEdges(const ObjectDef& o);

    const ObjectWorldConfig& objCfg_;
    const CameraConfig&      camCfg_;

    std::map<int, ObjectRuntime>   objects_;     // keyed by object marker id
    std::map<int, WorldMarkerGeom> worldGeom_;   // keyed by world-board marker id
    float rollOffsetRad_ = 0.0f;             // world-board roll trim (config roll_offset_deg)
    int  activeId_ = 0;
    bool scanning_ = false;                  // scan/training phase active (no guidance target)

    // Per-frame world pose (valid only when poseOk_).
    bool        poseOk_ = false;
    cv::Matx33d worldR_;                 // world -> camera rotation
    cv::Vec3d   worldT_;
    int         worldMarkerCount_ = 0;

    // Per-frame resolved guidance target.
    bool        hasTarget_   = false;
    bool        targetLive_  = false;
    cv::Point3f targetPosMm_ = {};
    float       targetRoll_  = 0.0f;

    // Per-frame effective world->camera rotation for the active target (see the
    // HasEffectiveWorldR()/GetEffectiveWorldR() accessors).
    bool        hasEffectiveWorldR_ = false;
    cv::Matx33d effectiveWorldR_    = cv::Matx33d::eye();

    std::vector<ObjectOverlay>              overlays_;
    std::vector<std::array<cv::Point2f, 4>> worldOutlines_;
};
