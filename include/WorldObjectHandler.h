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
//   * World->camera pose: one multi-marker solvePnPRansac each frame over the
//     detected world markers' centres <-> their config positions. Uses accurate
//     config geometry, so it is steady and low-lag; RANSAC rejects a misdetected
//     marker. (Config accuracy matters: ~1 mm positions give sub-mm/steady pose;
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
// All PnP runs on the MAIN thread from the corners ArucoHandler already returns
// (single detection thread; no extra threads / VideoCapture). Frame convention
// (marker frame, mm): origin = marker centre, +X right, +Y up, +Z out of the
// face. Camera/DetectedMarker are Y-up, OpenCV pose is Y-down, so Y is negated
// at the boundary (GetTargetPosMm()).
// =============================================================================

#include <array>
#include <map>
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

    /** @brief Call when entering OBJECTS state - clears held anchors and target. */
    void Reset();

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

    static std::vector<std::pair<cv::Point3f, cv::Point3f>> BuildEdges(const ObjectDef& o);

    const ObjectWorldConfig& objCfg_;
    const CameraConfig&      camCfg_;

    std::map<int, ObjectRuntime> objects_;   // keyed by object marker id
    float rollOffsetRad_ = 0.0f;             // world-board roll trim (config roll_offset_deg)
    int  activeId_ = 0;

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

    std::vector<ObjectOverlay>              overlays_;
    std::vector<std::array<cv::Point2f, 4>> worldOutlines_;
};
