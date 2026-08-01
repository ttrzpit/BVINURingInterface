#pragma once

// =============================================================================
// WorldObjectHandler.h - OBJECTS mode: clean global-PnP object localisation
//
// Mirrors FittsTaskHandler's shape for the OBJECTS ('O') task. Physical objects
// (ids 60-72), each tagged by an ArUco marker, are guided to; a physical ArUco
// "world board" (ids 1-45, a single ground plane at y = 0, DICT_6X6_100,
// positions in config.yaml) provides a persistent reference so an object stays
// localized when its own marker is occluded by the reaching hand. Two ring
// markers (ids 73/74) on the hand track the fingertip live (see RingOverlay
// and the ring_markers section of config.yaml).
//
// LOCALISATION MODEL - global PnP, NO filtering (zero lag):
//   * World->camera pose: solved each frame from the detected world markers'
//     CORNERS <-> their config-derived world corner positions (centre +
//     world_marker_size_mm + mounting convention: ground-plane markers face-up
//     with the printed top edge toward the z = 0 row, world -Z; legacy wall
//     markers, y != 0, upright with the top edge along +Y).
//     With >= 2 markers on the (coplanar) ground plane the solve is fully
//     DETERMINISTIC: solvePnPGeneric(SOLVEPNP_IPPE) returns BOTH planar
//     solutions explicitly and the above-the-floor one with the lower
//     reprojection error is picked; one deterministic outlier pass then drops
//     any marker whose mean corner reprojection error exceeds a gate (e.g. a
//     misconfigured marker_positions entry) and re-solves; solvePnPRefineLM
//     polishes the kept set. No RANSAC randomness - a static scene yields the
//     same marker set and pose every frame (the previous solvePnPRansac's
//     random inlier subsets made the pose JUMP frame-to-frame whenever a
//     biased marker sat near the threshold). RANSAC remains only as the
//     fallback for a non-coplanar set (legacy wall markers). With a SINGLE
//     marker its own IPPE square pose is composed with its known world
//     placement (reprojection-gated), so ONE visible world marker is enough.
//     Every accepted pose must put the camera ABOVE the ground plane (world
//     +Y). (Config accuracy matters: ~1 mm positions give sub-mm/steady pose;
//     several-mm errors bias it - see the 'D' probe's reprojection row.)
//   * Object pose - TRAINED ANCHORS ONLY ('t'). An object is mapped into the
//     world frame exclusively by a training burst (StartTraining, ~train_frames
//     detection frames): every burst frame with a valid world pose contributes
//     one marker->world sample per visible object; translation is averaged and
//     rotation SVD-orthonormalized into a LOCKED anchor. A trained object is
//     thereafter rendered AND guided from anchor + current world pose - its own
//     marker being visible only recolors the wireframe - so its jitter reduces
//     to world-pose jitter and it is immune to hand occlusion. It never re-maps
//     automatically; re-press 't' after physically moving an object, 'u' clears
//     all training. If NO world pose is available this frame, a trained object
//     COASTS on its last rendered camera-frame pose for a short window (then
//     hides) - it never falls back to its own live marker pose: switching
//     reference systems mid-stream made the wireframe visibly snap, since the
//     40 mm marker's solo IPPE pose is far noisier and tilt-ambiguous.
//   * Untrained objects: a live per-frame preview is drawn while their marker is
//     visible (so the operator can aim before pressing 't'), but they never
//     produce a guidance target.
//
// SCAN/TRAINING PHASE: entering OBJECTS starts in a scanning phase (Reset()).
// The operator aims the camera so an object (or several) and at least one world
// marker are visible, presses 't', and holds steady for the burst; repeat per
// placement. Guidance targets stay disabled until FinishScan() (ENTER), after
// which target selection picks among the TRAINED objects - their target
// persists off any single world marker even if the object marker never
// reappears. 't'/'u' remain available after the scan phase for re-training.
//
// CONTACT DETECTION (retrieval task): while guidance is running to the active
// object, its own marker - when visible alongside the world board - is compared
// against its LOCKED trained anchor. Horizontal (ground-plane) displacement past
// objects.contact_move_threshold_mm on kContactConfirmFrames consecutive frames
// LATCHES ObjectOverlay::contact, which DisplayHandler renders as " CONTACT"
// after the object's name: the participant has reached the object and moved it.
// World Y is ignored (a slide triggers, a pure vertical lift does not), and the
// multi-frame confirmation keeps the 40 mm marker's solo-IPPE noise from
// latching a false positive at the small default threshold. Because the reaching
// hand is exactly what occludes the object marker, detection resumes the moment
// the marker reappears - so this marks "the object has been disturbed", not the
// instant of first touch. The latch clears on OnNewTarget ('m'/'r'),
// StartTraining ('t' - the anchor itself is re-measured), UntrainAll ('u') and
// Reset. The anchor is never re-mapped, so a displaced object keeps rendering
// (and guiding) at its trained location - the CONTACT flag is also the signal
// that the anchor has gone stale.
//
// OVERSHOOT DETECTION (retrieval task): with a resolved target AND a ring
// fingertip, the guidance error Δp = target - fingertip is rotated into the ring
// arrow frame (see GetCamYupToArrowR) and its z component - the reach still to go
// along the finger's pointing direction - is tested against
// objects.overshoot_threshold_mm. A depth that has gone NEGATIVE by more than the
// threshold means the fingertip is now past the object along that direction, and
// DisplayHandler renders "OVERSHOOT" in the top right of the operator view. The
// cue is LIVE, not latched: it clears the frame the fingertip comes back inside
// the threshold, and goes false whenever either half of the error vector is
// missing. Note the axis is the FINGER's, not the reach path's, so re-aiming the
// finger without moving the hand can flip the sign - raise the threshold if that
// shows up as flicker.
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
    bool        trained  = false;   // has a locked trained anchor (untrained = live preview)
    bool        active   = false;   // this object is the current guidance target
    bool        contact  = false;   // LATCHED: the object has been displaced from its
                                    // trained anchor while being guided to (reached)
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


// Ring-marker fingertip overlay, projected into the camera image this frame.
// Consumed by DisplayHandler. All points are operator-view pixels.
struct RingOverlay {
    bool visible    = false;   // a ring pose was resolved this frame
    bool fromSecond = false;   // base marker occluded - pose came via the second marker

    bool        hasArrow = false;   // cyan fingertip arrow (tail -> tip = fingertip)
    cv::Point2f arrowTail{}, arrowTip{};

    bool        hasRay = false;     // thin pointing ray extending past the arrow tip
    cv::Point2f rayEnd{};           //   (kRayLenMm along the finger's pointing dir)

    // Where the finger's pointing ray meets the world-board ground plane
    // (world y = 0). Valid only when a world pose is solved this frame and the
    // ray actually strikes the plane ahead of the fingertip. Drawn as a small
    // cyan circle so the operator can see where the finger is aiming in
    // physical space.
    bool        hasGroundHit = false;
    cv::Point2f groundHit{};

    // Projected fingertip pixel - valid whenever a fingertip is available
    // (live, or reprojected from the held position while COASTING, when the
    // arrow itself can't draw). Anchor of the operator-view error-vector line
    // (fingertip -> active object target), the pixel-space twin of the
    // guidance error Δp = target − fingertip.
    bool        hasFingertipPx = false;
    cv::Point2f fingertipPx{};

    // Detected ring marker squares (base and/or second), for cyan outlines.
    std::vector<std::array<cv::Point2f, 4>> outlines;
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
    /** @brief End the scan phase (ENTER). @return number of objects trained. */
    int  FinishScan();
    /** @brief True once `id` has a TRAINED world-frame anchor ('t' burst with a
     *         valid world pose), i.e. its target survives occlusion. */
    bool IsObjectScanned( int id ) const;
    int  ScannedCount() const;
    /** @brief One-line scan progress for the Output row / HUD. */
    std::string GetScanStatus() const;

    // ---- Training ('t') / untraining ('u') ------------------------------------
    /** @brief Start a training burst (config train_frames detection frames).
     *         Every burst frame with a valid world pose accumulates a
     *         marker->world sample per visible object; at the end each sampled
     *         object gets an averaged, LOCKED anchor. Restarts any burst in
     *         progress. Works during and after the scan phase. */
    void StartTraining();

    // ---- Presence re-scan ('r' random-target pre-check) -----------------------
    /** @brief Start a presence re-scan burst (config presence_scan_frames
     *         detection frames). Counts how often each object's own marker is
     *         detected; on expiry, TRAINED objects seen on at least
     *         kPresenceMinSeenFrames frames are published via GetPresentIds()
     *         and HasPresenceResult() goes true (one-shot; consume with
     *         ClearPresenceResult). A trained object whose marker was
     *         physically removed thus drops out of the 'r' random pool.
     *         Restarts any presence scan in progress. */
    void StartPresenceScan();
    bool IsPresenceScanning() const { return presenceScanning_; }
    /** @brief Detection frames remaining in the current presence scan. */
    int  PresenceFramesLeft() const { return presenceFramesLeft_; }
    /** @brief True once a presence scan has completed and the result has not
     *         been consumed yet. */
    bool HasPresenceResult()  const { return presenceResultReady_; }
    /** @brief TRAINED object ids whose marker was seen during the last
     *         completed presence scan (valid while HasPresenceResult()). */
    const std::vector<int>& GetPresentIds() const { return presentIds_; }
    void ClearPresenceResult() { presenceResultReady_ = false; }
    /** @brief Forget every trained anchor ('u'). Objects revert to live preview
     *         and stop producing guidance targets until re-trained. */
    void UntrainAll();
    bool IsTraining()          const { return training_; }
    /** @brief Detection frames remaining in the current burst (HUD countdown). */
    int  TrainingFramesLeft()  const { return trainFramesLeft_; }
    /** @brief Objects trained by the most recent completed burst. */
    int  LastTrainedCount()    const { return lastTrainedCount_; }

    // ---- Corner-jitter probe ('D') --------------------------------------------
    /** @brief Start the corner-jitter probe: over the next kJitterProbeFrames
     *         detection frames, accumulate the raw detected corner positions of
     *         world markers 1, 5, 9, 19, 27, 37, 45 and then print ONE
     *         copy/paste-friendly row of per-marker corner standard deviations
     *         [px] to the terminal. Hold the camera rigidly still while it runs.
     *         Restarts any probe in progress. */
    void StartCornerJitterProbe();

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

    // ---- Overshoot (retrieval task, see the header notes) --------------------
    /** @brief True while the fingertip has passed the active object's target by
     *         more than objects.overshoot_threshold_mm along the finger's
     *         pointing direction. LIVE (never latched); false whenever there is
     *         no target or no ring fingertip this frame. */
    bool  IsOvershooting()     const { return overshooting_; }
    /** @brief How far [mm] the fingertip is PAST the target along the pointing
     *         direction (= -Δp.z in the arrow frame): positive = beyond the
     *         target, negative = still short of it. Valid whenever
     *         HasTarget() && HasRingFingertip(); 0 otherwise. */
    float GetOvershootMm()     const { return overshootMm_; }

    // ---- Ring fingertip (for ControllerHandler::SetFingertipOffsetOverride) --
    // Live-measured fingertip position in the camera frame, Y-up (same frame as
    // GetTargetPosMm): base ring marker pose * fingertip_offset, with the second
    // marker as fallback when the base is occluded. With the camera mounted
    // ABOVE the scene (not on the ring), this is the ONLY valid fingertip
    // source for OBJECTS guidance - the error vector is
    // GetTargetPosMm() - GetRingFingertipCamYup(), both camera frame Y-up.
    // When BOTH ring markers drop out, the last fingertip COASTS for a short
    // window (kCoastFrames, ~0.25 s - same policy as trained objects) so
    // one-frame detection hiccups don't pulse guidance off/on, then goes
    // invalid until a ring marker reappears.
    bool        HasRingFingertip()      const { return hasRingFingertip_; }
    cv::Point3f GetRingFingertipCamYup() const { return ringFingertipCamYup_; }
    /** @brief True while the fingertip is riding the coast window (no ring
     *         marker detected this frame). */
    bool        RingCoasting()          const { return ringCoasting_; }
    /** @brief True when this frame's ring pose came via the second marker. */
    bool        RingFromSecond()        const { return ringFromSecond_; }

    // ---- Arrow-frame error rotation (OBJECTS guidance error frame) -----------
    // Rotation taking a CAMERA-frame Y-up vector (the frame of GetTargetPosMm /
    // GetRingFingertipCamYup) into the base ring marker's ARROW frame:
    //   X = across the finger (marker +X), Y = out of the marker face
    //   (marker +Z), Z = along the arrow, tail -> tip (marker -Y).
    // For the guidance error Δp = target - fingertip rotated by this, Δp.z is
    // the distance to go along the pointing direction and Δp.x/Δp.y the
    // lateral deviation from the pointing line (aim the arrow at the target
    // -> x,y -> 0). ring.arrowYawTrimDeg rotates the X/Y axes about the arrow
    // to align them with the device motor frame. Valid whenever
    // HasRingFingertip() (live pose, or held through the coast window).
    cv::Matx33f GetCamYupToArrowR() const { return camYupToArrowR_; }
    /** @brief True once the base<-second transform has been learned from a frame
     *         where BOTH ring markers were visible (config-derived seed replaced).
     *         Until then the second-marker fallback runs on the seed. */
    bool        RingRelLearned()        const { return ringRelLearned_; }

    // ---- Diagnostics (HUD / console) ----------------------------------------
    int  GetWorldMarkerCount() const { return worldMarkerCount_; }   // world markers seen this frame
    bool HasWorldPose()        const { return poseOk_; }             // world->cam solved this frame
    bool HasActiveAnchor()     const;                                // active object has a held anchor
    /** @brief Frames (since Reset) where >= 1 world marker was detected but no
     *         world pose was accepted - exposes intermittent solve failures that
     *         the per-frame "pose: OK/--" readout is too coarse to show. */
    uint64_t GetPoseFailCount() const { return poseFailCount_; }
    /** @brief Mean corner reprojection error [px] per DETECTED world marker
     *         against this frame's accepted world pose (empty when !poseOk_).
     *         A marker consistently far above its neighbours has a biased
     *         marker_positions entry (bias that the corner-std probe cannot
     *         see). Also accumulated into the 'D' probe's second output row. */
    const std::map<int, double>& GetWorldReprojErrors() const { return worldReprojErr_; }

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
    /** @brief Ring-marker outlines + fingertip arrow this frame (cyan overlay). */
    const RingOverlay& GetRingOverlay() const { return ringOverlay_; }
    /** @brief Expected outline of EVERY configured world marker, reprojected
     *         through the solved world pose (config show_known_layout). Empty
     *         when the flag is off or no world pose was solved this frame. */
    const std::vector<std::array<cv::Point2f, 4>>& GetKnownLayoutOutlines() const {
        return knownLayoutOutlines_;
    }

private:
    // TRAINED object pose expressed in the world frame (object -> world):
    // burst-averaged by StartTraining()/Update() and then LOCKED - it never
    // refreshes on re-sighting (re-train with 't' after moving an object).
    struct ObjectAnchor {
        cv::Matx33d R = cv::Matx33d::eye();
        cv::Vec3d   t;
        bool        has = false;
    };

    // Accumulates marker->world pose samples for one object during a training
    // burst (translation summed for the mean; rotations summed, then the mean
    // is SVD-orthonormalized back to a proper rotation).
    struct TrainAccum {
        cv::Matx33d Rsum = cv::Matx33d::zeros();
        cv::Vec3d   tsum = cv::Vec3d(0, 0, 0);
        int         n    = 0;
    };

    struct ObjectRuntime {
        const ObjectDef*                                 def = nullptr;
        std::vector<std::pair<cv::Point3f, cv::Point3f>> edges;   // marker frame
        ObjectAnchor                                     anchor;

        // Last rendered camera-frame pose of a TRAINED object + the coast
        // countdown: when the world pose drops out, the object holds this pose
        // for a short window instead of snapping to its own live marker pose
        // (see the header's localisation notes).
        cv::Matx33d lastR = cv::Matx33d::eye();
        cv::Vec3d   lastT;
        bool        hasLast   = false;
        int         coastLeft = 0;

        // Contact detection (see the header notes): consecutive frames whose
        // measured horizontal displacement from the trained anchor exceeded the
        // threshold, and the resulting LATCHED flag. Cleared by OnNewTarget /
        // StartTraining / UntrainAll / Reset.
        int  contactRun    = 0;
        bool contactLatched = false;
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

    /** @brief Deterministic multi-marker world solve for a COPLANAR corner set
     *         (the ground-plane board): IPPE both-solutions pick + one outlier
     *         pass + refineLM (see the header notes). Sets worldR_/worldT_/
     *         poseOk_ on success. @return true if the planar path handled the
     *         solve (poseOk_ may still be false = genuinely no valid pose);
     *         false = solver threw, caller should fall back to RANSAC. */
    bool SolvePlanarWorldPose(const std::vector<cv::Point3f>& worldPts,
                              const std::vector<cv::Point2f>& imgPts);

    /** @brief Recompute overshooting_/overshootMm_ from this frame's resolved
     *         target, ring fingertip and arrow-frame rotation. Called at the end
     *         of Update(), once both halves of the error vector are final. */
    void UpdateOvershoot();

    const ObjectWorldConfig& objCfg_;
    const CameraConfig&      camCfg_;

    std::map<int, ObjectRuntime>   objects_;     // keyed by object marker id
    std::map<int, WorldMarkerGeom> worldGeom_;   // keyed by world-board marker id
    float rollOffsetRad_ = 0.0f;             // world-board roll trim (config roll_offset_deg)
    int  activeId_ = 0;
    bool scanning_ = false;                  // scan/training phase active (no guidance target)

    // Training burst state ('t'). The countdown decrements once per Update()
    // (i.e. per detection frame) whether or not a world pose was solved, so a
    // burst is always time-bounded; only frames WITH a world pose contribute
    // samples.
    bool                      training_        = false;
    int                       trainFramesLeft_ = 0;
    int                       lastTrainedCount_ = 0;   // objects trained by the last burst
    std::map<int, TrainAccum> trainAcc_;

    // Presence re-scan state ('r'). Counts raw marker DETECTIONS per object id
    // (no pose needed - presence only asks "is the printed marker still
    // there?"); the countdown decrements once per Update() like the training
    // burst. An object must be seen on >= kPresenceMinSeenFrames frames to
    // count as present, so a single-frame phantom decode cannot keep a removed
    // object in the pool.
    static constexpr int kPresenceMinSeenFrames = 3;
    bool                 presenceScanning_    = false;
    int                  presenceFramesLeft_  = 0;
    bool                 presenceResultReady_ = false;
    std::map<int, int>   presenceSeen_;
    std::vector<int>     presentIds_;

    // Corner-jitter probe ('D'): running sums for each probe marker's corner
    // std. 8 coordinates per marker (4 corners x X/Y); naive sum / sum-of-
    // squares in double is plenty of precision here (~1600^2 x 300 frames).
    struct CornerStat {
        std::array<double, 8> s{};    // per-coordinate sum
        std::array<double, 8> ss{};   // per-coordinate sum of squares
        int                   n = 0;  // frames this marker was detected
        double                esum = 0.0;  // sum of mean reprojection error [px]
        int                   en   = 0;    // frames with a world pose to measure against
    };
    static constexpr int                kJitterProbeFrames = 300;
    static constexpr std::array<int, 7> kJitterProbeIds{ 1, 5, 9, 19, 27, 37, 45 };
    bool                      probeActive_     = false;
    int                       probeFramesLeft_ = 0;
    std::map<int, CornerStat> probeStats_;

    // Per-frame world pose (valid only when poseOk_).
    bool        poseOk_ = false;
    cv::Matx33d worldR_;                 // world -> camera rotation
    cv::Vec3d   worldT_;
    int         worldMarkerCount_ = 0;

    // Diagnostics: per-detected-marker mean reprojection error [px] against the
    // accepted pose (per frame), and the count of frames where world markers
    // were seen but no pose was accepted (since Reset).
    std::map<int, double> worldReprojErr_;
    uint64_t              poseFailCount_ = 0;

    // World marker ids in gather order - parallel to each group of 4 corners in
    // the worldObj/worldImg vectors built by Update() (per frame).
    std::vector<int> worldIds_;

    // Per-frame resolved guidance target.
    bool        hasTarget_   = false;
    bool        targetLive_  = false;
    cv::Point3f targetPosMm_ = {};
    float       targetRoll_  = 0.0f;

    // Per-frame overshoot state (see UpdateOvershoot / the header notes). LIVE -
    // recomputed from scratch every Update(), never latched.
    bool  overshooting_ = false;
    float overshootMm_  = 0.0f;   // + = fingertip is past the target

    // Per-frame effective world->camera rotation for the active target (see the
    // HasEffectiveWorldR()/GetEffectiveWorldR() accessors).
    bool        hasEffectiveWorldR_ = false;
    cv::Matx33d effectiveWorldR_    = cv::Matx33d::eye();

    // ---- Ring markers (live fingertip) ---------------------------------------
    // base<-second rigid transform (X_base = R*X_second + t). Seeded in the
    // constructor from ring.relationshipAngleDeg (marker planes folded about the
    // base marker's printed top edge); replaced by the live-learned value the
    // first time both markers are seen in one frame, then kept refreshed. The
    // mount is rigid, so the learned value persists across Reset().
    cv::Matx33d ringRelR_ = cv::Matx33d::eye();
    cv::Vec3d   ringRelT_;
    bool        ringRelLearned_ = false;

    // Per-frame ring fingertip (valid only when hasRingFingertip_).
    bool        hasRingFingertip_   = false;
    cv::Point3f ringFingertipCamYup_ = {};
    bool        ringFromSecond_      = false;   // this frame's pose came via marker 74
    bool        ringCoasting_        = false;   // riding the coast window (no marker seen)

    // Ring-fingertip coast: when BOTH ring markers drop out, the last measured
    // fingertip is held for a short window (kCoastFrames, shared with the
    // trained-object coast) instead of invalidating guidance for a one-frame
    // detection hiccup. NEVER falls back to any camera-co-located offset model.
    cv::Point3f lastRingFingertip_ = {};
    bool        hasLastRing_       = false;
    int         ringCoastLeft_     = 0;

    // Arrow-frame error rotation (see GetCamYupToArrowR). arrowTrimM_ is the
    // constant left factor Rz(arrowYawTrimDeg) * M^T (M = arrow->marker axes),
    // precomputed in the constructor; per frame the full rotation is
    // arrowTrimM_ * Rring^T * diag(1,-1,1). Held through the coast window
    // alongside the fingertip position.
    cv::Matx33d arrowTrimM_     = cv::Matx33d::eye();
    cv::Matx33f camYupToArrowR_ = cv::Matx33f::eye();
    cv::Matx33f lastArrowR_     = cv::Matx33f::eye();

    std::vector<ObjectOverlay>              overlays_;
    std::vector<std::array<cv::Point2f, 4>> worldOutlines_;
    RingOverlay                             ringOverlay_;
    std::vector<std::array<cv::Point2f, 4>> knownLayoutOutlines_;
};
