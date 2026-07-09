#pragma once

// =============================================================================
// FittsTaskHandler.h - Fitts pointing task: virtual fingertip + touch error log
//
// Computes the live "virtual fingertip" cursor (camera principal point plus
// the calibrated camera-to-fingertip offset, scaled by the detected ArUco
// plane depth) and, on each touchscreen contact while a Fitts target is
// active, records:
//   - The touch point in touchscreen-local pixels (red dot)
//   - Pixel/mm error between the camera principal point and the target marker
//   - Pixel/mm error between the virtual fingertip and the target marker
//
// The recorded sample stays valid (and is shown) until the next random
// target is selected via OnNewTarget().
// =============================================================================

#include <array>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "ArucoHandler.h"        // DetectedMarker
#include "Config.h"
#include "FittsBoardLayout.h"    // FittsBoardLayout
#include "TouchHandler.h"        // TouchState


class FittsTaskHandler {
public:
    FittsTaskHandler(const FittsBoardConfig&   boardCfg,
                     const TouchscreenConfig&  touchCfg,
                     const CameraConfig&       camCfg);

    /** @brief Call when entering the FITTS state - clears all task state. */
    void Reset();

    /** @brief Call when a new random target is selected ('r'). Clears the touch sample. */
    void OnNewTarget(int targetId);

    /**
     * @brief Compute the shared per-frame board fits ONCE for this camera frame:
     *        the board-plane homography (used by EstimateTargetFromBoard) and
     *        the board solvePnP pose + circular-mean roll (used by Update and
     *        GetTargetFullPose).
     *
     *        Call exactly once per NEW camera frame, before any of
     *        EstimateTargetFromBoard() / Update() / GetTargetFullPose(). Those
     *        methods only read the cache - they no longer re-run solvePnP /
     *        findHomography themselves. (Previously the logging path re-solved
     *        both up to twice more per frame on the main thread, which is what
     *        caused the >25 ms main-loop stalls and dropped trial-log frames.)
     */
    void PrepareFrame(const std::vector<DetectedMarker>& markers);

    /**
     * @brief Process one loop iteration while in FITTS state. Uses the pose
     *        cached by PrepareFrame() - call that first on each new frame.
     * @param touch        Latest touch state from TouchHandler
     * @param cal3Complete Whether Cal3 has finished (virtual fingertip available)
     * @param cal3Offset   Calibrated camera-to-fingertip offset [mm]
     * @param cal3RollRef  Calibration roll reference [rad]
     */
    void Update(const TouchState&                  touch,
                bool                               cal3Complete,
                cv::Point3f                        cal3Offset,
                float                              cal3RollRef);

    // ---- Virtual fingertip cursor (for DisplayHandler) -----------------------
    bool        HasVirtualFingertip() const { return ftValid_; }
    cv::Point2i GetVirtualFingertipPx() const { return ftPx_; }

    /**
     * @brief Estimate the active target's camera-relative position + roll from
     *        the board-plane homography cached by PrepareFrame(), for guidance
     *        when the target's own marker is not directly detected - far away
     *        (coarse markers carry it) or lost up close (neighbouring fine
     *        markers carry it). The target's location on the board is known
     *        from the layout, so its position is recoverable from any fit.
     * @param targetId  The active target marker ID
     * @param posOut    Camera-relative target position [mm], Y-up (matches
     *                  DetectedMarker::positionMm)
     * @param rollOut   Camera roll [rad] (circular mean of visible board markers)
     * @param cornersPxOut  Optional: the target marker's four corners projected
     *                  into the camera image (for drawing the estimated outline)
     * @return false if the target is unknown or no homography was fit this frame
     */
    bool EstimateTargetFromBoard(int targetId,
                                 cv::Point3f& posOut, float& rollOut,
                                 std::array<cv::Point2f, 4>* cornersPxOut = nullptr) const;

    /**
     * @brief Per-frame trial-logging sample for the target marker.
     *
     * Position (posMmOut) comes from the ambiguity-free homography + apparent-
     * scale estimator (EstimateTargetFromBoard), camera frame Y-UP, so the
     * logged trajectory stays clean at long range instead of flipping between
     * the two planar-pose solutions the way solvePnP depth does.
     *
     * dispMmOut is the fingertip-compensated displacement Δp = target - fingertip
     * (camera frame Y-up), computed with the FULL 3D Cal3 offset (incl. its Z
     * standoff), so dz -> 0 as the fingertip reaches the target plane - unlike
     * the controller's planar-only displacement.
     *
     * quatXyzwOut is the board->camera rotation (OpenCV Y-DOWN) from solvePnP;
     * nice-to-have orientation only, may be noisy far away.
     *
     * @param cal3Complete  Whether Cal3 finished (compensation available)
     * @param cal3Offset    Calibrated camera->fingertip offset [mm] (Cal3 frame)
     * @param cal3RollRef   Cal3 roll reference [rad]
     * @param posMmOut      Target marker centre, camera frame Y-up [mm]
     * @param quatXyzwOut   Quaternion (x,y,z,w) of board->camera rotation
     * @param dispMmOut     Δp = target - fingertip, camera frame Y-up [mm]
     * @param detectedOut   True if the target marker itself was detected this frame
     * @return false if the target is unknown or no stable estimate was available
     */
    bool GetTargetFullPose(const std::vector<DetectedMarker>& markers, int targetId,
                           bool cal3Complete, cv::Point3f cal3Offset, float cal3RollRef,
                           cv::Point3f& posMmOut, cv::Vec4f& quatXyzwOut,
                           cv::Point3f& dispMmOut, bool& detectedOut) const;
    // (markers is still taken here only to set detectedOut - the position and
    //  orientation come from the PrepareFrame() cache.)

    // ---- Touch sample (for ArucoHandler touchscreen overlay) ------------------
    bool               HasTouchSample() const { return sampleValid_; }
    cv::Point2i        GetTouchScreenPx() const { return touchScreenPx_; }
    const std::string& GetErrorLine1() const { return errorLine1_; }
    const std::string& GetErrorLine2() const { return errorLine2_; }

    // ---- Virtual fingertip position at the moment of touch (for DisplayHandler) ----
    bool        HasTouchFingertip() const { return sampleValid_ && touchFtValid_; }
    cv::Point2i GetTouchFingertipPx() const { return touchFtPx_; }

private:
    /** @brief Virtual fingertip pixel position: principal point + roll-corrected offset / depth. */
    cv::Point2i VirtualFingertipPx(const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                                   cv::Point3f d, float rollRefRad) const;

    const TouchscreenConfig&  touchCfg_;
    const CameraConfig&       camCfg_;
    FittsBoardLayout          layout_;

    // ---- Per-frame board-fit cache (set by PrepareFrame, read by the rest) ---
    // frameH_: board(mm) -> image(px) homography over the RANSAC inliers of all
    // visible board markers. framePose*: solvePnP ITERATIVE camera pose over the
    // same correspondences (used for the fingertip cursor + logged quaternion,
    // never for guidance depth - see EstimateTargetFromBoard). frameRoll_:
    // circular-mean image roll of the visible board markers.
    cv::Mat   frameH_;
    bool      frameHValid_    = false;
    cv::Vec3d frameRvec_      = {};
    cv::Vec3d frameTvec_      = {};
    bool      framePoseValid_ = false;
    float     frameRoll_      = 0.0f;
    bool      frameRollValid_ = false;

    int  targetId_   = 0;
    bool wasTouched_ = false;

    bool        ftValid_ = false;
    cv::Point2i ftPx_    = {};

    bool        sampleValid_   = false;
    cv::Point2i touchScreenPx_ = {};
    std::string errorLine1_;
    std::string errorLine2_;

    bool        touchFtValid_ = false;
    cv::Point2i touchFtPx_    = {};
};
