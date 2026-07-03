#pragma once

// =============================================================================
// RigAlignmentHandler.h - one-time screen<->world-board orientation capture
//
// Measures the fixed rotation R_screen->world between the touchscreen/Cal3 frame
// and the OBJECTS world-board frame, so OBJECTS guidance can rotate the Cal3
// fingertip offset (measured in the screen frame) by the live world->camera pose
// each frame - the automatic, full-pose replacement for the scalar
// object_world.roll_offset_deg trim.
//
// WHY A SEPARATE CAPTURE: the Cal3 fingertip offset is anchored to the
// touchscreen, but at OBJECTS runtime nothing ever sees the touchscreen - only
// the world board. The screen<->world relationship is therefore unmeasurable at
// runtime; it must be captured once, when the camera can see BOTH references in
// the same frame(s). It is a property of the RIG (where the board sits relative
// to the screen), not of the participant, so it is captured once per setup and
// reused for every session.
//
// SELF-CONTAINED / MAIN THREAD: unlike the threaded ArucoHandler (single active
// dictionary), this runs its OWN two detectors - DICT_4X4_1000 for the screen
// calibration grid and DICT_6X6_100 for the world board - directly on the raw
// grayscale frame on the main thread, so both dictionaries are decoded in the
// same frame. It is a low-rate, one-time operation, so the extra per-frame
// detection cost is irrelevant.
//
// Frames (matching the rest of the code):
//   * Screen pose  = Cal3's screen->camera solvePnP over the calibration grid
//     (screen frame: X right, Y down, Z toward camera; layout mirrors
//     Cal3Handler::ComputeCameraPoseInScreen exactly).
//   * World pose   = WorldObjectHandler's world->camera solvePnPRansac over the
//     detected world-board marker centres <-> config marker_positions.
//   R_screen->world = R_world->cam^T * R_screen->cam, averaged (rotation mean +
//   SVD re-orthonormalisation) over kTargetSamples good frames.
// =============================================================================

#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "Config.h"


class RigAlignmentHandler {
public:
    RigAlignmentHandler(const ArucoCalibrationGridConfig& calGridCfg,
                        const TouchscreenConfig&          touchCfg,
                        const ObjectWorldConfig&          objCfg,
                        const CameraConfig&               camCfg);

    /** @brief Clear all accumulated samples - call when entering the capture. */
    void Reset();

    /** @brief Process one raw grayscale frame: detect both boards, solve both
     *         poses, and (if BOTH solve) accumulate one R_screen->world sample.
     *         Completes automatically after kTargetSamples good samples. */
    void Update(const cv::Mat& grayFrame);

    /** @brief True once enough good samples have been averaged. */
    bool IsComplete() const { return complete_; }

    /** @brief Operator-facing progress / visibility status line. */
    std::string GetStatus() const;

    /** @brief Averaged, orthonormalised screen->world rotation (valid once
     *         IsComplete()). Identity before completion. */
    cv::Matx33d GetScreenToWorldR() const { return finalR_; }

    int SampleCount() const { return count_; }

    /** @brief Persist the captured rotation to a sidecar YAML (rig_alignment.yaml)
     *         under key "rig_screen_to_world". No-op / false if not complete. */
    bool Save(const std::string& path) const;

private:
    // Screen->camera pose from the calibration-grid markers (mirrors
    // Cal3Handler::ComputeCameraPoseInScreen). false if < 2 grid markers.
    bool SolveScreenPose(const std::vector<int>&                     ids,
                         const std::vector<std::vector<cv::Point2f>>& corners,
                         cv::Matx33d&                                Rout) const;

    // World->camera pose from the world-board markers (mirrors WorldObjectHandler
    // world PnP). false if < 4 world markers.
    bool SolveWorldPose(const std::vector<int>&                     ids,
                        const std::vector<std::vector<cv::Point2f>>& corners,
                        cv::Matx33d&                                Rout) const;

    // ---- Config -------------------------------------------------------------
    const ArucoCalibrationGridConfig& calGridCfg_;
    const TouchscreenConfig&          touchCfg_;
    const ObjectWorldConfig&          objCfg_;
    const CameraConfig&               camCfg_;

    // ---- Detectors (own, main-thread) ---------------------------------------
    cv::aruco::Dictionary    calDictionary_;   // DICT_4X4_1000 (screen cal grid)
    cv::aruco::Dictionary    objDictionary_;   // DICT_6X6_100  (world board)
    cv::aruco::ArucoDetector calDetector_;
    cv::aruco::ArucoDetector objDetector_;

    // ---- Accumulation -------------------------------------------------------
    static constexpr int kTargetSamples = 30;
    cv::Matx33d sumR_    = cv::Matx33d::zeros();   // running sum of R_screen->world
    int         count_   = 0;
    bool        complete_ = false;
    cv::Matx33d finalR_  = cv::Matx33d::eye();

    // Last-frame visibility (for the status line). The marker counts are written
    // from the const solve helpers, hence mutable.
    bool         lastScreenOk_ = false;
    bool         lastWorldOk_  = false;
    mutable int  lastScreenMarkers_ = 0;
    mutable int  lastWorldMarkers_  = 0;
};
