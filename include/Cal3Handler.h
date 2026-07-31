#pragma once

// =============================================================================
// Cal3Handler.h - Calibration Stage 3: Camera-to-Fingertip Offset
//
// The participant presses their fingertip against the touchscreen 10 times at
// random positions. At each stable contact (~200 ms hold), the system records:
//   - Fingertip touch position from the touchscreen [mm]
//   - Camera position in screen space via multi-tag solvePnP
//
// The vector from camera to fingertip is stored as offset_cam_to_fingertip (d).
// After 10 samples the offsets are averaged to give the final calibrated value.
//
// State machine per sample:
//   WAITING → (touch begins) → HOLDING → (200 ms elapsed) → record → COOLDOWN
//   COOLDOWN → (2 s elapsed) → WAITING
//   After 10th sample: DONE
// =============================================================================

#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "ArucoHandler.h"   // DetectedMarker
#include "Config.h"
#include "Globals.h"
#include "TouchHandler.h"   // TouchState


class Cal3Handler {
public:
    Cal3Handler(const TouchscreenConfig&          touchCfg,
                const CameraConfig&               camCfg,
                const ArucoCalibrationGridConfig& calGridCfg,
                const Cal3Config&                 cal3Cfg);

    /** @brief Reset all samples and state - call when entering CAL3. */
    void Reset();

    /**
     * @brief Process one loop iteration while in CAL3 state.
     * @param touch      Latest touch state from TouchHandler
     * @param markers    Latest detections from ArucoHandler (for multi-tag pose)
     * @param nowSecs    Current time in seconds (cv::getTickCount / frequency)
     * @return true if a new sample was just recorded this call
     */
    bool Update(const TouchState&                  touch,
                const std::vector<DetectedMarker>& markers,
                double                             nowSecs);

    // ---- Results ------------------------------------------------------------
    bool        IsComplete()      const { return sampleCount_ >= maxSamples_; }
    int         GetSampleCount()  const { return sampleCount_; }
    cv::Point3f GetLastOffset()   const { return lastOffset_; }
    cv::Point3f GetFinalOffset()  const { return finalOffset_; }
    float       GetRollRef()      const { return rollReference_; }

    /** @brief Inject a camera->fingertip offset + roll reference loaded from a
     *         participant config file and mark the stage complete, bypassing the
     *         touch-collection state machine. */
    void LoadOffset( cv::Point3f offset, float rollRef ) {
        finalOffset_   = offset;
        lastOffset_    = offset;
        rollReference_ = rollRef;
        sampleCount_   = maxSamples_;   // -> IsComplete()
        phase_         = Phase::DONE;
        status_        = "Fingertip offset: loaded from participant config.";
    }

    /** @brief One-line status string suitable for the telemetry Output row. */
    const std::string& GetStatus() const { return status_; }

private:
    /**
     * @brief Pool all detected marker corners into a single solvePnP call.
     *        Computes camera position in screen-world mm coordinates.
     * @param markers   Current frame's detections
     * @param rvecOut   Rotation vector (Rodrigues) for roll extraction
     * @param camPosOut Camera position in screen world [mm] - populated on success
     * @return true if solvePnP succeeded with enough correspondences
     */
    bool ComputeCameraPoseInScreen(const std::vector<DetectedMarker>& markers,
                                    cv::Vec3d&   rvecOut,
                                    cv::Point3f& camPosOut) const;

    const TouchscreenConfig&          touchCfg_;
    const CameraConfig&               camCfg_;
    const ArucoCalibrationGridConfig& calGridCfg_;
    Cal3Config                        cal3Cfg_;

    // ---- State machine ------------------------------------------------------
    enum class Phase { WAITING, HOLDING, COOLDOWN, DONE };
    Phase  phase_           = Phase::WAITING;
    double touchStartSecs_  = 0.0;
    double cooldownEndSecs_ = 0.0;

    double holdSecs_;
    double cooldownSecs_;
    int    maxSamples_;

    // ---- Sample storage -----------------------------------------------------
    int                      sampleCount_  = 0;
    std::vector<cv::Point3f> offsets_;
    std::vector<float>       rollSamples_;
    cv::Point3f              lastOffset_   = {};
    cv::Point3f              finalOffset_  = {};
    float                    rollReference_= 0.0f;

    std::string status_ = "Touch screen (0/10)";
};
