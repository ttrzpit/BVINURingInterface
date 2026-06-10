#pragma once

// =============================================================================
// FittsTaskHandler.h — Fitts pointing task: virtual fingertip + touch error log
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

#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "ArucoHandler.h"   // DetectedMarker
#include "Config.h"
#include "TouchHandler.h"   // TouchState


class FittsTaskHandler {
public:
    FittsTaskHandler(const ArucoDisplayConfig& displayCfg,
                     const TouchscreenConfig&  touchCfg,
                     const CameraConfig&       camCfg);

    /** @brief Call when entering the FITTS state — clears all task state. */
    void Reset();

    /** @brief Call when a new random target is selected ('r'). Clears the touch sample. */
    void OnNewTarget(int targetId);

    /**
     * @brief Process one loop iteration while in FITTS state.
     * @param markers      Latest detections from ArucoHandler
     * @param touch        Latest touch state from TouchHandler
     * @param cal3Complete Whether Cal3 has finished (virtual fingertip available)
     * @param cal3Offset   Calibrated camera-to-fingertip offset [mm]
     * @param cal3RollRef  Calibration roll reference [rad]
     */
    void Update(const std::vector<DetectedMarker>& markers,
                const TouchState&                  touch,
                bool                               cal3Complete,
                cv::Point3f                        cal3Offset,
                float                              cal3RollRef);

    // ---- Virtual fingertip cursor (for DisplayHandler) -----------------------
    bool        HasVirtualFingertip() const { return ftValid_; }
    cv::Point2i GetVirtualFingertipPx() const { return ftPx_; }

    // ---- Touch sample (for ArucoHandler touchscreen overlay) ------------------
    bool               HasTouchSample() const { return sampleValid_; }
    cv::Point2i        GetTouchScreenPx() const { return touchScreenPx_; }
    const std::string& GetErrorLine1() const { return errorLine1_; }
    const std::string& GetErrorLine2() const { return errorLine2_; }

private:
    /** @brief Multi-tag solvePnP from the Fitts grid markers — current camera pose. */
    bool ComputeArucoPose(const std::vector<DetectedMarker>& markers,
                          cv::Vec3d& rvecOut, cv::Vec3d& tvecOut) const;

    /** @brief World-space centre of the current target marker, projected to image px. */
    cv::Point2f TargetCenterPx(const cv::Vec3d& rvec, const cv::Vec3d& tvec) const;

    /** @brief Virtual fingertip pixel position: principal point + roll-corrected offset / depth. */
    cv::Point2i VirtualFingertipPx(const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                                   cv::Point3f d, float rollRefRad) const;

    const ArucoDisplayConfig& displayCfg_;
    const TouchscreenConfig&  touchCfg_;
    const CameraConfig&       camCfg_;

    int  targetId_   = 0;
    bool wasTouched_ = false;

    bool        ftValid_ = false;
    cv::Point2i ftPx_    = {};

    bool        sampleValid_   = false;
    cv::Point2i touchScreenPx_ = {};
    std::string errorLine1_;
    std::string errorLine2_;
};
