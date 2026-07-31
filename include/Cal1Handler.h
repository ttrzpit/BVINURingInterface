#pragma once

// =============================================================================
// Cal1Handler.h - Calibration Stage 1: Finger Active Range of Motion (AROM)
//
// While the operator holds preload tension on all three tendons (no active
// PID target), the participant traces circles at the edge of comfortable
// reach for cfg_.recordSecs. The recorded virtual fingertip positions
// (ControllerHandler::GetVirtualPosition()) are converted to polar
// coordinates, binned into kAromBoundaryPoints angular sectors, and the
// 95th-percentile radius per sector is fit with a periodic cubic spline to
// define the AROM boundary (nuring_calibration_implementation_guide.md, item 5).
//
// State machine:
//   RECORDING → (recordSecs elapsed) → ComputeBoundary() → DONE
// =============================================================================

#include <array>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "Config.h"
#include "ControllerHandler.h"


constexpr int kAromBoundaryPoints = CONSTANT_CALIBRATION_ANGLES_COUNT;

// ---- AROM boundary -----------------------------------------------------------
// kAromBoundaryPoints (angle, radius) control points plus periodic cubic
// spline coefficients, sorted by theta ascending. theta[] covers [0, 2*PI).

struct AromBoundary {
    bool valid = false;

    std::array<float, kAromBoundaryPoints> theta  = {};  ///< Bin center angles [rad], ascending
    std::array<float, kAromBoundaryPoints> radius = {};  ///< 95th-percentile radius per bin [mm]
    std::array<float, kAromBoundaryPoints> accel  = {};  ///< Periodic cubic spline 2nd derivatives

    /** @brief Boundary radius [mm] at heading thetaRad, via periodic cubic spline. */
    float RadiusAtAngle(float thetaRad) const;
};


class Cal1Handler {
public:
    Cal1Handler(const ControllerHandler& controller, const Cal1Config& cfg);

    /** @brief Reset to start a new recording on the next Update() call. */
    void Reset();

    /**
     * @brief Process one loop iteration while in CAL_ROM state. Records the
     *        current virtual position; once cfg_.recordSecs has elapsed,
     *        computes the AROM boundary and transitions to DONE.
     * @param nowSecs  Current time in seconds (cv::getTickCount / frequency)
     */
    void Update(double nowSecs);

    bool IsComplete() const { return phase_ == Phase::DONE; }

    /** @brief One-line status string suitable for the telemetry Output row. */
    const std::string& GetStatus() const { return status_; }

    const AromBoundary& GetBoundary() const { return boundary_; }

    /** @brief Inject an AROM boundary loaded from a participant config file and
     *         mark the stage complete, bypassing the recording phase. */
    void LoadBoundary( const AromBoundary& b ) {
        boundary_       = b;
        boundary_.valid = true;
        phase_          = Phase::DONE;
        status_         = "AROM: loaded from participant config.";
    }

    /** @brief Recorded virtual fingertip positions [mm] so far this recording. */
    const std::vector<cv::Point2f>& GetSamples() const { return samples_; }

private:
    enum class Phase { RECORDING, DONE };

    /** @brief Bin recorded samples by heading, take 95th-pct radius per bin,
     *         and fit the periodic cubic spline boundary. */
    void ComputeBoundary();

    const ControllerHandler& controller_;
    Cal1Config cfg_;

    Phase  phase_        = Phase::RECORDING;
    bool   timerStarted_ = false;
    double startSecs_    = 0.0;

    std::vector<cv::Point2f> samples_;  ///< Recorded virtual positions [mm]

    AromBoundary boundary_;
    std::string  status_ = "AROM: Trace circles with your finger.";
};
