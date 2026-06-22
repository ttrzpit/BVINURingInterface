#pragma once

// =============================================================================
// Cal2Handler.h - Calibration Stage 2: Finger Deflection Stiffness
//
// For each of the CONSTANT_CALIBRATION_ANGLES_COUNT calibration headings
// (CONSTANT_CALIBRATION_ANGLES_DEG), commands an open-loop force ramp via
// ControllerHandler's calibration force mode: ramp up at cfg_.forceRampRate
// until the AROM boundary (Cal1Handler, AromBoundary::RadiusAtAngle()) is
// reached along that heading, then hold at peak for cfg_.holdSecs. During
// ramp-up, the measured force (from amplifier current) and virtual fingertip
// deflection are sampled each cycle, both projected onto the heading; a
// least-squares fit of force vs. deflection gives K(theta) = dF/dx [N/mm]
// (nuring_calibration_implementation_guide.md, item 6 / Phase 3, items 1/3/5).
//
// After the hold, the commanded force is released immediately (no ramp-down,
// since system friction prevents the finger from returning to center during
// a slow ramp-down); the handler then waits cfg_.releaseWaitSecs before
// starting the next heading, giving the finger time to relax back to center.
//
// Camera-based ground truth (phi_camera, C(theta) mapping correction) is
// deferred to Phase 2 - see items 2/4/6.
//
// State machine per heading:
//   RAMP_UP -> HOLD -> WAIT (released, no ramp-down) -> (next heading) -> ... -> DONE
//
// Requires Cal1 (AROM) to have completed first; if AromBoundary::valid is
// false on Reset(), enters BLOCKED with an explanatory status.
// =============================================================================

#include <array>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "Cal1Handler.h"  // AromBoundary
#include "Config.h"
#include "ControllerHandler.h"
#include "Globals.h"


// ---- Per-heading calibration result -------------------------------------------

struct Cal2HeadingResult {
    float theta         = 0.0f;   ///< Calibration heading [rad]
    float stiffness_kP  = 0.0f;   ///< K(theta) [N/mm], from ramp-up linear fit
    int   rampUpSamples = 0;      ///< Number of samples used in the fit
    bool  valid         = false;  ///< True if the fit succeeded
};


class Cal2Handler {
public:
    Cal2Handler(const ControllerHandler& controller, const AromBoundary& aromBoundary,
                const Cal2Config& cfg);

    /** @brief Reset to start a new calibration run on the next Update() call.
     *         Enters BLOCKED if the AROM boundary (Cal1) is not yet valid. */
    void Reset();

    /**
     * @brief Process one loop iteration while in CAL_STI state. Advances the
     *        per-heading force ramp state machine; the commanded force is
     *        available via GetCommandedForce() for the caller to forward to
     *        ControllerHandler::SetCalibrationForce().
     * @param nowSecs  Current time in seconds (cv::getTickCount / frequency)
     */
    void Update(double nowSecs);

    bool IsComplete() const { return phase_ == Phase::DONE; }

    /** @brief One-line status string suitable for the telemetry Output row. */
    const std::string& GetStatus() const { return status_; }

    /** @brief Open-loop force command [N] for the current cycle. */
    cv::Point2f GetCommandedForce() const { return commandedForce_; }

    /** @brief K(theta) values aligned with CONSTANT_CALIBRATION_ANGLES_DEG,
     *         meaningful once IsComplete() is true. */
    std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT> GetStiffnessProfile() const;

    const std::array<Cal2HeadingResult, CONSTANT_CALIBRATION_ANGLES_COUNT>& GetResults() const { return results_; }

    /** @brief Index into CONSTANT_CALIBRATION_ANGLES_DEG for the heading
     *         currently being measured, or -1 before the first Update(). */
    int GetCurrentHeadingIndex() const { return headingIdx_; }

private:
    enum class Phase { RAMP_UP, HOLD, WAIT, DONE, BLOCKED };

    void StartHeading(int index, double nowSecs);
    void FitStiffness(int index);

    const ControllerHandler& controller_;
    const AromBoundary&      aromBoundary_;
    Cal2Config               cfg_;

    Phase  phase_          = Phase::RAMP_UP;
    int    headingIdx_      = -1;
    double phaseStartSecs_  = 0.0;
    float  peakForceMag_    = 0.0f;

    cv::Point2f commandedForce_ = {};

    // Ramp-up samples for the current heading: (deflection [mm], force [N]),
    // both projected onto the heading direction.
    std::vector<cv::Point2f> rampSamples_;

    std::array<Cal2HeadingResult, CONSTANT_CALIBRATION_ANGLES_COUNT> results_ = {};

    std::string status_ = "Stiffness: waiting to start.";
};
