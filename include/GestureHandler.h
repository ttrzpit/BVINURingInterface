#pragma once

// =============================================================================
// GestureHandler.h — Flick up/down + confirm-circle gesture detection
//                     (RobotState::READY only)
//
// Detects two families of gestures from the virtual fingertip position
// (ControllerHandler::GetVirtualPosition/GetVirtualVelocity). +Y is "up"
// (matches the on-screen Virtual Fingertip Mapping plot, which flips Y for
// display).
//
// --- Flick (up/down) ---------------------------------------------------------
// A quick scroll-style flick, evaluated every Update() call. Two consecutive
// same-direction "raw" flicks within cfg_.doubleFlickWindowSecs are required
// to register a single FLICK_UP/FLICK_DOWN event — this is the second line of
// defense (after the motion-duration gate below) against a confirm-circle
// gesture being misread as a flick, since a circle's incidental up/down raw
// flicks rarely repeat in the same direction twice in a row.
//
//   IDLE → (|vel.y| over threshold for cfg_.armSamples consecutive samples,
//           AND the finger has been continuously in motion for
//           <= cfg_.flickMaxMotionSecs since it last was at rest) → ARMED
//   ARMED → (net |dy| since arming reaches cfg_.minDisplacementMm, same sign)
//           → "raw flick" recorded, → COOLDOWN
//             - if this is the 2nd same-direction raw flick within
//               cfg_.doubleFlickWindowSecs, fires FLICK_UP/FLICK_DOWN
//             - otherwise becomes the pending 1st flick and waits
//   ARMED → (cfg_.maxWindowSecs elapsed without confirming displacement) → IDLE
//   COOLDOWN → (cfg_.cooldownSecs elapsed since the raw flick) → IDLE
//
// The motion-duration gate is the primary defense against a "draw a circle"
// confirm gesture being misread as a flick: a flick's velocity spike happens
// right as the finger leaves rest, while a circle has been in continuous
// motion for a while before any vertical-velocity spike occurs.
//
// --- Confirm (circle) ---------------------------------------------------------
// While the fingertip speed stays >= cfg_.restSpeedThreshMmS, the signed
// heading (atan2(vel.y, vel.x)) is accumulated into a rolling sum over the
// last cfg_.circleMaxWindowSecs. If |cumulative rotation| reaches
// cfg_.circleConfirmRad (~300 deg) AND the path over that window stays within
// [circleMinRadiusMm, circleMaxRadiusMm] of its own centroid (rejects both
// jitter and large sweeps), CONFIRM fires. Dropping below
// cfg_.restSpeedThreshMmS resets the accumulator.
//
// Call Reset() whenever RobotState transitions away from READY (and on entry
// to READY) so stale velocity history can't trigger a spurious gesture.
// =============================================================================

#include <deque>

#include <opencv2/core.hpp>

#include "Config.h"
#include "ControllerHandler.h"


enum class GestureEvent { NONE, FLICK_UP, FLICK_DOWN, CONFIRM };


class GestureHandler {
public:
    GestureHandler(const ControllerHandler& controller, const GestureConfig& cfg);

    /**
     * @brief Process one loop iteration. Only call while RobotState::READY.
     * @param nowSecs  Current time in seconds (cv::getTickCount / frequency)
     * @return The gesture detected this frame, or GestureEvent::NONE.
     */
    GestureEvent Update(double nowSecs);

    /** @brief Reset the state machine to IDLE and clear the indicator. Call on
     *         entry/exit of RobotState::READY. */
    void Reset();

    /** @brief True while the on-screen indicator for the last gesture should be
     *         shown — cfg_.cooldownSecs for flicks, cfg_.circleCooldownSecs for
     *         a confirm. */
    bool IsIndicatorActive(double nowSecs) const;

    /** @brief The last detected gesture (persists through cooldown for display). */
    GestureEvent GetLastGesture() const { return lastGesture_; }

private:
    enum class Phase { IDLE, ARMED, COOLDOWN };

    // One sample retained for the rolling circle-rotation window.
    struct RotationSample {
        double      t;         // nowSecs at the time of this sample
        float       dHeading;  // signed heading delta from the previous sample [rad]
        cv::Point2f pos;       // pos_virtual_ at this sample [mm]
    };

    void UpdateCircleAccumulator(double nowSecs, cv::Point2f pos, float heading);
    void ResetCircleAccumulator();
    bool CheckCircleConfirm() const;

    const ControllerHandler& controller_;
    GestureConfig cfg_;

    // Flick state machine
    Phase  phase_         = Phase::IDLE;
    int    armCount_      = 0;
    int    armSign_       = 0;     // +1 = upward flick candidate, -1 = downward
    bool   armGateOk_     = false; // Motion-duration gate result, latched when armCount_ starts a new run
    float  armY_          = 0.0f;  // pos_virtual_.y at the moment armed [mm]
    double armTimeSecs_   = 0.0;

    // Double-flick tracking — a registered FLICK_UP/FLICK_DOWN requires two
    // same-direction raw flicks within doubleFlickWindowSecs
    int    pendingFlickSign_     = 0;     // +1/-1 = first raw flick of a pending double-flick, 0 = none
    double pendingFlickTimeSecs_ = 0.0;   // nowSecs when pendingFlickSign_ was set
    double lastRawFlickSecs_     = -1e9;  // nowSecs of the most recent raw flick — drives COOLDOWN

    // Shared "at rest" tracking — gates flick arming, also resets the circle accumulator
    double motionDurationSecs_ = 0.0;  // Continuous time speed has stayed >= restSpeedThreshMmS, as of the end of the previous frame (0 if at rest)
    double lastUpdateSecs_     = -1.0; // nowSecs at the previous Update() call (-1 = first call)

    // Confirm (circle) accumulator — rolling window of heading deltas + positions
    std::deque<RotationSample> rotationHistory_;
    float cumulativeRotationRad_ = 0.0f;
    float prevHeading_           = 0.0f;
    float sumPosX_                = 0.0f;
    float sumPosY_                = 0.0f;

    // Last fired gesture, for the on-screen indicator
    double       lastEventSecs_ = -1e9;
    GestureEvent lastGesture_   = GestureEvent::NONE;
};
