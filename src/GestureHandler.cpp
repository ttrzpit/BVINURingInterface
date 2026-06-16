#include "GestureHandler.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Globals.h"

// =============================================================================
// GestureHandler.cpp
//
// See GestureHandler.h for the state machine overview.
// =============================================================================

namespace {

float WrapToTwoPi(float theta) {
    while (theta < 0.0f)             theta += CONSTANT_TWO_PI;
    while (theta >= CONSTANT_TWO_PI) theta -= CONSTANT_TWO_PI;
    return theta;
}

// Smallest signed difference (a - b), wrapped to [-PI, PI].
float AngleDiff(float a, float b) {
    float d = WrapToTwoPi(a - b);
    if (d > CV_PI) d -= CONSTANT_TWO_PI;
    return d;
}

}  // namespace


GestureHandler::GestureHandler(const ControllerHandler& controller, const GestureConfig& cfg)
    : controller_(controller), cfg_(cfg)
{}


void GestureHandler::Reset() {
    phase_         = Phase::IDLE;
    armCount_      = 0;
    armSign_       = 0;
    armGateOk_     = false;

    pendingFlickSign_     = 0;
    pendingFlickTimeSecs_ = 0.0;
    lastRawFlickSecs_     = -1e9;

    motionDurationSecs_ = 0.0;
    lastUpdateSecs_     = -1.0;
    ResetCircleAccumulator();

    lastGesture_   = GestureEvent::NONE;
    lastEventSecs_ = -1e9;
}


bool GestureHandler::IsIndicatorActive(double nowSecs) const {
    if (lastGesture_ == GestureEvent::NONE) return false;
    double cooldown = (lastGesture_ == GestureEvent::CONFIRM) ? cfg_.circleCooldownSecs : cfg_.cooldownSecs;
    return (nowSecs - lastEventSecs_) < cooldown;
}


void GestureHandler::UpdateCircleAccumulator(double nowSecs, cv::Point2f pos, float heading) {
    float dHeading = rotationHistory_.empty() ? 0.0f : AngleDiff(heading, prevHeading_);
    prevHeading_ = heading;

    rotationHistory_.push_back({nowSecs, dHeading, pos});
    cumulativeRotationRad_ += dHeading;
    sumPosX_ += pos.x;
    sumPosY_ += pos.y;

    while (!rotationHistory_.empty() &&
           nowSecs - rotationHistory_.front().t > cfg_.circleMaxWindowSecs) {
        const RotationSample& old = rotationHistory_.front();
        cumulativeRotationRad_ -= old.dHeading;
        sumPosX_ -= old.pos.x;
        sumPosY_ -= old.pos.y;
        rotationHistory_.pop_front();
    }
}


void GestureHandler::ResetCircleAccumulator() {
    rotationHistory_.clear();
    cumulativeRotationRad_ = 0.0f;
    prevHeading_           = 0.0f;
    sumPosX_               = 0.0f;
    sumPosY_               = 0.0f;
}


bool GestureHandler::CheckCircleConfirm() const {
    if (std::abs(cumulativeRotationRad_) < cfg_.circleConfirmRad) return false;
    if (rotationHistory_.empty()) return false;

    cv::Point2f centroid(sumPosX_ / static_cast<float>(rotationHistory_.size()),
                          sumPosY_ / static_cast<float>(rotationHistory_.size()));

    float minR = std::numeric_limits<float>::max();
    float maxR = 0.0f;
    for (const RotationSample& s : rotationHistory_) {
        float r = static_cast<float>(cv::norm(s.pos - centroid));
        minR = std::min(minR, r);
        maxR = std::max(maxR, r);
    }
    return minR >= cfg_.circleMinRadiusMm && maxR <= cfg_.circleMaxRadiusMm;
}


GestureEvent GestureHandler::Update(double nowSecs) {
    cv::Point2f pos   = controller_.GetVirtualPosition();
    cv::Point2f vel   = controller_.GetVirtualVelocity();
    float       speed = std::hypot(vel.x, vel.y);
    bool        atRest = speed < cfg_.restSpeedThreshMmS;

    GestureEvent result = GestureEvent::NONE;

    // ---- Flick (up/down) state machine --------------------------------------
    // armGateOk_ is latched when a new arm sequence starts, using
    // motionDurationSecs_ as it stood at the end of the previous frame - i.e.
    // "the finger has only just started moving, this isn't a sustained motion
    // like a circle".
    switch (phase_) {

        case Phase::IDLE: {
            if (std::abs(vel.y) >= cfg_.velocityThreshMmS) {
                int sign = (vel.y > 0.0f) ? 1 : -1;
                if (sign == armSign_) {
                    armCount_++;
                } else {
                    armSign_   = sign;
                    armCount_  = 1;
                    armGateOk_ = (motionDurationSecs_ <= cfg_.flickMaxMotionSecs);
                }
                if (armCount_ >= cfg_.armSamples && armGateOk_) {
                    phase_       = Phase::ARMED;
                    armY_        = pos.y;
                    armTimeSecs_ = nowSecs;
                }
            } else {
                armCount_ = 0;
                armSign_  = 0;
            }
            break;
        }

        case Phase::ARMED: {
            float dy = pos.y - armY_;
            bool confirmed = (armSign_ > 0 && dy >= cfg_.minDisplacementMm)
                          || (armSign_ < 0 && dy <= -cfg_.minDisplacementMm);
            if (confirmed) {
                if (pendingFlickSign_ == armSign_ &&
                    (nowSecs - pendingFlickTimeSecs_) <= cfg_.doubleFlickWindowSecs) {
                    // Second same-direction raw flick within the window - register it.
                    result            = (armSign_ > 0) ? GestureEvent::FLICK_UP : GestureEvent::FLICK_DOWN;
                    pendingFlickSign_ = 0;
                } else {
                    // First raw flick of a potential double-flick - wait for the second.
                    pendingFlickSign_     = armSign_;
                    pendingFlickTimeSecs_ = nowSecs;
                }
                lastRawFlickSecs_ = nowSecs;
                phase_    = Phase::COOLDOWN;
                armCount_ = 0;
                armSign_  = 0;
            } else if (nowSecs - armTimeSecs_ > cfg_.maxWindowSecs) {
                phase_    = Phase::IDLE;
                armCount_ = 0;
                armSign_  = 0;
            }
            break;
        }

        case Phase::COOLDOWN: {
            if (nowSecs - lastRawFlickSecs_ >= cfg_.cooldownSecs) {
                phase_ = Phase::IDLE;
            }
            break;
        }
    }

    // ---- Confirm (circle) accumulator ----------------------------------------
    // Dropping below restSpeedThreshMmS resets the accumulator - a real circle
    // is traced at a roughly continuous speed, so a pause means "not a circle".
    if (atRest) {
        ResetCircleAccumulator();
    } else {
        float heading = std::atan2(vel.y, vel.x);
        UpdateCircleAccumulator(nowSecs, pos, heading);
        if (result == GestureEvent::NONE && CheckCircleConfirm()) {
            result = GestureEvent::CONFIRM;
            ResetCircleAccumulator();
            // A confirm overrides any in-progress flick arming/pending-double-flick state.
            phase_            = Phase::IDLE;
            armCount_         = 0;
            armSign_          = 0;
            pendingFlickSign_ = 0;
        }
    }

    // ---- Motion-duration bookkeeping for next frame's flick-arm gate --------
    double dt = (lastUpdateSecs_ >= 0.0) ? (nowSecs - lastUpdateSecs_) : 0.0;
    motionDurationSecs_ = atRest ? 0.0 : (motionDurationSecs_ + dt);
    lastUpdateSecs_     = nowSecs;

    // ---- Record result for the on-screen indicator --------------------------
    if (result != GestureEvent::NONE) {
        lastGesture_   = result;
        lastEventSecs_ = nowSecs;
    }

    return result;
}
