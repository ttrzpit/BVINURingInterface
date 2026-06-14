#include "PretensionHandler.h"

#include <iomanip>
#include <sstream>

// =============================================================================
// PretensionHandler.cpp
//
// See PretensionHandler.h for the 4-step phase overview.
// =============================================================================

PretensionHandler::PretensionHandler(ControllerHandler& controller)
    : controller_(controller)
{}

void PretensionHandler::Reset() {
    phase_           = Phase::UNSPOOL;
    sendZeroCommand_ = false;
    zeroSentAtSecs_  = 0.0;
    tensionPhaseEntered_ = false;
    controller_.SetOutputEnabled(false);
    controller_.SetManualTensionMode(false);
    status_ = "Tension 1/4: Unspool all tendons fully, then press Enter.";
}

void PretensionHandler::Update(double nowSecs) {
    if (phase_ != Phase::ZERO) {
        sendZeroCommand_ = false;
        return;
    }

    sendZeroCommand_ = (nowSecs - zeroSentAtSecs_) < kZeroCommandSecs;

    if (nowSecs - zeroSentAtSecs_ >= kZeroSettleSecs) {
        controller_.SetOutputEnabled(true);
        controller_.SetManualTensionMode(true);
        phase_  = Phase::TENSION;
        tensionPhaseEntered_ = true;
        status_.clear();  // step 3/4 status is computed dynamically by GetStatus()
    }
}

void PretensionHandler::Advance(const TeensyToPcPacket& rx, double nowSecs) {
    switch (phase_) {
        case Phase::UNSPOOL:
            phase_          = Phase::ZERO;
            zeroSentAtSecs_ = nowSecs;
            status_ = "Tension 2/4: Zeroing motor encoders...";
            break;

        case Phase::ZERO:
            // Auto-advances via Update(); Enter is ignored here.
            break;

        case Phase::TENSION:
            // Capture the operator-set tensions as the preload held by
            // SolveTensions() at zero force — must happen before
            // SetHomePosition() resets tension_A/B/C to zero.
            controller_.SetPreloadTensions();
            controller_.SetHomePosition(rx);
            // Leave output enabled — exiting manual tension mode lets the
            // normal solve pipeline settle to the captured preload tensions,
            // and the RobotState ladder takes over (READY) once PRETENSION exits.
            controller_.SetManualTensionMode(false);
            phase_  = Phase::DONE;
            status_ = "Tension 4/4: Home position recorded. ";
            break;

        case Phase::DONE:
            break;
    }
}

std::string PretensionHandler::GetStatus() const {
    if (phase_ != Phase::TENSION) return status_;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Tension 3/4: Select motor [a,b,c,d], [+/-], [n.n]; press Enter to save.";
    return ss.str();
}

std::string PretensionHandler::GetTensionAdjustStatus() const {
    cv::Point3f t = controller_.GetTensions();
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Tension adjust: [a/b/c/d] select motor, +/- = +/-0.1N, n.n+Enter = set value. ";
    return ss.str();
}
