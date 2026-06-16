#include "PretensionHandler.h"

#include <iomanip>
#include <sstream>

// =============================================================================
// PretensionHandler.cpp
//
// See PretensionHandler.h for the 3-step phase overview.
// =============================================================================

PretensionHandler::PretensionHandler(ControllerHandler& controller)
    : controller_(controller)
{}

void PretensionHandler::Reset() {
    phase_               = Phase::UNSPOOL;
    tensionPhaseEntered_ = false;
    controller_.SetOutputEnabled(false);
    controller_.SetManualTensionMode(false);
    status_ = "Tension 1/3: Unspool all tendons fully, then press Enter.";
}

void PretensionHandler::Advance(const TeensyToPcPacket& rx) {
    switch (phase_) {
        case Phase::UNSPOOL:
            controller_.SetOutputEnabled(true);
            controller_.SetManualTensionMode(true);
            phase_               = Phase::TENSION;
            tensionPhaseEntered_ = true;
            status_.clear();  // step 2/3 status is computed dynamically by GetStatus()
            break;

        case Phase::TENSION:
            // Capture the operator-set tensions as the new T_preload held by
            // Stage 2 (ComputeTensionOutputs) at zero deflection force.
            controller_.SetPreloadTensions();
            controller_.SetHomePosition(rx);
            // Leave output enabled - exiting manual tension mode lets the
            // normal solve pipeline settle to the captured preload tensions,
            // and the RobotState ladder takes over (READY) once PRETENSION exits.
            controller_.SetManualTensionMode(false);
            phase_  = Phase::DONE;
            status_ = "Tension 3/3: Home position recorded. ";
            break;

        case Phase::DONE:
            break;
    }
}

std::string PretensionHandler::GetStatus() const {
    if (phase_ != Phase::TENSION) return status_;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Tension 2/3: Select motor [a,b,c,d], [+/-], [n.n]; press Enter to save.";
    return ss.str();
}

std::string PretensionHandler::GetTensionAdjustStatus() const {
    cv::Point3f t = controller_.GetTensions();
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Tension adjust: [a/b/c/d] select motor, +/- = +/-0.1N, n.n+Enter = set value. ";
    return ss.str();
}
