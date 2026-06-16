#pragma once

// =============================================================================
// PretensionHandler.h - Guided pretensioning state machine
//
// Walks the operator through the 3-step pretensioning procedure from
// nuring_calibration_implementation_guide.md ("Pretensioning & Encoder
// Zeroing"). Motor encoders are zeroed once, by the Teensy at boot
// (T_AmplifierClass::Begin() -> ZeroEncoders()) - NOT here. Re-zeroing at
// runtime would shift the amplifier's internal cogging-compensation table
// out of alignment with the motor's commutation reference (cogging returns)
// and would also invalidate q_abs = 0 <-> bare-pulley used by
// ControllerHandler's spool-radius model.
//
//   1. UNSPOOL  - operator manually unspools all tendons, then presses Enter.
//                 ControllerHandler output is enabled and put into manual
//                 tension mode (seeded to tension_preload_min on all three motors).
//   2. TENSION  - Operator uses the Tension interface ([a]/[b]/[c]/[d] select
//                 motor, +/- nudge by 0.1 N, or n.n + Enter for an absolute
//                 value) to set each tendon's preload, watching the live
//                 tension/PWM values in the controller panel. A "bare" Enter
//                 (nothing typed) advances to step 3/3.
//   3. DONE     - ControllerHandler::SetHomePosition() records q_home_A/B/C
//                 from the current encoder counts; manual tension mode and
//                 output are disabled again.
//
// Driven from main.cpp:
//   - Reset()  on entering SystemState::PRETENSION
//   - Advance() when kb.pendingPretensionAdvance is set (Enter key)
//   - GetStatus() feeds keyboard.SetExternalStatus()
//
// Pressing 'T' opens a menu (InputState::TEN_MENU): 'p' starts the guided
// sequence above; [a]/[b]/[c]/[d] instead jump straight into standalone
// tension adjustment (SystemState::TENSION_ADJUST) - main.cpp enables manual
// tension mode directly, skipping the unspool/home-recording steps.
// GetTensionAdjustStatus() feeds keyboard.SetExternalStatus() in that mode.
// =============================================================================

#include <string>

#include "ControllerHandler.h"
#include "PacketTypes.h"  // TeensyToPcPacket


class PretensionHandler {
public:
    explicit PretensionHandler(ControllerHandler& controller);

    /** @brief Reset to step 1/3. Call when entering SystemState::PRETENSION. */
    void Reset();

    /** @brief Advance to the next step. Call when Enter is pressed. */
    void Advance(const TeensyToPcPacket& rx);

    /** @brief True once home position has been recorded (step 3/3 complete). */
    bool IsComplete() const { return phase_ == Phase::DONE; }

    /**
     * @brief One-shot: true exactly once, the first call after the UNSPOOL->TENSION
     *        transition (step 2/3 begins). Clears the flag on read - main.cpp
     *        uses this to default the input state to TEN_SEL_ALL so +/- adjusts
     *        all three motors immediately, without requiring [a/b/c/d] first.
     */
    bool ConsumeTensionPhaseEntered() {
        bool fired = tensionPhaseEntered_;
        tensionPhaseEntered_ = false;
        return fired;
    }

    /**
     * @brief One-line status string for the [DISPLAY_TEXT] box. During step
     *        2/3 this is computed live from ControllerHandler::GetTensions().
     */
    std::string GetStatus() const;

    /**
     * @brief One-line status string for standalone tension adjustment
     *        (SystemState::TENSION_ADJUST), computed live from
     *        ControllerHandler::GetTensions(). Independent of phase_.
     */
    std::string GetTensionAdjustStatus() const;

private:
    enum class Phase { UNSPOOL, TENSION, DONE };

    ControllerHandler& controller_;

    Phase  phase_               = Phase::UNSPOOL;
    bool   tensionPhaseEntered_ = false;

    std::string status_;
};
