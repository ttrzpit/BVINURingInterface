#pragma once

// =============================================================================
// PretensionHandler.h — Guided pretensioning / encoder-zeroing state machine
//
// Walks the operator through the 4-step pretensioning procedure from
// nuring_calibration_implementation_guide.md ("Pretensioning & Encoder
// Zeroing"):
//
//   1. UNSPOOL  — operator manually unspools all tendons, then presses Enter.
//   2. ZERO     — PC sends PcState::ZERO_ENC for a short window so the Teensy
//                 calls Amplifier.ZeroEncoders() (absolute zero = bare pulley);
//                 auto-advances to TENSION after a short settle delay.
//   3. TENSION  — ControllerHandler output is enabled and put into manual
//                 tension mode (seeded to tension_min on all three motors).
//                 Operator uses the Tension interface ([a]/[b]/[c]/[d] select
//                 motor, +/- nudge by 0.1 N, or n.n + Enter for an absolute
//                 value) to set each tendon's preload, watching the live
//                 tension/PWM values in the controller panel. A "bare" Enter
//                 (nothing typed) advances to step 4/4.
//   4. DONE     — ControllerHandler::SetHomePosition() records q_home_A/B/C;
//                 manual tension mode and output are disabled again.
//
// Driven from main.cpp:
//   - Reset()  on entering SystemState::PRETENSION
//   - Update() every loop while in SystemState::PRETENSION
//   - Advance() when kb.pendingPretensionAdvance is set (Enter key)
//   - ShouldSendZeroCommand() gates lastTxPkt.state = PcState::ZERO_ENC
//   - GetStatus() feeds keyboard.SetExternalStatus()
//
// Pressing 'T' opens a menu (InputState::TEN_MENU): 'p' starts the guided
// sequence above; [a]/[b]/[c]/[d] instead jump straight into standalone
// tension adjustment (SystemState::TENSION_ADJUST) — main.cpp enables manual
// tension mode directly, skipping the unspool/zero/home-recording steps.
// GetTensionAdjustStatus() feeds keyboard.SetExternalStatus() in that mode.
// =============================================================================

#include <string>

#include "ControllerHandler.h"
#include "PacketTypes.h"  // TeensyToPcPacket


class PretensionHandler {
public:
    explicit PretensionHandler(ControllerHandler& controller);

    /** @brief Reset to step 1/4. Call when entering SystemState::PRETENSION. */
    void Reset();

    /**
     * @brief Per-loop update — drives the ZERO -> TENSION auto-transition once
     *        the zero-encoder command has been sent and has had time to settle.
     */
    void Update(double nowSecs);

    /** @brief Advance to the next step. Call when Enter is pressed. */
    void Advance(const TeensyToPcPacket& rx, double nowSecs);

    /** @brief True while main.cpp should set lastTxPkt.state = PcState::ZERO_ENC. */
    bool ShouldSendZeroCommand() const { return sendZeroCommand_; }

    /** @brief True once home position has been recorded (step 4/4 complete). */
    bool IsComplete() const { return phase_ == Phase::DONE; }

    /**
     * @brief One-line status string for the [DISPLAY_TEXT] box. During step
     *        3/4 this is computed live from ControllerHandler::GetTensions().
     */
    std::string GetStatus() const;

    /**
     * @brief One-line status string for standalone tension adjustment
     *        (SystemState::TENSION_ADJUST), computed live from
     *        ControllerHandler::GetTensions(). Independent of phase_.
     */
    std::string GetTensionAdjustStatus() const;

private:
    enum class Phase { UNSPOOL, ZERO, TENSION, DONE };

    ControllerHandler& controller_;

    Phase  phase_           = Phase::UNSPOOL;
    double zeroSentAtSecs_  = 0.0;
    bool   sendZeroCommand_ = false;

    std::string status_;

    static constexpr double kZeroCommandSecs = 0.25;  // duration to assert PcState::ZERO_ENC
    static constexpr double kZeroSettleSecs  = 0.75;  // total wait before TENSION phase begins
};
