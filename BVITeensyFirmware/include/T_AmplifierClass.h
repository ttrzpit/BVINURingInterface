#pragma once

// =============================================================================
// T_AmplifierClass.h — Copley NES-090-10-Z amplifier handler
//
// Key design points vs. the reference code:
//
//   Baud upgrade: each amplifier is initialized at 9600 (Copley power-up
//   default) then upgraded to 115200 during Begin(). Verification is performed
//   after each upgrade; the corresponding status LED (pins 30/31/32) lights up
//   only if the upgrade succeeds.
//
//   Parallel state machines: instead of a String-based sequential queue, each
//   amplifier has its own AmpQueryState running on its own HW serial port.
//   All three query simultaneously so the round-trip time is ~3 ms total
//   rather than ~36 ms sequential.
//
//   ISR isolation: DrivePWMFromISR() contains ONLY analogWrite() calls and is
//   safe to call directly from a 1000 Hz IntervalTimer. PollHWSerial() runs
//   all non-ISR work and must be called from the main loop.
// =============================================================================

#include <Arduino.h>
#include <memory>
#include "T_Config.h"
#include "T_SharedDataManager.h"


// ---- Per-amplifier state machine ---------------------------------------------

struct AmpQueryState {

    enum class Phase : uint8_t {
        IDLE,       ///< Ready to start a new poll cycle
        SENT_POS,   ///< g r0x32\r sent, awaiting position response
        SENT_CUR    ///< g r0x0c\r sent (position received), awaiting current
    };

    Phase   phase    = Phase::IDLE;
    char    rxBuf[16] = {};  ///< Fixed receive buffer — zero heap allocation
    uint8_t rxIdx    = 0;

    // Latest parsed values — updated when a full poll cycle completes
    int32_t encoderCount = 0;  ///< Motor position in encoder counts
    int16_t currentRaw   = 0;  ///< Measured current in 0.01 A units
};


// ---- Amplifier handler -------------------------------------------------------

class T_AmplifierClass {
public:
    explicit T_AmplifierClass(SharedDataManager& dataHandle);

    // ---- Lifecycle ----------------------------------------------------------

    /**
     * @brief Full initialization sequence:
     *   1. Configure pins, set PWM resolution
     *   2. Reset all amplifiers (enable: HIGH→LOW→HIGH, 500ms)
     *   3. Start HW serial at 9600 (Copley power-up default)
     *   4. Per-amplifier: upgrade 9600→115200; LED on if verification passes
     *   5. Per-amplifier: set PWM current mode (s r0x24 3\r)
     *   6. Enable amplifiers
     *   7. Zero encoder positions (s r0x32 0\r)
     */
    void Begin();

    // ---- 1000 Hz ISR interface -----------------------------------------------

    /**
     * @brief Write commanded PWM to all three motor amplifiers.
     *        Contains ONLY analogWrite() — safe to call from an IntervalTimer ISR.
     *        Reads commandedPwm_* from shared data written by T_SerialClass.
     */
    void DrivePWMFromISR();

    // ---- 500 Hz main-loop interface ------------------------------------------

    /**
     * @brief Run the parallel state machines for all three HW serial ports.
     *        If all three are IDLE, starts a new poll cycle (sends g r0x32\r
     *        to all three simultaneously). Drains available bytes on each port
     *        and chains the current query (g r0x0c\r) as soon as position
     *        arrives. When all three complete, pushes results to shared data.
     *        Call from main loop when the poll flag is set.
     */
    void PollHWSerial();

    // ---- Control interface (called by T_SerialClass) -------------------------

    void Enable();
    void Disable();
    void ZeroEncoders();  ///< Send s r0x32 0\r to all three amplifiers

private:
    // ---- Init helpers -------------------------------------------------------

    /**
     * @brief Upgrade one amplifier from 9600 to 115200 baud per the Copley guide:
     *   1. Send "s r0x90 115200\r" at 9600
     *   2. Wait 150 ms
     *   3. Switch Teensy HW serial to 115200
     *   4. Verify by sending "g r0x90\r" and checking the response
     * @return true if the verification response contained a value near 115200
     */
    bool UpgradeBaudRate(HardwareSerial& port);

    /**
     * @brief Send "s r0x24 3\r" (PWM current mode) and wait for "ok\r".
     */
    void SetPwmMode(HardwareSerial& port);

    /**
     * @brief Reset one amplifier: enable pin HIGH→LOW, wait 500 ms, HIGH.
     */
    void ResetAmplifier(uint8_t enablePin);

    // ---- Poll cycle helpers -------------------------------------------------

    /** @brief Send "g r0x32\r" to all three ports at once. */
    void StartPollCycle();

    /** @brief Drive one port's receive state machine one step. */
    void PollPort(HardwareSerial& port, AmpQueryState& amp);

    /** @brief True when all three amps have returned to IDLE. */
    bool AllIdle() const;

    /**
     * @brief Parse "v <integer>" from a fixed buffer.
     *        Expects rxBuf to contain the response without the trailing '\r'.
     */
    int32_t ParseValueResponse(const AmpQueryState& amp) const;

    // ---- Members ------------------------------------------------------------
    AmpQueryState ampA_, ampB_, ampC_;

    SharedDataManager&           dataHandle_;
    std::shared_ptr<ManagedData> shared_;
};
