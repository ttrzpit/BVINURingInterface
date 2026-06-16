#include "T_AmplifierClass.h"

#include <stdlib.h>   // strtol - fixed-buffer int parsing, no heap

// =============================================================================
// T_AmplifierClass.cpp
//
// Baud upgrade procedure (per the Copley communication guide):
//   1. Send "s r0x90 115200\r" at 9600 - IMPORTANT: use .print() not
//      .println(). The \n that println() adds can be misinterpreted as a
//      break command that resets the baud back to 9600.
//   2. The amplifier responds "ok\r" at the NEW baud, so the Teensy won't see it.
//   3. Wait 150 ms minimum (Copley requirement).
//   4. Switch the Teensy HW serial port to 115200.
//   5. Wait 50 ms for the port to stabilize.
//   6. Verify: send "g r0x90\r" and check that the response value is near 115200.
//
// Poll cycle state machine:
//   On each PollHWSerial() call, if all three amps are IDLE, a new cycle starts
//   by sending "g r0x32\r" to all three ports simultaneously. As each response
//   arrives ('\r' received), the current query ("g r0x0c\r") is chained
//   immediately. When all three reach IDLE again, fresh values are written to
//   shared data.
// =============================================================================


T_AmplifierClass::T_AmplifierClass(SharedDataManager& dataHandle)
    : dataHandle_(dataHandle)
    , shared_(dataHandle.getData())
{}


// =============================================================================
// Begin - full initialization sequence
// =============================================================================

void T_AmplifierClass::Begin() {

    // ---- Configure output pins ----------------------------------------------
    pinMode(AMP_PIN_ENABLE_A, OUTPUT);
    pinMode(AMP_PIN_ENABLE_B, OUTPUT);
    pinMode(AMP_PIN_ENABLE_C, OUTPUT);
    pinMode(AMP_PIN_PWM_A,    OUTPUT);
    pinMode(AMP_PIN_PWM_B,    OUTPUT);
    pinMode(AMP_PIN_PWM_C,    OUTPUT);

    // ---- Status LEDs - all off until baud upgrade is verified ---------------
    pinMode(LED_PIN_AMP_A, OUTPUT);
    pinMode(LED_PIN_AMP_B, OUTPUT);
    pinMode(LED_PIN_AMP_C, OUTPUT);
    digitalWriteFast(LED_PIN_AMP_A, LOW);
    digitalWriteFast(LED_PIN_AMP_B, LOW);
    digitalWriteFast(LED_PIN_AMP_C, LOW);

    // ---- Safe initial state -------------------------------------------------
    Disable();
    analogWriteResolution(12);
    analogWrite(AMP_PIN_PWM_A, PWM_OFF);
    analogWrite(AMP_PIN_PWM_B, PWM_OFF);
    analogWrite(AMP_PIN_PWM_C, PWM_OFF);

    // ---- Reset all three amplifiers -----------------------------------------
    // Each reset holds the enable pin LOW for 500 ms then brings it back HIGH.
    ResetAmplifier(AMP_PIN_ENABLE_A);
    ResetAmplifier(AMP_PIN_ENABLE_B);
    ResetAmplifier(AMP_PIN_ENABLE_C);

    // ---- Start HW serial at 9600 (Copley power-up default) ------------------
    HWSerialA.begin(BAUD_INIT);
    delay(250) ; // Stagger starts to avoid concurrent port conflicts
    HWSerialB.begin(BAUD_INIT);
    delay(250) ; // Stagger starts to avoid concurrent port conflicts
    HWSerialC.begin(BAUD_INIT);
    delay(500);  // Allow amplifiers to finish booting after reset

    // ---- Baud upgrade: 9600 → 115200 (per Copley guide) --------------------
    // Each upgrade is done sequentially to avoid concurrent port conflicts.
    // LED lights on success; stays off on failure.

    bool okA = UpgradeBaudRate(HWSerialA, LED_PIN_AMP_A);
    digitalWriteFast(LED_PIN_AMP_A, okA ? HIGH : LOW);

    bool okB = UpgradeBaudRate(HWSerialB, LED_PIN_AMP_B);
    digitalWriteFast(LED_PIN_AMP_B, okB ? HIGH : LOW);

    bool okC = UpgradeBaudRate(HWSerialC, LED_PIN_AMP_C);
    digitalWriteFast(LED_PIN_AMP_C, okC ? HIGH : LOW);

    // ---- Set PWM current mode (s r0x24 3\r) on each amplifier ---------------
    SetPwmMode(HWSerialA);
    SetPwmMode(HWSerialB);
    SetPwmMode(HWSerialC);

    // ---- Enable amplifiers and zero encoders --------------------------------
    // Zeroing here (once, at boot) establishes q_abs = 0 <-> bare-pulley for the
    // spool-radius model in ControllerHandler AND keeps the amplifier's
    // internal cogging-compensation table aligned to the motor's commutation
    // reference. It must NOT be repeated at runtime (e.g. during pretensioning)
    // - re-zeroing later shifts that reference and cogging returns.
    Enable();
    ZeroEncoders();

}


// =============================================================================
// ISR-safe PWM write (1000 Hz)
// =============================================================================

void T_AmplifierClass::DrivePWMFromISR() {
    // Read volatile commandedPwm values from shared data and apply.
    // ONLY analogWrite() here - no serial, no branching, no heap.
    analogWrite(AMP_PIN_PWM_A, shared_->Amplifier.commandedPwm_A);
    analogWrite(AMP_PIN_PWM_B, shared_->Amplifier.commandedPwm_B);
    analogWrite(AMP_PIN_PWM_C, shared_->Amplifier.commandedPwm_C);
}


// =============================================================================
// HW serial polling (500 Hz, called from main loop)
// =============================================================================

void T_AmplifierClass::PollHWSerial() {

    // Start a new poll cycle if all three have completed the previous one
    if (AllIdle()) {
        StartPollCycle();
    }

    // Drain any bytes that have arrived on each port and advance each state machine
    PollPort(HWSerialA, ampA_);
    PollPort(HWSerialB, ampB_);
    PollPort(HWSerialC, ampC_);

    // When the cycle is complete, push fresh values to shared data
    // (so T_SerialClass always reads the latest completed values)
    if (AllIdle() && ampA_.encoderCount != 0) {
        shared_->Amplifier.encoder_count_A = ampA_.encoderCount;
        shared_->Amplifier.encoder_count_B = ampB_.encoderCount;
        shared_->Amplifier.encoder_count_C = ampC_.encoderCount;
        shared_->Amplifier.current_raw_A   = ampA_.currentRaw;
        shared_->Amplifier.current_raw_B   = ampB_.currentRaw;
        shared_->Amplifier.current_raw_C   = ampC_.currentRaw;
    }
}


// =============================================================================
// Control interface
// =============================================================================

void T_AmplifierClass::Enable() {
    digitalWriteFast(AMP_PIN_ENABLE_A, HIGH);
    digitalWriteFast(AMP_PIN_ENABLE_B, HIGH);
    digitalWriteFast(AMP_PIN_ENABLE_C, HIGH);
    shared_->Amplifier.isEnabled = true;
}

void T_AmplifierClass::Disable() {
    digitalWriteFast(AMP_PIN_ENABLE_A, LOW);
    digitalWriteFast(AMP_PIN_ENABLE_B, LOW);
    digitalWriteFast(AMP_PIN_ENABLE_C, LOW);
    shared_->Amplifier.isEnabled = false;
}

void T_AmplifierClass::ZeroEncoders() {
    HWSerialA.print("s r0x32 0\r");
    HWSerialB.print("s r0x32 0\r");
    HWSerialC.print("s r0x32 0\r");
    delay(100);
}


// =============================================================================
// Private - initialization helpers
// =============================================================================

bool T_AmplifierClass::UpgradeBaudRate(HardwareSerial& port, uint8_t ledPin) {

    // Step 1: Send baud change command at 9600.
    // CRITICAL: use .print() not .println() - the extra \n can trigger a break
    // condition that resets the Copley baud rate back to 9600 (Copley manual warning).
    port.print("s r0x90 115200\r");

    // Step 2: The amplifier responds "ok\r" at the NEW baud so we cannot read it.
    // Wait the minimum required time for the amplifier to complete the switch.
    DelayBlink(ledPin, 150);

    // Step 3: Switch the Teensy port to 115200
    port.begin(BAUD_FAST);

    // Step 4: Wait for the port to stabilize
    DelayBlink(ledPin, 50);

    // Step 5: Flush any stale bytes from the RX buffer
    while (port.available()) port.read();

    // Step 6: Send verification query
    port.print("g r0x90\r");

    // Step 7: Wait for response with timeout, blinking the LED throughout
    char     verifyBuf[20] = {};
    uint8_t  verifyIdx     = 0;
    uint32_t startMs       = millis();
    uint32_t lastToggle    = millis();
    bool     ledState      = false;

    while (millis() - startMs < 150) {
        if (millis() - lastToggle >= 50) {
            ledState = !ledState;
            digitalWriteFast(ledPin, ledState ? HIGH : LOW);
            lastToggle = millis();
        }
        if (port.available()) {
            char c = static_cast<char>(port.read());
            if (c == '\r') break;
            if (verifyIdx < sizeof(verifyBuf) - 1) {
                verifyBuf[verifyIdx++] = c;
            }
        }
    }
    verifyBuf[verifyIdx] = '\0';

    // Step 8: Validate - Copley responds "v <value>" where value is the actual
    // baud rate set (may differ slightly from 115200 - any value >100000 is OK)
    if (verifyIdx >= 3 && verifyBuf[0] == 'v' && verifyBuf[1] == ' ') {
        int32_t returnedBaud = strtol(verifyBuf + 2, nullptr, 10);
        if (returnedBaud > 100000 && returnedBaud < 130000) {
            return true;
        }
    }

    return false;  // No response or unexpected value at 115200
}

void T_AmplifierClass::DelayBlink(uint8_t ledPin, uint32_t durationMs) {
    uint32_t start      = millis();
    uint32_t lastToggle = millis();
    bool     state      = false;
    while (millis() - start < durationMs) {
        if (millis() - lastToggle >= 50) {
            state = !state;
            digitalWriteFast(ledPin, state ? HIGH : LOW);
            lastToggle = millis();
        }
    }
}

void T_AmplifierClass::SetPwmMode(HardwareSerial& port) {
    while (port.available()) port.read();  // Flush stale bytes
    port.print("s r0x24 3\r");

    // Wait for "ok\r" response (200ms timeout)
    char    buf[8] = {};
    uint8_t idx    = 0;
    uint32_t startMs = millis();

    while (millis() - startMs < 200) {
        if (port.available()) {
            char c = static_cast<char>(port.read());
            if (c == '\r') break;
            if (idx < sizeof(buf) - 1) buf[idx++] = c;
        }
    }
    // Response "ok" - we don't strictly validate it here; the amplifier will
    // behave correctly if it accepted the mode change
}

void T_AmplifierClass::ResetAmplifier(uint8_t enablePin) {
    // Copley reset: bring enable HIGH, then LOW, wait, then HIGH again
    digitalWriteFast(enablePin, HIGH);
    digitalWriteFast(enablePin, LOW);
    delay(500);
    digitalWriteFast(enablePin, HIGH);
}


// =============================================================================
// Private - parallel poll cycle
// =============================================================================

void T_AmplifierClass::StartPollCycle() {
    // Send position query to all three ports simultaneously.
    // Each port is independent so all three transmit at the same time.
    HWSerialA.print("g r0x32\r");
    ampA_.phase = AmpQueryState::Phase::SENT_POS;
    ampA_.rxIdx = 0;

    HWSerialB.print("g r0x32\r");
    ampB_.phase = AmpQueryState::Phase::SENT_POS;
    ampB_.rxIdx = 0;

    HWSerialC.print("g r0x32\r");
    ampC_.phase = AmpQueryState::Phase::SENT_POS;
    ampC_.rxIdx = 0;
}

void T_AmplifierClass::PollPort(HardwareSerial& port, AmpQueryState& amp) {

    while (port.available()) {
        char c = static_cast<char>(port.read());

        if (c == '\r') {
            // Response complete - null-terminate and parse
            amp.rxBuf[amp.rxIdx] = '\0';

            if (amp.phase == AmpQueryState::Phase::SENT_POS) {
                amp.encoderCount = ParseValueResponse(amp);
                // Immediately chain the current query - no wait needed
                port.print("g r0x0c\r");
                amp.phase = AmpQueryState::Phase::SENT_CUR;

            } else if (amp.phase == AmpQueryState::Phase::SENT_CUR) {
                amp.currentRaw = static_cast<int16_t>(ParseValueResponse(amp));
                amp.phase = AmpQueryState::Phase::IDLE;
            }

            amp.rxIdx = 0;

        } else if (amp.rxIdx < sizeof(amp.rxBuf) - 1) {
            amp.rxBuf[amp.rxIdx++] = c;
        }
    }
}

bool T_AmplifierClass::AllIdle() const {
    return ampA_.phase == AmpQueryState::Phase::IDLE &&
           ampB_.phase == AmpQueryState::Phase::IDLE &&
           ampC_.phase == AmpQueryState::Phase::IDLE;
}

int32_t T_AmplifierClass::ParseValueResponse(const AmpQueryState& amp) const {
    // Copley successful get response: "v <integer>" (without trailing \r)
    if (amp.rxIdx >= 2 && amp.rxBuf[0] == 'v' && amp.rxBuf[1] == ' ') {
        return strtol(amp.rxBuf + 2, nullptr, 10);
    }
    return 0;  // Unexpected response (error or empty) - treat as zero
}
