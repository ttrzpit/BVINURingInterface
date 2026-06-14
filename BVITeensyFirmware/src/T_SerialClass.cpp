#include "T_SerialClass.h"

#include <string.h>  // memcpy

// =============================================================================
// T_SerialClass.cpp
//
// Framing (identical on both PC and Teensy):
//   RX (PC → Teensy):  [0xAA] [8 bytes PcToTeensyPacket] [XOR checksum]
//   TX (Teensy → PC): [0xAA] [20 bytes TeensyToPcPacket] [XOR checksum]
//
// On a valid incoming packet the following are written to shared data:
//   commandedPwm_A/B/C  — read by DrivePWMFromISR() at 1000 Hz
//   packetIndex         — echoed in the next outgoing packet
//   teensyState         — reflected from PC state
// =============================================================================

T_SerialClass::T_SerialClass(SharedDataManager& dataHandle)
    : dataHandle_(dataHandle)
    , shared_(dataHandle.getData())
{}

void T_SerialClass::Begin() {
    Serial.begin(1000000);  // Nominal baud — USB CDC speed is USB-limited
    delay(500);             // Allow USB stack to enumerate
}


// =============================================================================
// ReadFromPC — call every loop() iteration
// =============================================================================

void T_SerialClass::ReadFromPC() {

    while (Serial.available() > 0) {

        uint8_t byte = static_cast<uint8_t>(Serial.read());

        switch (rxPhase_) {

            case RxPhase::WAIT_START:
                if (byte == PACKET_START_BYTE) {
                    rxPhase_ = RxPhase::READ_PAYLOAD;
                    rxIdx_   = 0;
                }
                break;

            case RxPhase::READ_PAYLOAD:
                rxBuf_[rxIdx_++] = byte;
                if (rxIdx_ == sizeof(PcToTeensyPacket)) {
                    rxPhase_ = RxPhase::READ_CHECKSUM;
                }
                break;

            case RxPhase::READ_CHECKSUM: {
                uint8_t expected = ComputeChecksum(rxBuf_, sizeof(PcToTeensyPacket));

                if (byte == expected) {
                    PcToTeensyPacket pkt;
                    memcpy(&pkt, rxBuf_, sizeof(PcToTeensyPacket));

                    connected_ = true;
                    lastValidPacketMillis_ = millis();

                    // Write commanded PWM values into shared data.
                    // These are declared volatile — safe for ISR to read concurrently.
                    shared_->Amplifier.commandedPwm_A = pkt.pwm_A;
                    shared_->Amplifier.commandedPwm_B = pkt.pwm_B;
                    shared_->Amplifier.commandedPwm_C = pkt.pwm_C;

                    // Store packet index and reflect PC state
                    shared_->System.packetIndex = pkt.packet_index;
                    switch (pkt.state) {
                        case PcState::IDLE:        shared_->System.teensyState = TeensyState::IDLE;    break;
                        case PcState::CALIBRATING: shared_->System.teensyState = TeensyState::IDLE;    break;
                        case PcState::FITTS:       shared_->System.teensyState = TeensyState::IDLE;    break;
                        case PcState::READY:       shared_->System.teensyState = TeensyState::READY;   break;
                        case PcState::ZERO_ENC:
                            shared_->System.teensyState = TeensyState::IDLE;
                            shared_->System.zeroEncoderRequested = true;
                            break;
                        default:                   shared_->System.teensyState = TeensyState::WAITING; break;
                    }
                }

                rxPhase_ = RxPhase::WAIT_START;
                break;
            }
        }
    }

    // ---- Comms watchdog -------------------------------------------------------
    // No valid packet within WATCHDOG_TIMEOUT_MS (PC crashed or serial
    // disconnected) — force PWM output off so the amplifiers stop applying
    // tension instead of holding the last commanded value indefinitely.
    if (millis() - lastValidPacketMillis_ > WATCHDOG_TIMEOUT_MS) {
        shared_->Amplifier.commandedPwm_A = PWM_OFF;
        shared_->Amplifier.commandedPwm_B = PWM_OFF;
        shared_->Amplifier.commandedPwm_C = PWM_OFF;
        connected_ = false;
        shared_->System.teensyState = TeensyState::WAITING;
    }
}


// =============================================================================
// SendToPC — call at 200 Hz from the flagged path in loop()
// =============================================================================

void T_SerialClass::SendToPC() {

    TeensyToPcPacket pkt;
    pkt.state           = shared_->System.teensyState;
    pkt.packet_index    = shared_->System.packetIndex;
    pkt.current_raw_A   = shared_->Amplifier.current_raw_A;
    pkt.current_raw_B   = shared_->Amplifier.current_raw_B;
    pkt.current_raw_C   = shared_->Amplifier.current_raw_C;
    pkt.encoder_count_A = shared_->Amplifier.encoder_count_A;
    pkt.encoder_count_B = shared_->Amplifier.encoder_count_B;
    pkt.encoder_count_C = shared_->Amplifier.encoder_count_C;

    constexpr size_t PAYLOAD = sizeof(TeensyToPcPacket);
    uint8_t buf[PAYLOAD + 2];

    buf[0] = PACKET_START_BYTE;
    memcpy(&buf[1], &pkt, PAYLOAD);
    buf[PAYLOAD + 1] = ComputeChecksum(reinterpret_cast<const uint8_t*>(&pkt), PAYLOAD);

    Serial.write(buf, sizeof(buf));
}


// =============================================================================
// Private
// =============================================================================

uint8_t T_SerialClass::ComputeChecksum(const uint8_t* data, size_t len) const {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}
