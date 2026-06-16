#pragma once

// =============================================================================
// T_PacketTypes.h - Shared binary packet definitions for PC ↔ Teensy serial
//
// These structs MUST be kept identical on both the PC side (PacketTypes.h) and
// the Teensy side (this file). Pragma pack(1) ensures no padding bytes so that
// sizeof() and memcpy-based serialisation match on both ends.
//
// Wire format for both directions:
//   [0xAA]  start byte
//   [N bytes]  raw struct payload (no padding)
//   [XOR checksum]  XOR of all payload bytes
//
// PC → Teensy payload:  8 bytes  (PcToTeensyPacket)
// Teensy → PC payload: 20 bytes  (TeensyToPcPacket)
// =============================================================================

#include <stdint.h>

// ---- Framing ----------------------------------------------------------------

static constexpr uint8_t PACKET_START_BYTE = 0xAA;

// ---- State bytes sent from PC to Teensy -------------------------------------

namespace PcState {
    static constexpr uint8_t IDLE        = 'I';
    static constexpr uint8_t CALIBRATING = 'C';
    static constexpr uint8_t FITTS       = 'F';
    static constexpr uint8_t READY       = 'R';  ///< RobotState::READY/GUIDING - preload tension held
}

// ---- State bytes sent from Teensy to PC -------------------------------------

namespace TeensyState {
    static constexpr uint8_t WAITING = 'W';   // No PC connection yet
    static constexpr uint8_t IDLE    = 'I';   // Connected, not driving
    static constexpr uint8_t DRIVING = 'D';   // Actively driving motors
    static constexpr uint8_t READY   = 'R';   // Connected, preload tension held (PcState::READY)
}

// ---- PC → Teensy (8 bytes payload) ------------------------------------------

#pragma pack(push, 1)

struct PcToTeensyPacket {
    uint8_t  state;         ///< PC system state (see PcState namespace)
    uint8_t  packet_index;  ///< 0–99 rolling counter; Teensy echoes this back
    uint16_t pwm_A;         ///< Motor A commanded PWM (0 = full torque, 2047 = off)
    uint16_t pwm_B;         ///< Motor B commanded PWM
    uint16_t pwm_C;         ///< Motor C commanded PWM
};

// ---- Teensy → PC (20 bytes payload) -----------------------------------------

struct TeensyToPcPacket {
    uint8_t  state;           ///< Teensy system state (see TeensyState namespace)
    uint8_t  packet_index;    ///< Echoed from the most recent valid PcToTeensyPacket
    int16_t  current_raw_A;   ///< Motor A measured current (raw ADC counts)
    int16_t  current_raw_B;   ///< Motor B measured current (raw ADC counts)
    int16_t  current_raw_C;   ///< Motor C measured current (raw ADC counts)
    int32_t  encoder_count_A; ///< Motor A incremental encoder count
    int32_t  encoder_count_B; ///< Motor B incremental encoder count
    int32_t  encoder_count_C; ///< Motor C incremental encoder count
};

#pragma pack(pop)

// Compile-time size guards - catch accidental struct changes immediately
static_assert(sizeof(PcToTeensyPacket)  ==  8, "PcToTeensyPacket must be 8 bytes");
static_assert(sizeof(TeensyToPcPacket)  == 20, "TeensyToPcPacket must be 20 bytes");
