#pragma once

// =============================================================================
// PacketTypes.h - Shared binary packet definitions for PC ↔ Teensy serial link
//
// Single full-duplex USB CDC port (/dev/ttyACM0).
// Frame format for both directions:
//   [0xAA] [payload bytes] [XOR checksum]
//
// Packet sizes (payload only):
//   PC → Teensy : sizeof(PcToTeensyPacket)  =  8 bytes  → 10 bytes on wire
//   Teensy → PC : sizeof(TeensyToPcPacket)  = 20 bytes  → 22 bytes on wire
//
// Naming follows nuring_variable_conventions.md:
//   pwm_A/B/C            raw PWM commands (0=full power, 2047=no power)
//   current_raw_A/B/C    raw ADC counts → convert to current_A/B/C [A] on PC
//   encoder_count_A/B/C  raw encoder ticks → convert to q_abs_A/B/C [rad] on PC
// =============================================================================

#include <cstdint>

// Packet framing
static constexpr uint8_t PACKET_START_BYTE = 0xAA;

// State bytes sent from PC to Teensy
namespace PcState {
    static constexpr uint8_t IDLE        = 'I';
    static constexpr uint8_t CALIBRATING = 'C';
    static constexpr uint8_t CAL1        = '3';
    static constexpr uint8_t CAL2        = '3';
    static constexpr uint8_t CAL3        = '3';
    static constexpr uint8_t FITTS       = 'F';
    static constexpr uint8_t READY       = 'R';  ///< RobotState::READY/GUIDING - preload tension held
}

// ---- Robot state ladder -----------------------------------------------------
// Coarse view of serial connection + tensioning + (future) guidance, derived
// each loop in main.cpp and used to drive PWM output policy and the
// telemetry display.
enum class RobotState {
    DISCONNECTED,  ///< No serial connection - no PWM values sent
    IDLE,          ///< Connected, PWM = 2047 (no output)
    READY,         ///< Connected, preload tension held (tensioning component only)
    GUIDING        ///< Guidance enabled - full controller output [NOT YET IMPLEMENTED]
};

// ---- PC → Teensy (8 bytes payload) ------------------------------------------
#pragma pack(push, 1)

struct PcToTeensyPacket {
    uint8_t  state;         ///< PC system state byte (see PcState::*)
    uint8_t  packet_index;  ///< Rolling counter 0–99; Teensy echoes this back
    uint16_t pwm_A;         ///< Motor A PWM command (0=full, 2047=off)
    uint16_t pwm_B;         ///< Motor B PWM command
    uint16_t pwm_C;         ///< Motor C PWM command
};

// ---- Teensy → PC (20 bytes payload) -----------------------------------------

struct TeensyToPcPacket {
    uint8_t  state;            ///< Teensy system state byte
    uint8_t  packet_index;     ///< Echoed from the incoming PC packet
    int16_t  current_raw_A;    ///< Motor A raw current (ADC counts) → current_A [A]
    int16_t  current_raw_B;    ///< Motor B raw current (ADC counts) → current_B [A]
    int16_t  current_raw_C;    ///< Motor C raw current (ADC counts) → current_C [A]
    int32_t  encoder_count_A;  ///< Motor A encoder ticks → q_abs_A [rad]
    int32_t  encoder_count_B;  ///< Motor B encoder ticks → q_abs_B [rad]
    int32_t  encoder_count_C;  ///< Motor C encoder ticks → q_abs_C [rad]
};

#pragma pack(pop)

static_assert(sizeof(PcToTeensyPacket)  ==  8, "PcToTeensyPacket must be 8 bytes");
static_assert(sizeof(TeensyToPcPacket)  == 20, "TeensyToPcPacket must be 20 bytes");

// ---- Serial display state (assembled by main.cpp each loop, passed to DisplayHandler)

struct SerialState {
    bool             isConnected   = false;  ///< Port is open and threads are running
    float            txFrequencyHz = 0.0f;   ///< Measured TX rate from the 200 Hz timer thread
    PcToTeensyPacket lastTx        = {};      ///< Last pending TX values (packet_index is TX-thread-managed)
    TeensyToPcPacket lastRx        = {};      ///< Last valid packet received from Teensy
    bool             hasRx         = false;   ///< True once at least one RX packet has arrived
    RobotState       robotState    = RobotState::DISCONNECTED;  ///< DISCONNECTED/IDLE/READY/GUIDING ladder
};
