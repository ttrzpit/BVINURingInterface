#pragma once

// =============================================================================
// T_SharedDataManager.h — Shared data bus between all firmware classes
//
// Mirrors the reference pattern exactly: a SharedDataManager owns a
// shared_ptr<ManagedData>; each class receives a reference to the manager
// and calls getData() in its constructor to hold its own shared_ptr copy.
//
// Data ownership:
//   AmplifierData.commandedPwm_*   written by T_SerialClass, read by ISR
//   AmplifierData.encoder_count_*  written by T_AmplifierClass, read by T_SerialClass
//   AmplifierData.current_raw_*    written by T_AmplifierClass, read by T_SerialClass
//   SystemData.*                   written by T_SerialClass
// =============================================================================

#include <Arduino.h>
#include <memory>
#include "T_Config.h"
#include "T_PacketTypes.h"


// ---- Amplifier data ----------------------------------------------------------

struct AmplifierData {

    bool isEnabled = false;

    // Measured values — written by T_AmplifierClass after each poll cycle,
    // read by T_SerialClass when building the outgoing TeensyToPcPacket.
    int32_t encoder_count_A = 0;   ///< Raw counts from g r0x32 (Maxon ENX 22: 1024 counts/rev)
    int32_t encoder_count_B = 0;
    int32_t encoder_count_C = 0;
    int16_t current_raw_A   = 0;   ///< 0.01 A units from g r0x0c (e.g. 150 = 1.50 A)
    int16_t current_raw_B   = 0;
    int16_t current_raw_C   = 0;

    // Commanded PWM — written by T_SerialClass (main thread) when a PC packet
    // arrives, read by DrivePWMFromISR() inside a 1000 Hz IntervalTimer.
    // uint16_t writes are atomic on Cortex-M7 — no mutex needed.
    volatile uint16_t commandedPwm_A = PWM_OFF;
    volatile uint16_t commandedPwm_B = PWM_OFF;
    volatile uint16_t commandedPwm_C = PWM_OFF;
};


// ---- System / comms data -----------------------------------------------------

struct SystemData {
    uint8_t teensyState = TeensyState::WAITING;
    uint8_t packetIndex = 0;   ///< Echoed from the most recent valid PC packet
};


// ---- Top-level managed data --------------------------------------------------

struct ManagedData {
    AmplifierData Amplifier;
    SystemData    System;
};


// ---- Shared data manager -----------------------------------------------------

class SharedDataManager {
public:
    SharedDataManager();

    /** @brief Returns a shared_ptr to the single ManagedData instance. */
    std::shared_ptr<ManagedData> getData();

private:
    std::shared_ptr<ManagedData> data_;
};
