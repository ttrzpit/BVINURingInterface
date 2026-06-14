#pragma once

// =============================================================================
// T_Config.h — Pin assignments and compile-time constants
// =============================================================================

#include <Arduino.h>

// ---- Amplifier PWM output pins (analogWrite, 12-bit) ------------------------
#define AMP_PIN_PWM_A     7
#define AMP_PIN_PWM_B     8
#define AMP_PIN_PWM_C    25

// ---- Amplifier enable pins (digital) ----------------------------------------
#define AMP_PIN_ENABLE_A 35
#define AMP_PIN_ENABLE_B 34
#define AMP_PIN_ENABLE_C 33

// ---- Status LED pins ---------------------------------------------------------
// LED turns ON when the corresponding amplifier successfully upgrades to 115200.
// Stays OFF if verification fails (amplifier not responding at high baud).
#define LED_PIN_AMP_A    30
#define LED_PIN_AMP_B    31
#define LED_PIN_AMP_C    32

// ---- HW serial port aliases --------------------------------------------------
#define HWSerialA Serial4   // Motor A (Copley NES-090-10-Z)
#define HWSerialB Serial5   // Motor B
#define HWSerialC Serial3   // Motor C

// ---- Amplifier serial baud rates ---------------------------------------------
inline constexpr uint32_t BAUD_INIT = 9600;    // Copley power-up default
inline constexpr uint32_t BAUD_FAST = 115200;  // Target after init sequence

// ---- PWM constants (12-bit, 0–4095) -----------------------------------------
inline constexpr uint16_t PWM_OFF = 2047;  // Zero torque output (safe default)
inline constexpr uint16_t PWM_MAX = 1;     // Maximum torque

// ---- Comms watchdog -----------------------------------------------------------
// If no valid PcToTeensyPacket has been received within this window (PC crashed
// or serial disconnected), ReadFromPC() forces commandedPwm_A/B/C back to
// PWM_OFF so the amplifiers stop applying tension. The PC sends at 200 Hz
// (5 ms period), so 250 ms is ~50 missed packets — comfortably above normal
// jitter but fast enough to cut output quickly on a real disconnect.
inline constexpr uint32_t WATCHDOG_TIMEOUT_MS = 250;

// ---- IntervalTimer periods (microseconds) ------------------------------------
inline constexpr uint32_t PERIOD_PWM_US      = 1000;  // 1000 Hz — PWM analogWrite
inline constexpr uint32_t PERIOD_POLL_AMP_US = 2000;  //  500 Hz — HW serial poll
inline constexpr uint32_t PERIOD_SEND_PC_US  = 5000;  //  200 Hz — USB serial send
