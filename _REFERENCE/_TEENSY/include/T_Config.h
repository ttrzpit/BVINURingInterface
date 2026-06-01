#pragma once

#include <Arduino.h>


/**
 * @brief Amplifier config values
 * 
 */
// Amplifier pins
#define AMPLIFIER_PIN_ENABLE_A 35
#define AMPLIFIER_PIN_ENABLE_B 34
#define AMPLIFIER_PIN_ENABLE_C 33
#define AMPLIFIER_PIN_PWM_A 7
#define AMPLIFIER_PIN_PWM_B 8
#define AMPLIFIER_PIN_PWM_C 25

// LED pins
#define LED_PIN_READY 30
#define LED_PIN_DRIVING 31
#define LED_PIN_MEASURING 32

// Killswitch pins
#define PIN_SAFETY_SWITCH 9

// Amplifier constants
inline constexpr unsigned short AMPLIFIER_PWM_ZERO = 2047;	  // Minimum PWM value
inline constexpr unsigned short AMPLIFIER_PWM_MAX  = 1;		  // Maximum PWM value
// inline constexpr unsigned short AMPLIFIER_PWM_MAX  = 4096;		  // Maximum PWM value

/**
 * @brief LED config values 
 * 
 */



/**
 * @brief Timing config values
 * 
 */
inline constexpr unsigned short TIMING_FREQ_AMPLIFIER_DRIVE			 = 1000;	// Hz
inline constexpr unsigned short TIMING_FREQ_AMPLIFIER_HWSERIAL		 = 20;		// Hz
inline constexpr unsigned short TIMING_FREQ_AMPLIFIER_SOFTWARESERIAL = 200;		// Hz
