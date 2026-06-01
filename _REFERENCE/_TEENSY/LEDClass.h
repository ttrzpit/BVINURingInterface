/* PIN INITIALIZATION CLASS */



#pragma once



// Pin declarations
#define PIN_LED_SERIAL_A 30  // LED indicators for serial status
#define PIN_LED_SERIAL_B 31
#define PIN_LED_SERIAL_C 32



// Main class definition
class LEDClass {

public:
  // Constructor
  LEDClass() {
    // Calls
  }

  void Begin();
  void ToggleBuiltInLED();
  void Amplifiers(bool state);
  void PC(bool state);
  void Enabled(bool state);
};


void LEDClass::Begin() {
  // Configure pin mode
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LED_SERIAL_A, OUTPUT);
  pinMode(PIN_LED_SERIAL_B, OUTPUT);
  pinMode(PIN_LED_SERIAL_C, OUTPUT);


  // Set pin initial states
  digitalWriteFast(LED_BUILTIN, LOW);
  digitalWriteFast(PIN_LED_SERIAL_A, LOW);  // Set LED pins low by default
  digitalWriteFast(PIN_LED_SERIAL_B, LOW);
  digitalWriteFast(PIN_LED_SERIAL_C, LOW);
}


// Toggles the onboard LED
void LEDClass::ToggleBuiltInLED() {
  digitalToggle(LED_BUILTIN);
}

void LEDClass::Amplifiers(bool state) {
  digitalWriteFast(PIN_LED_SERIAL_B, state);
}

void LEDClass::PC(bool state) {
  digitalWriteFast(PIN_LED_SERIAL_A, state);
}

void LEDClass::Enabled(bool state) {
  digitalWriteFast(PIN_LED_SERIAL_C, state);
}
