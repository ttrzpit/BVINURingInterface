// Ensure this header is only initialized once
#pragma once

// Encoder library
#include <Encoder.h>

// Encoder pins
#define PIN_ENCODER_A 10
#define PIN_ENCODER_B 11

Encoder encoder(PIN_ENCODER_A, PIN_ENCODER_B);

// Main class definition
class EncoderClass {

public:

  EncoderClass() {
  }

  // Variables

  long rawCount = 0;
  float angleDeg = 0.0f;
  bool valueUpdated = false;

  // Public functions
  void ReadEncoders();
  void Begin();
  void Reset();


private:
  long newCount = 0;
};


void EncoderClass::ReadEncoders() {

  newCount = encoder.read();
  if (newCount != rawCount) {
    rawCount = newCount;
    valueUpdated = true;
    angleDeg = degrees(rawCount * (2.0f * M_PI) / (2 * 4096));
  } else {
    valueUpdated = false;
  }
}


void EncoderClass::Begin() {

  // Reset encoder values
  rawCount = 0;
  encoder.write(0);
}


void EncoderClass::Reset() {

  // Reset encoder values
  rawCount = 0;
  encoder.write(0);
}