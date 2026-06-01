/* Amplifier Control Class */

// Ensure this header is only initialized once
#pragma once

// Define hardware pins
#define PIN_ENABLE_A 35  // Pins to enable copley controllers & drive status LEDs
#define PIN_ENABLE_B 34
#define PIN_ENABLE_C 33
#define PIN_PWM_A 7  // PWM Output pins (on FlexPWM1.3 Timer)
#define PIN_PWM_B 8
#define PIN_PWM_C 25

// Amplifier PWM constants
#define PWM_MAX 2
#define PWM_ZERO 2047

// Hardware serial
#define SerialA Serial5  // AdEx
#define SerialB Serial4  // AbEx
#define SerialC Serial3  // Flex

// Main class definition
class AmplifierClass {

private:

public:
  // Constructor
  AmplifierClass() {
    // Calls
  }

  // Variables
  bool ENABLED = false;
  uint16_t pwmCommandedA = PWM_ZERO;
  uint16_t pwmCommandedB = PWM_ZERO;
  uint16_t pwmCommandedC = PWM_ZERO;

  // Functions
  void Begin();                                       // Initialize amplifier
  void Enable();                                      // Enable output
  void Disable();                                     // Disable output
  void AmpReset();                                    // Reset
  void DrivePWM(uint16_t A, uint16_t B, uint16_t C);  // Send PWM signal to amplifier
  void DrivePWMZero();                                // Send PWM stop signal to amplifier

  // void CalculatePWM(int16_t posHeading, int16_t posMagnitude);
  // void CalculatePwmXYZ(int16_t x, int16_t y, int16_t z);
  // void CalculatePwmAngular(int8_t thX, int8_t thY);
};



void AmplifierClass::Begin() {

  // Set pinmode
  pinMode(PIN_ENABLE_A, OUTPUT);
  pinMode(PIN_ENABLE_B, OUTPUT);
  pinMode(PIN_ENABLE_C, OUTPUT);
  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_PWM_B, OUTPUT);
  pinMode(PIN_PWM_C, OUTPUT);

  // Disable amplifiers
  AmplifierClass::Disable();

  // Set PWM resolution
  analogWriteResolution(12);

  // Set default PWM value (centered)
  analogWrite(PIN_PWM_A, PWM_ZERO);
  analogWrite(PIN_PWM_B, PWM_ZERO);
  analogWrite(PIN_PWM_C, PWM_ZERO);

  // Begin HW serial
  SerialA.begin(9600);
  SerialB.begin(9600);
  SerialC.begin(9600);

  delay(1000);
  AmpReset();

  delay(500);
  SerialA.print("s r0x24 3\n");
  delay(500);
  SerialB.print("s r0x24 3\n");
  delay(500);
  SerialC.print("s r0x24 3\n");

  delay(2000);


  digitalWriteFast(PIN_ENABLE_A, HIGH);
  digitalWriteFast(PIN_ENABLE_B, HIGH);
  digitalWriteFast(PIN_ENABLE_C, HIGH);

  // ENABLED = true;
}


// Disable amplifiers
void AmplifierClass::Disable() {

  digitalWriteFast(PIN_ENABLE_A, LOW);
  digitalWriteFast(PIN_ENABLE_B, LOW);
  digitalWriteFast(PIN_ENABLE_C, LOW);

  ENABLED = false;
}





// Enable amplifiers
void AmplifierClass::Enable() {

  digitalWriteFast(PIN_ENABLE_A, HIGH);
  digitalWriteFast(PIN_ENABLE_B, HIGH);
  digitalWriteFast(PIN_ENABLE_C, HIGH);

  ENABLED = true;
}


void AmplifierClass::AmpReset() {
  // Serial.println("Resetting...");

  digitalWriteFast(PIN_ENABLE_A, HIGH);
  digitalWriteFast(PIN_ENABLE_A, LOW);
  delay(500);
  digitalWriteFast(PIN_ENABLE_A, HIGH);

  digitalWriteFast(PIN_ENABLE_B, HIGH);
  digitalWriteFast(PIN_ENABLE_B, LOW);
  delay(500);
  digitalWriteFast(PIN_ENABLE_B, HIGH);

  digitalWriteFast(PIN_ENABLE_C, HIGH);
  digitalWriteFast(PIN_ENABLE_C, LOW);
  delay(500);
  digitalWriteFast(PIN_ENABLE_C, HIGH);
}


// Send PWM values to amplifiers
void AmplifierClass::DrivePWM(uint16_t A, uint16_t B, uint16_t C) {

  // Constrain values to prevent unexpected behavior
  A = constrain(A, 1, 2047);
  B = constrain(B, 1, 2047);
  C = constrain(C, 1, 2047);

  // Write PWM values
  analogWrite(PIN_PWM_A, A);
  analogWrite(PIN_PWM_B, B);
  analogWrite(PIN_PWM_C, C);
}

// Send PWM values to amplifiers
void AmplifierClass::DrivePWMZero() {

  // Write PWM values
  analogWrite(PIN_PWM_A, 2048);
  analogWrite(PIN_PWM_B, 2048);
  analogWrite(PIN_PWM_C, 2048);
}




// void AmplifierClass::CalculatePWM(int16_t posHeading, int16_t posMagnitude) {

//   // Convert to radians
//   float headingRad = posHeading * 3.14159f / 180.0f;

//   // Convert to cartesian
//   float x = posMagnitude * cos(headingRad);
//   float y = posMagnitude * sin(headingRad);

//   // Project onto motor axis
//   float rawA = Ax * x + Ay * y;
//   float rawB = Bx * x + By * y;
//   float rawC = Cx * x + Cy * y;

//   rawA = min(rawA, 1000.0f);
//   rawB = min(rawB, 1000.0f);
//   rawC = min(rawC, 1000.0f);

//   // Map
//   if (enabled) {
//     pwmCommandedA = 2047 - int(rawA * 20.47) - pwmTensionA;
//     pwmCommandedB = 2047 - int(rawB * 20.47) - pwmTensionB;
//     pwmCommandedC = 2047 - int(rawC * 20.47) - pwmTensionC;
//     pwmCommandedA = constrain(pwmCommandedA, 20, 2047 - pwmTensionA);
//     pwmCommandedB = constrain(pwmCommandedB, 20, 2047 - pwmTensionB);
//     pwmCommandedC = constrain(pwmCommandedC, 20, 2047 - pwmTensionC);

//   } else {
//     pwmCommandedA = pwmCenterA - pwmTensionA;
//     pwmCommandedB = pwmCenterB - pwmTensionA;
//     pwmCommandedC = pwmCenterC - pwmTensionA;
//   }
// }


// void AmplifierClass::CalculatePwmXYZ(int16_t x, int16_t y, int16_t z) {

//   // Create multiplier based on distance
//   float D = ((800.0f - z) / 800.0f) * 0.5f;

//   // Project onto motor axis and scale by distance
//   float rawA = ((Ax * x) + (Ay * y)) * D * 1.223;  // 1.223 is a multiplier to get the between 0 and 100
//   float rawB = ((Bx * x) + (By * y)) * D * 1.223;
//   float rawC = ((Cx * x) + (Cy * y)) * D;

//   int pwmA = constrain(2047 - int(rawA * 20.47), 5, 2047 - pwmTensionA);
//   int pwmB = constrain(2047 - int(rawB * 20.47), 5, 2047 - pwmTensionB);
//   int pwmC = constrain(2047 - int(rawC * 20.47), 5, 2047 - pwmTensionC);

//   // Map
//   // if (enabled) {
//   pwmCommandedA = pwmA;
//   pwmCommandedB = pwmB;
//   pwmCommandedC = pwmC;
//   // pwmCommandedA = constrain(pwmCommandedA, 20, 2047 - pwmTensionA);
//   // pwmCommandedB = constrain(pwmCommandedB, 20, 2047 - pwmTensionB);
//   // pwmCommandedC = constrain(pwmCommandedC, 20, 2047 - pwmTensionC);

//   // } else {
//   //   pwmCommandedA = pwmCenterA - pwmTensionA;
//   //   pwmCommandedB = pwmCenterB - pwmTensionA;
//   //   pwmCommandedC = pwmCenterC - pwmTensionA;
//   // }
//   // Serial.print("A ");
//   // Serial.print(pwmA);
//   // Serial.print("   B ");
//   // Serial.print(pwmB);
//   // Serial.print("   C ");
//   // Serial.print(pwmC);
//   // Serial.println() ;


//   //  pwmCommandedA =
//   //     pwmCommandedA = 2047 - int(rawA * 20.47) - pwmTensionA;
//   //     pwmCommandedB = 2047 - int(rawB * 20.47) - pwmTensionB;
//   //     pwmCommandedC = 2047 - int(rawC * 20.47) - pwmTensionC;
//   //     pwmCommandedA = constrain(pwmCommandedA, 20, 2047 - pwmTensionA);
//   //     pwmCommandedB = constrain(pwmCommandedB, 20, 2047 - pwmTensionB);
//   // pwmCommandedC = constrain(pwmCommandedC, 20, 2047 - pwmTensionC);
// }


// void AmplifierClass::CalculatePwmAngular(int8_t thX, int8_t thY) {

//   // Convert angle to percentage
//   ratioX = int(float(thX / 45.0f) * 100);
//   ratioY = int(float(thY / 45.0f) * 100);

//   // Project onto motor axis and scale by distance
//   ratioMotorA = constrain(((Ax * ratioX) + (Ay * ratioY)), 0.0f, 100.0f);
//   ratioMotorB = constrain(((Bx * ratioX) + (By * ratioY)), 0.0f, 100.0f);
//   ratioMotorC = constrain(((Cx * ratioX) + (Cy * ratioY)), 0.0f, 100.0f);

//   int pwmA = constrain(2047 - int(ratioMotorA * 20.47), 5, 2047 - pwmTensionA);
//   int pwmB = constrain(2047 - int(ratioMotorB * 20.47), 5, 2047 - pwmTensionB);
//   int pwmC = constrain(2047 - int(ratioMotorC * 20.47), 5, 2047 - pwmTensionC);

//   // Map
//   // if (enabled) {
//   pwmCommandedA = pwmA;
//   pwmCommandedB = pwmB;
//   pwmCommandedC = pwmC;
// }