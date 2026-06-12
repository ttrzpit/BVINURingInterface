// =============================================================================
// BVI NURing Teensy Firmware — main.cpp
//
// Three IntervalTimers run concurrently:
//
//   IT_DrivePWM      (1000 Hz) — calls Amplifier.DrivePWMFromISR() directly.
//                                Contains only analogWrite(); safe in ISR.
//
//   IT_PollAmplifiers ( 500 Hz) — sets flagPollHWSerial.
//                                 Main loop calls Amplifier.PollHWSerial()
//                                 to run the parallel state machines and update
//                                 shared encoder/current values.
//
//   IT_SendToPC      ( 200 Hz) — sets flagSendToPC.
//                                Main loop calls SerialPort.SendToPC() to push
//                                a TeensyToPcPacket with fresh telemetry.
//
// LED status (pins 30/31/32):
//   10 Hz blink — baud upgrade in progress (connected at 9600, waiting for 115200)
//   ON (solid)  — amplifier A/B/C successfully upgraded to 115200 baud
//   OFF         — upgrade failed (amplifier not responding at high baud)
//
// Built-in LED (pin 13):
//   1 Hz pulse  — powered on, no PC connection
//  10 Hz pulse  — PC serial connection active
// =============================================================================

#include <Arduino.h>
#include "T_SharedDataManager.h"
#include "T_AmplifierClass.h"
#include "T_SerialClass.h"

// ---- Shared objects ---------------------------------------------------------

SharedDataManager dataHandle;
T_AmplifierClass  Amplifier(dataHandle);
T_SerialClass     SerialPort(dataHandle);

// ---- ISR flags (volatile — written in ISR, cleared in main loop) ------------

volatile bool flagPollHWSerial = false;
volatile bool flagSendToPC     = false;

// ---- ISR callbacks ----------------------------------------------------------

// Global pointer so lambdas or plain functions can reach the Amplifier object
T_AmplifierClass* gAmplifier = nullptr;

void IT_DrivePWM_Callback() {
    if (gAmplifier) gAmplifier->DrivePWMFromISR();
}

void IT_PollAmp_Callback()  { flagPollHWSerial = true; }
void IT_SendPC_Callback()   { flagSendToPC     = true; }

// ---- IntervalTimers ---------------------------------------------------------

IntervalTimer IT_DrivePWM;
IntervalTimer IT_PollAmplifiers;
IntervalTimer IT_SendToPC;


// =============================================================================
// setup()
// =============================================================================

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWriteFast(LED_BUILTIN, LOW);

    // Initialize USB serial first so the PC can see startup messages
    SerialPort.Begin();

    // Full amplifier init: reset, baud upgrade, PWM mode, enable, zero encoders
    // Status LEDs (pins 30/31/32) light up as each amp successfully upgrades.
    gAmplifier = &Amplifier;
    Amplifier.Begin();

    // Start interval timers after Begin() completes so PWM writes begin with
    // safe (off) values and the poll state machines start from IDLE.
    IT_DrivePWM.begin(IT_DrivePWM_Callback, PERIOD_PWM_US);
    IT_PollAmplifiers.begin(IT_PollAmp_Callback,  PERIOD_POLL_AMP_US);
    IT_SendToPC.begin(IT_SendPC_Callback,    PERIOD_SEND_PC_US);
}


// =============================================================================
// loop()
// =============================================================================

void loop() {

    // ---- Always: parse incoming bytes from the PC ---------------------------
    // ReadFromPC() updates commandedPwm and packetIndex in shared data.
    SerialPort.ReadFromPC();

    // ---- One-shot: zero motor encoders (requested during pretensioning) -----
    if (dataHandle.getData()->System.zeroEncoderRequested) {
        Amplifier.ZeroEncoders();
        dataHandle.getData()->System.zeroEncoderRequested = false;
    }

    // ---- 500 Hz: run the parallel amplifier state machines ------------------
    if (flagPollHWSerial) {
        flagPollHWSerial = false;
        Amplifier.PollHWSerial();
    }

    // ---- 200 Hz: send telemetry packet to PC --------------------------------
    if (flagSendToPC) {
        flagSendToPC = false;
        SerialPort.SendToPC();
    }

    // ---- Built-in LED: heartbeat pulse --------------------------------------
    // 1 Hz when idle, 10 Hz when PC serial is active.
    static uint32_t ledLastToggle = 0;
    static bool     ledState      = false;
    uint32_t halfPeriod = SerialPort.IsConnected() ? 50 : 500;  // ms
    uint32_t now = millis();
    if (now - ledLastToggle >= halfPeriod) {
        ledLastToggle = now;
        ledState = !ledState;
        digitalWriteFast(LED_BUILTIN, ledState ? HIGH : LOW);
    }
}
