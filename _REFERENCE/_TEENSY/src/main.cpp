/** NURingTeensyFirmware **/

#include <Arduino.h>

// System data manager object handle
#include "T_SharedDataManagerClass.h"	 // Shared data manager header
SharedDataManager DataHandle;


// Custom libraries

#include "T_AmplifierClass.h"	 // Amplifier header
#include "T_Config.h"			 // Global config constants
#include "T_LEDClass.h"			 // LED header
#include "T_SerialClass.h"		 // Serial header

// Custom classes
T_AmplifierClass Amplifier( DataHandle );
T_LEDClass		 LEDs( DataHandle );
T_SerialClass	 SerialPort( DataHandle );

// Handle to shared data
auto shared = DataHandle.getData();

// Interval Timer
IntervalTimer IT_DriveAmplifiers;
IntervalTimer IT_SendSerialToPC;
IntervalTimer IT_ReadAmplifiers;

// Flags for serial IO
volatile bool flagSendSerialToPC = false;
volatile bool flagReadHWSerial	 = false;

/** FUNCTION PROTOTYPES **/
void IT_Callback_WriteToAmplifiers();
void IT_Callback_SendSerialToPC();
void IT_Callback_ReadFromAmplifiers();
void UpdateSystemState();
void ReadSafetySwitch();

/**
 * @brief Main program setup
 * 
 */
void setup() {

	// Initialize safety switch
	pinMode( PIN_SAFETY_SWITCH, INPUT_PULLDOWN );

	// Initialize LEDs
	LEDs.Begin();


	// Initialize serial ports
	SerialPort.Begin();
	// DataHandle.getData()->PrintDebug( "Debugger initialized" );

	// Toggle debug on 3rd serial port
	shared->Serial.useDebugText = false;

	// Initialize amplifiers
	Amplifier.Begin();

	// Initialize amplifier serial interface


	// Start interval timers
	IT_DriveAmplifiers.begin( IT_Callback_WriteToAmplifiers, shared->Timing.periodAmplifier );		// Drives the motors
	IT_SendSerialToPC.begin( IT_Callback_SendSerialToPC, shared->Timing.periodSoftwareSerial );		// Send data to serial
	IT_ReadAmplifiers.begin( IT_Callback_ReadFromAmplifiers, shared->Timing.periodHWSerial );		// Read data from amplifiers

	// Set default state
	shared->System.state = stateEnum::WAITING;
}



/**
 * @brief Main program loop
 * 
 * 
 */
void loop() {

	// Check safety switch
	ReadSafetySwitch();

	// Update LEDs
	LEDs.Update();

	// Handle flags from timers
	if ( flagSendSerialToPC ) {
		flagSendSerialToPC = false;
		SerialPort.Update();	// NOW safe: runs in main thread, not interrupt
	}

	if ( flagReadHWSerial ) {
		flagReadHWSerial = false;
		Amplifier.UpdateHWSerial();	   // safe in main thread
	}


	// Update system state
	UpdateSystemState();
}



void IT_Callback_WriteToAmplifiers() {



	// Update amplifier
	Amplifier.Update();
}



void IT_Callback_ReadFromAmplifiers() {
	// Amplifier.UpdateHWSerial();
	flagReadHWSerial = true;
}



void IT_Callback_SendSerialToPC() {


	// Vibrotactile.Drive();

	// Update serial port
	// SerialPort.Update();
	flagSendSerialToPC = true;
}


/**
 * @brief Change state 
 * 
 */
void UpdateSystemState() {


	// Select state
	switch ( shared->System.state ) {

		// Waiting (PC not yet connected)
		case stateEnum::WAITING: {

			// Update amplifier state
			shared->Amplifier.isEnabled = false;

			// Update LEDs
			shared->LED.isCommunicatingWithPC = false;
			shared->LED.isDrivingMotors		  = false;
			shared->LED.isMeasuringLimits	  = false;

			break;
		}

		// Idle (PC connected, not driving)
		case stateEnum::IDLE: {

			// Update amplifier state
			shared->Amplifier.isEnabled = false;

			// Update LEDs
			shared->LED.isCommunicatingWithPC = true;
			shared->LED.isDrivingMotors		  = false;
			shared->LED.isMeasuringLimits	  = false;

			break;
		}

		// Driving PWM
		case stateEnum::DRIVING_PWM: {

			// Update LEDs
			shared->LED.isCommunicatingWithPC = true;
			shared->LED.isDrivingMotors		  = true;
			shared->LED.isMeasuringLimits	  = true;

			break;
		}

		// Measuring limits / current
		case stateEnum::MEASURING_LIMITS: {

			// Update amplifier state
			shared->Amplifier.isEnabled = true;

			// Update LEDs
			shared->LED.isCommunicatingWithPC = true;
			shared->LED.isDrivingMotors		  = true;


			break;
		}

		// Zeroing encoder
		case stateEnum::ZERO_ENCODER: {

			// Update amplifier state
			shared->Amplifier.isEnabled = true;


			// Update LEDs
			shared->LED.isCommunicatingWithPC = true;
			shared->LED.isDrivingMotors		  = true;


			break;
		}



		default: {

			// Nothing

			break;
		}
	}
}


/**
 * @brief Read safety switch 
 * 
 */
void ReadSafetySwitch() {

	// Read switch
	// shared->System.toggleReverse = digitalRead( PIN_SAFETY_SWITCH );
}