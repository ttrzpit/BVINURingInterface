/** SharedDataManager Class Header */

#pragma once

#pragma once

// Memory for shared data
#include "T_Config.h"
#include "T_PacketTypes.h"
#include "T_SerialClass.h"
#include "queue"
#include <Arduino.h>
#include <memory>

// Shared data manager
class SharedDataManager;
class T_SerialClass;
struct ManagedData;

enum class stateEnum { WAITING, IDLE, DRIVING_PWM, MEASURING_LIMITS, MEASURING_CURRENTS, ZERO_ENCODER };


struct AmplifierStruct {

	// Amplifier enabled flags
	bool isEnabled		   = false;
	bool isEncoderReset	   = false;
	bool isHWSerialEnabled = false;
	bool useDebugPort	   = true;

	// Reverse flag
	int toggleReverse = 0;


	bool isQuerySent = false;

	// Packet variables
	uint8_t	 packetType			   = 'W';	 // Waiting initially
	uint8_t	 commandedState		   = 0;
	uint8_t	 packetCounter		   = 0;
	uint16_t commandedPwmA		   = AMPLIFIER_PWM_ZERO;
	uint16_t commandedPwmB		   = AMPLIFIER_PWM_ZERO;
	uint16_t commandedPwmC		   = AMPLIFIER_PWM_ZERO;
	int16_t	 currentMeasuredRawA   = 0;
	int16_t	 currentMeasuredRawB   = 0;
	int16_t	 currentMeasuredRawC   = 0;
	int32_t	 encoderMeasuredCountA = 0;
	int32_t	 encoderMeasuredCountB = 0;
	int32_t	 encoderMeasuredCountC = 0;

	// Other variables
	String	nameA = "";
	String	nameB = "";
	String	nameC = "";
	int32_t baudA = 0;
	int32_t baudB = 0;
	int32_t baudC = 0;
};

struct LEDStruct {

	// Flags
	bool isCommunicatingWithPC = false;
	bool isDrivingMotors	   = false;
	bool isMeasuringLimits	   = false;
};



/**
 * @brief Struct for serial port data
 * 
 */
struct SerialStruct {

	// Serial flags
	bool isIncomingPacketReady = false;
	bool isOutgoingPacketReady = false;

	// Packets
	String packetIn	 = "";
	String packetOut = "";

	// Debug
	String debugText	= "";
	bool   useDebugText = true;
};



/**
 * @brief Struct for system data and flags
 * 
 */
struct SystemStruct {

	stateEnum state = stateEnum::WAITING;
};



struct TimingStruct {

	// Interval timer periods
	int32_t periodAmplifier		 = 1000000 / TIMING_FREQ_AMPLIFIER_DRIVE;
	int32_t periodHWSerial		 = 1000000 / TIMING_FREQ_AMPLIFIER_HWSERIAL;
	int32_t periodSoftwareSerial = 1000000 / TIMING_FREQ_AMPLIFIER_SOFTWARESERIAL;
};



/**
 * @brief Main struct that gets shared
 * 
 */
struct ManagedData {

	// Declare system structs
	AmplifierStruct Amplifier;
	LEDStruct		LED;
	SerialStruct	Serial;
	SystemStruct	System;
	TimingStruct	Timing;

	// Point to serial class

	T_SerialClass* serialClassPtr = nullptr;
	void		   PrintDebug( const String& msg );
};


/**
 * @brief 
 * 
 */
class SharedDataManager {
public:
	// Class
	SharedDataManager();

	// Functions
	std::shared_ptr<ManagedData> getData();



private:
	std::shared_ptr<ManagedData> data;
};