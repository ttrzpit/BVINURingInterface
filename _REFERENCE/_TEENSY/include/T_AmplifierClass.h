/** Amplifier Class **/

#pragma once

// Memory for shared data
#include "T_Config.h"
#include "queue"
#include <Arduino.h>
#include <memory>

// Forward declarations
class SharedDataManager;
struct ManagedData;

// Hardware serial ports
#define HWSerialA Serial5	 // AdEx serial
#define HWSerialB Serial4	 // AbEx serial
#define HWSerialC Serial3	 // Flex serial


/** 
 * @brief Amplifier  class definition
 */
class T_AmplifierClass {

public:
	// Default constructor
	T_AmplifierClass( SharedDataManager& dataHandle );

	// Public functions
	void Begin();
	void Disable();
	void DrivePWM();
	void Enable();
	void Update();
	void UpdateHWSerial();
	void ZeroEncoder();

private:
	// Data manager handle
	std::shared_ptr<ManagedData> shared;
	SharedDataManager&			 dataHandle;

	// Amplifier Functions
	void ResetIntoPwmMode();
	void ZeroAmplifierOutput();
	void Reset();

	/** HWSerial Elements */
	// HWSerial commands
	String asciiSetIdle			= "";
	String asciiGetCurrent		= "g r0x0c\r";
	String asciiGetBaud			= "g r0x90\r";
	String asciiGetEncoderCount = "g r0x17\r";
	String asciiGetName			= "g f0x92\r";
	String asciiSetBaud115237	= "s r0x90 115237\r";
	String asciiSetCurrentMode	= "s r0x24 3\r";
	String asciiSetEncoderZero	= "s r0x17 0\r";

	// HWSerial query states
	String queryStateA = "";
	String queryStateB = "";
	String queryStateC = "";

	// HWSerial query responses
	String queryResponseA = "";
	String queryResponseB = "";
	String queryResponseC = "";

	// HWSerial query flags
	bool isAwaitingResponseA = false;
	bool isAwaitingResponseB = false;
	bool isAwaitingResponseC = false;

	// Query Queues
	std::queue<String> queryQueueA;
	std::queue<String> queryQueueB;
	std::queue<String> queryQueueC;

	// HWSerial Query Functions
	void HWSerial_EnqueueQueryA( const String& cmd );	 // Add query to queue A
	void HWSerial_EnqueueQueryB( const String& cmd );	 // Add query to queue B
	void HWSerial_EnqueueQueryC( const String& cmd );	 // Add query to queue C
	void HWSerial_ProcessQueryA();						 // Process next query in queue A
	void HWSerial_ProcessQueryB();						 // Process next query in queue B
	void HWSerial_ProcessQueryC();						 // Process next query in queue C
	void HWSerial_ReadQueryA();							 // Read response to query A
	void HWSerial_ReadQueryB();							 // Read response to query B
	void HWSerial_ReadQueryC();							 // Read response to query C
	void HWSerial_ParseResponseA();						 // Parse response to query A
	void HWSerial_ParseResponseB();						 // Parse response to query B
	void HWSerial_ParseResponseC();						 // Parse response to query C
};
