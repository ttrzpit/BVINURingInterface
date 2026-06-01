/** Serial Class **/

#pragma once

// Memory for shared data
#include <Arduino.h>
#include <memory>

// Shared data manager
#include "T_SharedDataManagerClass.h"
class SharedDataManager;
struct ManagedData;

// Packet types
#include "T_PacketTypes.h"

#define SerialIn Serial
#define SerialOut SerialUSB1
#define SerialDebug SerialUSB2


/** 
 * @brief Serial class definition
 */
class T_SerialClass {

public:
	// Default constructor
	T_SerialClass( SharedDataManager& dataHandle );

	// Public functions
	void Begin();
	void Update();
	// void UpdateDebug();
	void PrintDebug( const String& msg );


private:
	// Data manager handle
	SharedDataManager&			 dataHandle;
	std::shared_ptr<ManagedData> shared;

	// Serial receive/parse/send
	void ReadPacketFromPC();
	void ParsePacketFromPC( PacketStruct* pkt );
	void SendPacketToPC();


	// Packet variables
	const uint8_t startByte = 0xAA;
	const uint8_t endByte	= 0x55;
};