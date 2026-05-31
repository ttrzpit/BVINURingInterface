#pragma once

// Memory for shared data
#include <memory>

// Packet types
#include "PacketTypes.h"

// Serial libraries
#include <cctype>		// For determining upper/lower case
#include <fcntl.h>		// File controls
#include <iomanip>		// For padding zeroes
#include <termios.h>	// Terminal control
#include <unistd.h>		// Write, read, and close functions

// Vectors
#include <vector>

// Forward declarations
class SystemDataManager;
struct ManagedData;



/** 
 * @brief Display class definition
 */
class SerialClass {

public:
	// Data manager handle
	SerialClass( SystemDataManager& dataHandle, uint8_t nPorts );

	// Public functions
	void Close();
	void Update();



private:
	// Data manager handle
	SystemDataManager&			 dataHandle;
	std::shared_ptr<ManagedData> shared;

	// Private functions
	void InitializePort0();
	void InitializePort1();

	// Port Variables
	int			   SerialOut;
	int			   SerialIn;
	struct termios tty0;
	struct termios tty1;
	int8_t		   nPortsOpen = 1;

	// Serial functions
	void SendPacketToTeensy();
	// void ReadPacketFromTeensy();
	bool ReadTeensyPacket( PacketStruct& outPacket );
	void ParsePacketFromTeensy( PacketStruct pkt );				// Parse packet from teensy and save data
	void ConvertPacketToSerialString( PacketStruct packet );	// Convert packet to string for debugging
	void PrintByte( std::vector<uint8_t> pktBytes );			// Print contents of a byte vector
	void StringOutput( const uint8_t* buff );

	float elapsedTimeNow = 0.0f; 
	float elapsedTimeLast = 0.0f; 

};