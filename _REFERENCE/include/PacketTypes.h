/** Packet Types **/

#pragma once

/**
 * @brief Struct for software serial packet (C++ to Teensy)
 * 
 */
#pragma pack( push, 1 )	   // Avoid padding
struct PacketStruct {

	// Elements
	uint8_t	 packetType		= 0;
	uint8_t	 packetCounter	= 0;
	uint8_t	 amplifierState = 0;
	uint16_t pwmA			= 0;
	uint16_t pwmB			= 0;
	uint16_t pwmC			= 0;
	int16_t	 currentA		= 0;
	int16_t	 currentB		= 0;
	int16_t	 currentC		= 0;
	int32_t	 encoderA		= 0;
	int32_t	 encoderB		= 0;
	int32_t	 encoderC		= 0;
	uint8_t	 reverseToggle	= 0;
};

#pragma pack( pop )