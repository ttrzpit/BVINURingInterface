//[DOXYGEN]
/**
 * @file SerialClass.cpp
 * @author your name (you@domain.com)
 * @brief Manage serial communications
 * @version 0.1
 * @date 2025-05-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// Call to class header
#include "SerialClass.h"

// System data manager
#include "SystemDataManager.h"



/**
 * @brief Construct a new Serial Class:: Serial Class object
 * 
 * @param ctx Handle to data manager
 * @param nPorts Number of ports to activate
 */
SerialClass::SerialClass( SystemDataManager& ctx, uint8_t nPorts )
	: dataHandle( ctx )
	, shared( ctx.getData() ) {
	nPortsOpen = nPorts;
	if ( nPortsOpen == 1 ) {
		InitializePort0();
	} else if ( nPortsOpen == 2 ) {
		InitializePort0();
		InitializePort1();
	} else {
		std::cout << "SerialClass: Invalid number of ports to open!";
	}
}



/**
 * @brief Initialize first serial port for sending data
 * 
 */
void SerialClass::InitializePort0() {

	// Open serial port
	SerialOut = open( CONFIG_SERIAL_PORT_0.c_str(), O_RDWR | O_NONBLOCK );

	// Get existing settings
	if ( tcgetattr( SerialOut, &tty0 ) != 0 ) {
		printf( "SerialClass:  Error %i from tctgetattr: %s\n", errno, strerror( errno ) );
		shared->Serial.isSerialSendOpen = false;
	} else {
		std::cout << "SerialClass:  Serial interface " << CONFIG_SERIAL_PORT_0 << " (PC -> Teensy) connected.\n";
		shared->Serial.isSerialSendOpen = true;
	}

	// Configure port
	tty0.c_cflag &= ~PARENB;														   // Clear parity bit, disabling parity (most common)
	tty0.c_cflag &= ~CSTOPB;														   // Clear stop field, only one stop bit used in communication (most common)
	tty0.c_cflag &= ~CSIZE;															   // Clear all bits that set the data size
	tty0.c_cflag |= CS8;															   // 8 bits per byte (most common)
	tty0.c_cflag &= ~CRTSCTS;														   // Disable RTS/CTS hardware flow control (most common)
	tty0.c_cflag |= CREAD | CLOCAL;													   // Turn on READ & ignore ctrl lines (CLOCAL = 1)
	tty0.c_lflag &= ~ICANON;														   //
	tty0.c_lflag &= ~ECHO;															   // Disable echo
	tty0.c_lflag &= ~ECHOE;															   // Disable erasure
	tty0.c_lflag &= ~ECHONL;														   // Disable new-line echo
	tty0.c_lflag &= ~ISIG;															   // Disable interpretation of INTR, QUIT and SUSP
	tty0.c_iflag &= ~( IXON | IXOFF | IXANY );										   // Turn off s/w flow ctrl
	tty0.c_iflag &= ~( IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL );	   // Disable any special handling of received bytes
	tty0.c_oflag &= ~OPOST;															   // Prevent special interpretation of output bytes (e.g. newline chars)
	tty0.c_oflag &= ~ONLCR;															   // Prevent conversion of newline to carriage return/line feed
	tty0.c_cc[VTIME] = 2;															   // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty0.c_cc[VMIN]	 = 0;

	// Set in/out baud rate to be 9600
	cfsetispeed( &tty0, B1000000 );
	cfsetospeed( &tty0, B1000000 );

	// Save tty0 settings, also checking for error
	if ( tcsetattr( SerialOut, TCSANOW, &tty0 ) != 0 ) {
		printf( "SerialClass:  Error %i from tcsetattr: %s\n", errno, strerror( errno ) );
	}
}


/**
 * @brief Initialize second serial port
 * 
 */
void SerialClass::InitializePort1() {
	// Open serial port
	SerialIn = open( CONFIG_SERIAL_PORT_1.c_str(), O_RDWR | O_NONBLOCK );

	// Get existing settings
	if ( tcgetattr( SerialIn, &tty1 ) != 0 ) {
		printf( "SerialClass:  Error %i from tctgetattr: %s\n", errno, strerror( errno ) );
		shared->Serial.isSerialReceiveOpen = false;
	} else {
		std::cout << "SerialClass:  Serial interface " << CONFIG_SERIAL_PORT_1 << " (Teensy -> PC) connected.\n";
		shared->Serial.isSerialReceiveOpen = true;
	}

	// Configure port
	tty1.c_cflag &= ~PARENB;														   // Clear parity bit, disabling parity (most common)
	tty1.c_cflag &= ~CSTOPB;														   // Clear stop field, only one stop bit used in communication (most common)
	tty1.c_cflag &= ~CSIZE;															   // Clear all bits that set the data size
	tty1.c_cflag |= CS8;															   // 8 bits per byte (most common)
	tty1.c_cflag &= ~CRTSCTS;														   // Disable RTS/CTS hardware flow control (most common)
	tty1.c_cflag |= CREAD | CLOCAL;													   // Turn on READ & ignore ctrl lines (CLOCAL = 1)
	tty1.c_lflag &= ~ICANON;														   // Canonical input
	tty1.c_lflag &= ~ECHO;															   // Disable echo
	tty1.c_lflag &= ~ECHOE;															   // Disable erasure
	tty1.c_lflag &= ~ECHONL;														   // Disable new-line echo
	tty1.c_lflag &= ~ISIG;															   // Disable interpretation of INTR, QUIT and SUSP
	tty1.c_iflag &= ~( IXON | IXOFF | IXANY );										   // Turn off s/w flow ctrl
	tty1.c_iflag &= ~( IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL );	   // Disable any special handling of received bytes
	tty1.c_oflag &= ~OPOST;															   // Prevent special interpretation of output bytes (e.g. newline chars)
	tty1.c_oflag &= ~ONLCR;															   // Prevent conversion of newline to carriage return/line feed
	tty1.c_cc[VTIME] = 2;															   // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty1.c_cc[VMIN]	 = 0;

	// Set in/out baud rate to be 9600
	cfsetispeed( &tty1, B1000000 );
	cfsetospeed( &tty1, B1000000 );

	// Save tty0 settings, also checking for error
	if ( tcsetattr( SerialIn, TCSANOW, &tty1 ) != 0 ) {
		printf( "SerialClass:  Error %i from tcsetattr: %s\n", errno, strerror( errno ) );
	}
}



void SerialClass::Close() {
	close( SerialOut );
	close( SerialIn );
}



/** ==================================================
 *  ================================================== 
 * 
 *  UU   UU  PPPPP    DDDDD      AAA    TTTTTT  EEEEEE 
 *  UU   UU  PP   PP  DD   DD   AA AA     TT    EE    
 *  UU   UU  PP   PP  DD   DD   AA AA     TT    EE  
 *  UU   UU  PPPPP    DD   DD  AAAAAAA    TT    EEEEE    
 *  UU   UU  PP       DD   DD  AA   AA    TT    EE    
 *  UU   UU  PP       DD   DD  AA   AA    TT    EE    
 *   UUUUU   PP       DDDDD    AA   AA    TT    EEEEEE 
 * 
 *  ================================================== 
 *  ================================================== 
*/



/**
 * @brief Update serial interface
 * 
 */
void SerialClass::Update() {

	// Make sure outgoing serial port is open and running
	if ( shared->Serial.isSerialSendOpen && shared->Serial.isSerialSending ) {

		// Send packet
		SendPacketToTeensy();
	}

	// Make sure incoming serial port is open and running
	if ( shared->Serial.isSerialReceiveOpen && shared->Serial.isSerialReceiving ) {

		PacketStruct incomingPacket;

		if ( ReadTeensyPacket( incomingPacket ) ) {

			ParsePacketFromTeensy( incomingPacket );

			// Calculate time
			elapsedTimeNow			   = shared->Timing.elapsedRunningTime;
			shared->Serial.packetDelay = ( elapsedTimeNow - elapsedTimeLast ) * 1000.0f;
			elapsedTimeLast			   = elapsedTimeNow;


		} else {
		}
	}
}



/** =============================================== 
 *  =============================================== 
 * 
 *  SSSS   EEEEEE  NN   NN   DDDD  
 * SS      EE      NNN  NN   DD  DD 
 * SS      EE      NNN  NN   DD  DD 
 *  SSSS   EEEE    NN NNNN   DD  DD 
 *     SS  EE      NN  NNN   DD  DD 
 *     SS  EE      NN   NN   DD  DD 
 *  SSSS   EEEEEE  NN   NN   DDDD  
 * 
 *  =============================================== 
 *  =============================================== 
*/



void SerialClass::SendPacketToTeensy() {

	// Local
	uint8_t		 buffer[32];
	size_t		 idx = 0;
	PacketStruct outgoingPacket;
	uint8_t		 newType;

	if ( shared->System.state == stateEnum::IDLE ) {
		newType = 'I';
	} else if ( shared->System.state == stateEnum::DRIVING_PWM ) {
		newType = 'D';
	} else if ( shared->System.state == stateEnum::MEASURING_LIMITS ) {
		newType = 'L';
	} else if ( shared->System.state == stateEnum::ZERO_ENCODER ) {
		newType = 'Z';
	} else {
		// Nope
	}


	const uint8_t startByte	   = 0xAA;
	const uint8_t endByte	   = 0x55;
	const uint8_t packetLength = sizeof( outgoingPacket );

	if ( shared->Amplifier.packetCounter == 99 ) {
		shared->Amplifier.packetCounter = 0;
	} else {
		shared->Amplifier.packetCounter++;
	}

	// Populate packet
	outgoingPacket.packetType	  = newType;
	outgoingPacket.packetCounter  = shared->Amplifier.packetCounter;
	outgoingPacket.amplifierState = shared->Amplifier.isAmplifierActive;


	if ( shared->System.state == stateEnum::IDLE ) {
		outgoingPacket.pwmA = 2048;
		outgoingPacket.pwmB = 2048;
		outgoingPacket.pwmC = 2048;
		// outgoingPacket.safetySwitch = shared->Vibration.isRunning;
	} else {
		outgoingPacket.pwmA = shared->Controller.commandedPwmABC.x;
		outgoingPacket.pwmB = shared->Controller.commandedPwmABC.y;
		outgoingPacket.pwmC = shared->Controller.commandedPwmABC.z;

		// Toggle Reverse
		if ( shared->Controller.toggleReverse && shared->Task.isRunning ) {
			if ( shared->Amplifier.isReverseConstant ) {
				outgoingPacket.reverseToggle = 1;
			} else {
				outgoingPacket.reverseToggle = 2;
			}
		} else {
			outgoingPacket.reverseToggle = 0;
		}

		// Disable reverse
		// outgoingPacket.reverseToggle = 0;
	}

	// Compute checksum
	uint8_t	 checkSum = newType ^ packetLength;
	uint8_t* raw	  = reinterpret_cast<uint8_t*>( &outgoingPacket );
	for ( uint8_t i = 0; i < packetLength; ++i ) {
		checkSum ^= raw[i];
	}

	// Build packet buffer
	buffer[idx++] = startByte;		 // 0xAA
	buffer[idx++] = packetLength;	 // sizeof(outgoingPacket)
	buffer[idx++] = checkSum;		 // sizeof(outgoingPacket)

	memcpy( &buffer[idx], &outgoingPacket, packetLength );
	idx += packetLength;
	buffer[idx++] = endByte;	// sizeof(outgoingPacket)

	// Send over serial port 0
	ssize_t bytesWritten = write( SerialOut, buffer, idx );
	if ( bytesWritten < 0 ) {
		shared->Serial.packetOut = "FAILED!";
		std::cout << "SerialClass: Failed to send packet!\n";
	} else {

		// std::cout << "Outgoing Packet: " << outgoingPacket.packetType << "\n";
		// StringOutput( buffer );
		ConvertPacketToSerialString( outgoingPacket );
	}
}



/** =========================================================== 
 *  ===========================================================  
 * 
 *  RRRRR    EEEEEE    CCCCC  EEEEEE  IIIIII  VV     VV  EEEEEE 
 *  RR   RR  EE      CC       EE        II    VV     VV  EE     
 *  RR   RR  EE      CC       EE        II     VV   VV   EE   
 *  RRRRR    EEEEE   CC       EEEEE     II     VV   VV   EEEEE     
 *  RR   RR  EE      CC       EE        II      VV VV    EE     
 *  RR   RR  EE      CC       EE        II      VV VV    EE     
 *  RR   RR  EEEEEE    CCCCC  EEEEEE  IIIIII     VVV     EEEEEE 
 * 
 *  ===========================================================  
 *  ===========================================================  
*/



bool SerialClass::ReadTeensyPacket( PacketStruct& outPacket ) {

	constexpr uint8_t START_BYTE = 0xAA;
	constexpr uint8_t END_BYTE	 = 0x55;	// Assumed

	uint8_t buffer[32];
	size_t	bufferIndex = 0;
	// std::cout << "Attempt read...";

	// Sync: look for START_BYTE
	while ( true ) {

		uint8_t byte;

		if ( read( SerialIn, &byte, 1 ) <= 0 ) {

			std::cout << "ReadTeensyPacket: Empty!     ";
			return false;	 // Read fail
		}
		if ( byte == START_BYTE ) {
			// std::cout << "Byte detected     ";
		}
		break;
	}

	// Read packetLength and checksum
	if ( read( SerialIn, buffer, 2 ) != 2 ) {
		std::cout << "ReadTeensyPacket: Not equal 2";
		return false;
	}

	uint8_t packetLength	 = buffer[0];
	uint8_t expectedChecksum = buffer[1];

	// Read payload + 2 footer bytes (0x00 + END_BYTE)
	if ( read( SerialIn, buffer, packetLength + 1 ) != static_cast<ssize_t>( packetLength + 1 ) ) {
		std::cout << "ReadTeensyPacket: Wrong size";
		return false;
	}

	// Validate checksum
	uint8_t actualChecksum = buffer[0] ^ packetLength;
	for ( size_t i = 0; i < packetLength; ++i ) {
		actualChecksum ^= buffer[i];
	}

	if ( actualChecksum != expectedChecksum ) {
		std::cout << "ReadTeensyPacket: Checksum mismatch!" << std::endl;
		return false;
	}

	// Validate footer
	if ( buffer[packetLength] != END_BYTE ) {
		std::cout << "ReadTeensyPacket: Invalid footer!" << std::endl;
		return false;
	}


	// Copy payload into struct
	std::memcpy( &outPacket, buffer, sizeof( PacketStruct ) );
	return true;
}



/** ==================================================
 *  ================================================== 
 * 
 *  PPPPPP      AAA    RRRRR     SSSS   EEEEEE 
 *  PP    PP   AA AA   RR   RR  SS   	EE    
 *  PP    PP   AA AA   RR   RR  SS      EE  
 *  PPPPPP    AAAAAAA  RRRRR     SSSS   EEEEE    
 *  PP        AA   AA  RR   RR      SS  EE    
 *  PP        AA   AA  RR   RR      SS  EE    
 *  PP        AA   AA  RR   RR   SSSS   EEEEEE 
 * 
 *  ================================================== 
 *  ================================================== 



/**
 * @brief Parse incoming packet
 * 
 * @param type Packet type
 * @param length Length of packet
 * @param bytes Bytes to convert
 */
void SerialClass::ParsePacketFromTeensy( PacketStruct pkt ) {

	// Store values
	shared->Amplifier.packetType			= pkt.packetType;
	shared->Amplifier.packetCounter			= pkt.packetCounter;
	shared->Amplifier.packetDriveState		= pkt.amplifierState;
	shared->Amplifier.measuredPwmA			= pkt.pwmA;
	shared->Amplifier.measuredPwmB			= pkt.pwmB;
	shared->Amplifier.measuredPwmC			= pkt.pwmC;
	shared->Amplifier.currentMeasuredRawA	= pkt.currentA;
	shared->Amplifier.currentMeasuredRawB	= pkt.currentB;
	shared->Amplifier.currentMeasuredRawC	= pkt.currentC;
	shared->Amplifier.encoderMeasuredCountA = pkt.encoderA;
	shared->Amplifier.encoderMeasuredCountB = pkt.encoderB;
	shared->Amplifier.encoderMeasuredCountC = pkt.encoderC;
	shared->Amplifier.isSafetySwitchEngaged = pkt.reverseToggle;

	// Below moved to controller update
	// shared->Amplifier.currentMeasuredAmpsA	= shared->Amplifier.currentMeasuredRawA * 0.01f;
	// shared->Amplifier.currentMeasuredAmpsB	= shared->Amplifier.currentMeasuredRawB * 0.01f;
	// shared->Amplifier.currentMeasuredAmpsC	= shared->Amplifier.currentMeasuredRawC * 0.01f;
	// shared->Amplifier.encoderMeasuredDegA	= ( shared->Amplifier.encoderMeasuredCountA / 4096.0f ) * 360.0f;
	// shared->Amplifier.encoderMeasuredDegB	= ( shared->Amplifier.encoderMeasuredCountB / 4096.0f ) * 360.0f;
	// shared->Amplifier.encoderMeasuredDegC	= ( shared->Amplifier.encoderMeasuredCountC / 4096.0f ) * 360.0f;

	// set back to idle
	if ( pkt.packetType == 'z' ) {
		shared->System.state = stateEnum::IDLE;
	}

	// Save as string
	ConvertPacketToSerialString( pkt );
}



void SerialClass::StringOutput( const uint8_t* buff ) {

	std::cout << "t:" << shared->FormatDecimal( shared->Timing.elapsedRunningTime, 4, 3 );
	std::cout << "HEX:   ";
	std::cout << "[HD]:" << std::hex << static_cast<int>( buff[0] ) << "  ";
	std::cout << "[SZ]:" << std::hex << static_cast<int>( buff[1] ) << "  ";
	std::cout << "[CK]:" << std::setw( 2 ) << std::setfill( '0' ) << std::hex << static_cast<int>( buff[2] );
	std::cout << " --- ";
	std::cout << "[TP]:" << std::hex << static_cast<int>( buff[3] ) << "  ";
	std::cout << "[CT]:" << std::setw( 2 ) << std::setfill( '0' ) << std::hex << static_cast<int>( buff[4] ) << "  ";
	std::cout << "[ST]:" << std::hex << static_cast<int>( buff[5] );
	std::cout << " --- PWM: ";
	std::cout << "A[" << std::hex << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[7] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[6] ) << "]  ";
	std::cout << "B[" << std::hex << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[9] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[8] ) << "]  ";
	std::cout << "C[" << std::hex << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[11] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[10] ) << "]  ";
	std::cout << " --- Cur: ";
	std::cout << "A[" << std::hex << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[13] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[12] ) << "]  ";
	std::cout << "B[" << std::hex << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[15] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[14] ) << "]  ";
	std::cout << "C[" << std::hex << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[17] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[16] ) << "]  ";
	std::cout << " --- Enc: ";
	std::cout << "A[" << std::hex << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[21] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[20] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[19] ) << " " << std::setw( 2 )
			  << std::setfill( '0' ) << static_cast<int>( buff[18] ) << "]  ";
	std::cout << "B[" << std::hex << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[25] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[24] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[23] ) << " " << std::setw( 2 )
			  << std::setfill( '0' ) << static_cast<int>( buff[22] ) << "]  ";
	std::cout << "C[" << std::hex << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[29] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[28] ) << " " << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( buff[27] ) << " " << std::setw( 2 )
			  << std::setfill( '0' ) << static_cast<int>( buff[26] ) << "]  ";
	std::cout << " --- [30]:" << std::hex << static_cast<int>( buff[30] ) << "  ";
	std::cout << "[31]:" << std::hex << static_cast<int>( buff[31] );
	std::cout << "\n";


	uint16_t pwmA = static_cast<uint16_t>( buff[6] ) | ( static_cast<uint16_t>( buff[7] ) << 8 );
	uint16_t pwmB = static_cast<uint16_t>( buff[8] ) | ( static_cast<uint16_t>( buff[9] ) << 8 );
	uint16_t pwmC = static_cast<uint16_t>( buff[10] ) | ( static_cast<uint16_t>( buff[11] ) << 8 );
	int16_t	 curA = static_cast<int16_t>( static_cast<uint16_t>( buff[12] ) | ( static_cast<uint16_t>( buff[13] ) << 8 ) );
	int16_t	 curB = static_cast<int16_t>( static_cast<uint16_t>( buff[14] ) | ( static_cast<uint16_t>( buff[15] ) << 8 ) );
	int16_t	 curC = static_cast<int16_t>( static_cast<uint16_t>( buff[16] ) | ( static_cast<uint16_t>( buff[17] ) << 8 ) );
	int32_t	 encA = static_cast<int32_t>( static_cast<uint32_t>( buff[18] ) | ( static_cast<uint32_t>( buff[19] ) << 8 ) | ( static_cast<uint32_t>( buff[20] ) << 16 ) | ( static_cast<uint32_t>( buff[21] ) << 24 ) );
	int32_t	 encB = static_cast<int32_t>( static_cast<uint32_t>( buff[22] ) | ( static_cast<uint32_t>( buff[23] ) << 8 ) | ( static_cast<uint32_t>( buff[24] ) << 16 ) | ( static_cast<uint32_t>( buff[25] ) << 24 ) );
	int32_t	 encC = static_cast<int32_t>( static_cast<uint32_t>( buff[26] ) | ( static_cast<uint32_t>( buff[27] ) << 8 ) | ( static_cast<uint32_t>( buff[28] ) << 16 ) | ( static_cast<uint32_t>( buff[29] ) << 24 ) );

	std::cout << "t:" << shared->FormatDecimal( shared->Timing.elapsedRunningTime, 4, 3 );
	std::cout << "DEC:   ";
	std::cout << "[HD]:" << std::hex << static_cast<int>( buff[0] ) << "  ";
	std::cout << "[SZ]:" << std::dec << static_cast<int>( buff[1] ) << "  ";
	std::cout << "[CK]:" << std::setw( 2 ) << std::setfill( '0' ) << std::hex << static_cast<int>( buff[2] );
	std::cout << " --- ";
	std::cout << "[TP]:" << std::hex << std::setw( 2 ) << std::setfill( ' ' ) << static_cast<char>( buff[3] ) << "  ";
	std::cout << "[CT]:" << std::setw( 2 ) << std::setfill( '0' ) << std::dec << static_cast<int>( buff[4] ) << "  ";
	std::cout << "[ST]:" << std::dec << static_cast<int>( buff[5] );
	std::cout << " --- PWM: ";
	std::cout << "A[ " << std::dec << std::setw( 4 ) << std::setfill( '0' ) << pwmA << "]  ";
	std::cout << "A[ " << std::dec << std::setw( 4 ) << std::setfill( '0' ) << pwmB << "]  ";
	std::cout << "A[ " << std::dec << std::setw( 4 ) << std::setfill( '0' ) << pwmC << "]  ";
	std::cout << " --- Cur: ";
	std::cout << "A[ " << std::dec << std::setw( 4 ) << std::setfill( '0' ) << curA << "]  ";
	std::cout << "B[ " << std::dec << std::setw( 4 ) << std::setfill( '0' ) << curB << "]  ";
	std::cout << "C[ " << std::dec << std::setw( 4 ) << std::setfill( '0' ) << curC << "]  ";
	std::cout << " --- Enc: ";
	std::cout << "A[   " << std::dec << std::setw( 8 ) << std::setfill( '0' ) << encA << "]  ";
	std::cout << "B[   " << std::dec << std::setw( 8 ) << std::setfill( '0' ) << encB << "]  ";
	std::cout << "C[   " << std::dec << std::setw( 8 ) << std::setfill( '0' ) << encC << "]  ";
	std::cout << " --- [30]:" << std::dec << static_cast<int>( buff[30] ) << "  ";
	std::cout << "[31]:" << std::dec << static_cast<int>( buff[31] );

	std::cout << "\n\n";
}



/**
 * @brief Converts and saves the packet to a string
 */
void SerialClass::ConvertPacketToSerialString( PacketStruct packet ) {


	// Create output string
	std::ostringstream oss;

	// Extract type
	uint8_t type = packet.packetType;

	// Build header
	if ( type == 'I' || type == 'i' ) {
		oss << ( std::isupper( type ) ? "IDLE" : "idle " ) << " [ ";
	} else if ( type == 'D' || type == 'd' ) {
		oss << ( std::isupper( type ) ? "DRIVE" : "drive" ) << " [ ";
	} else if ( type == 'L' || type == 'l' ) {
		oss << ( std::isupper( type ) ? "LIMITS" : "limits" ) << " [ ";
	} else if ( type == 'C' || type == 'c' ) {
		oss << ( std::isupper( type ) ? "CURRENT" : "current" ) << " [ ";
	} else if ( type == 'Z' || type == 'z' ) {
		oss << ( std::isupper( type ) ? "ZERO" : "zero" ) << " [ ";
	} else {
		oss << "!ERROR!" << " [ ";
	}

	// Build string
	oss << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( packet.packetCounter ) << " ] [ ";
	oss << std::setw( 1 ) << std::setfill( '0' ) << static_cast<int>( packet.amplifierState ) << " ] [ ";
	oss << std::setw( 4 ) << std::setfill( '0' ) << static_cast<int>( packet.pwmA ) << " ";
	oss << std::setw( 4 ) << std::setfill( '0' ) << static_cast<int>( packet.pwmB ) << " ";
	oss << std::setw( 4 ) << std::setfill( '0' ) << static_cast<int>( packet.pwmC ) << " ] [ ";
	oss << std::setw( 4 ) << std::setfill( ' ' ) << static_cast<int>( packet.currentA ) << " ";
	oss << std::setw( 4 ) << std::setfill( ' ' ) << static_cast<int>( packet.currentB ) << " ";
	oss << std::setw( 4 ) << std::setfill( ' ' ) << static_cast<int>( packet.currentC ) << " ] [ ";
	oss << std::setw( 6 ) << std::setfill( ' ' ) << static_cast<int>( packet.encoderA ) << " ";
	oss << std::setw( 6 ) << std::setfill( ' ' ) << static_cast<int>( packet.encoderB ) << " ";
	oss << std::setw( 6 ) << std::setfill( ' ' ) << static_cast<int>( packet.encoderC ) << " ] [ ";
	oss << std::setw( 1 ) << std::setfill( ' ' ) << static_cast<int>( packet.reverseToggle ) << " ]";

	if ( std::isupper( type ) ) {

		// std::cout << "OUT: " << oss.str() << "\n";
		shared->Serial.packetOut = oss.str();
	} else {
		// std::cout << "IN: " << shared->Timing.elapsedRunningTime << "   " << oss.str() << "\n";
		shared->Serial.packetIn = oss.str();
	}
}



/** =============================================== **/
/** HELPERS HELPERS HELPERS HELPERS HELPERS HELPERS **/
/** HELPERS HELPERS HELPERS HELPERS HELPERS HELPERS **/
/** HELPERS HELPERS HELPERS HELPERS HELPERS HELPERS **/
/** =============================================== **/

/**
 * @brief Print the given byte
 * 
 * @param pktBytes Vector of bytes
 */
void SerialClass::PrintByte( std::vector<uint8_t> pktBytes ) {

	for ( size_t i = 0; i < pktBytes.size(); ++i ) {
		std::cout << std::hex << std::setw( 2 ) << std::setfill( '0' ) << static_cast<int>( pktBytes[i] ) << " ";
	}
	std::cout << std::dec << "\n";
}
