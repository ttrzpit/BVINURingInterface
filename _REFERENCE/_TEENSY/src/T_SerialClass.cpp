// Call to class header
#include "T_SerialClass.h"

// System data manager
#include "T_SharedDataManagerClass.h"


T_SerialClass::T_SerialClass( SharedDataManager& ctx )
	: dataHandle( ctx )
	, shared( ctx.getData() ) {
	shared->serialClassPtr = this;
};


/**
 * @brief Initialize the incoming and outgoing serial ports
 * 
 */
void T_SerialClass::Begin() {

	SerialIn.begin( 1000000 );	   // Input
	SerialOut.begin( 1000000 );	   // Output

	if ( shared->Serial.useDebugText ) {
		SerialDebug.begin( 1000000 );	 // Debug interface
	}
	// while ( !SerialDebug );			 // Wait until ready

	// PrintDebug( "Serial initialized" );
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
 * @brief Update
 * 
 */
void T_SerialClass::Update() {


	// // See if data available
	if ( SerialIn.available() ) {
		ReadPacketFromPC();
		SendPacketToPC();
	}
}


void T_SerialClass::PrintDebug( const String& msg ) {
	if ( shared->Serial.useDebugText ) {
		SerialDebug.println( "[Debug] " + msg );
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



/**
 * @brief Reads new packet from the PC
 * 
 */
void T_SerialClass::ReadPacketFromPC() {

	static uint8_t buffer[32];
	static uint8_t idx			  = 0;
	static uint8_t state		  = 0;
	static uint8_t expectedLength = 0;



	// Iterate while bytes available
	while ( SerialIn.available() ) {

		shared->LED.isCommunicatingWithPC = true;

		uint8_t byte = SerialIn.read();

		switch ( state ) {
			case 0:	   // Wait for start byte
				if ( byte == 0xAA ) {
					idx			  = 0;
					buffer[idx++] = byte;
					state		  = 1;
				}
				break;

			case 1:	   // Read length
				expectedLength = byte;
				buffer[idx++]  = byte;
				state		   = 2;
				break;

			case 2:	   // Read checksum
				buffer[idx++] = byte;
				state		  = 3;
				break;

			case 3:	   // Read payload
				buffer[idx++] = byte;
				if ( idx == 3 + expectedLength ) {	  // +1 for footer
					state = 4;
				}
				break;

			case 4:	   // Read footer
				if ( byte == 0x55 ) {
					buffer[idx++] = byte;

					// Check checksum
					uint8_t packetType = buffer[1];
					uint8_t length	   = buffer[2];
					// uint8_t receivedChecksum = buffer[3];
					uint8_t computed = packetType ^ length;

					for ( uint8_t i = 0; i < length; ++i ) {
						computed ^= buffer[4 + i];
					}

					PacketStruct* newPacket = ( PacketStruct* )&buffer[3];
					PrintDebug( "Type: " + String( newPacket->packetType ) );

					// Parse packet
					ParsePacketFromPC( newPacket );
				}
				// Reset after processing
				idx	  = 0;
				state = 0;
				break;
		}
	}
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
 */



void T_SerialClass::ParsePacketFromPC( PacketStruct* pkt ) {

	// React accordingly
	switch ( pkt->packetType ) {

		// Idle
		case 'I': {

			// Update state
			shared->System.state = stateEnum::IDLE;

			break;
		}

		// Drive PWM
		case 'D': {

			// Update state
			shared->System.state = stateEnum::DRIVING_PWM;



			break;
		}

		// Read limits
		case 'L': {

			// Update state
			shared->System.state = stateEnum::MEASURING_LIMITS;

			break;
		}

		case 'Z': {

			// Update state
			shared->System.state = stateEnum::ZERO_ENCODER;

			break;
		}

		// Read limits
		case 'C': {

			// Update state
			shared->System.state = stateEnum::MEASURING_CURRENTS;

			break;
		}

		// Default
		default: {

			// Update state
			shared->System.state = stateEnum::IDLE;
			break;
		}
	}

	// Store incoming values
	shared->Amplifier.packetCounter	 = pkt->packetCounter;
	shared->Amplifier.commandedState = pkt->amplifierState;
	shared->Amplifier.commandedPwmA	 = pkt->pwmA;
	shared->Amplifier.commandedPwmB	 = pkt->pwmB;
	shared->Amplifier.commandedPwmC	 = pkt->pwmC;
	shared->Amplifier.toggleReverse	 = pkt->toggleReverseFlag;
	// shared->Vibration.isRunning		 = pkt->vibration;

	// Toggle constant reverse
	if ( shared->Amplifier.toggleReverse == 1 ) {
		shared->Amplifier.commandedPwmA = constrain( shared->Amplifier.commandedPwmA - 450, AMPLIFIER_PWM_MAX, AMPLIFIER_PWM_ZERO );
		shared->Amplifier.commandedPwmB = constrain( shared->Amplifier.commandedPwmB - 450, AMPLIFIER_PWM_MAX, AMPLIFIER_PWM_ZERO );
		shared->Amplifier.commandedPwmC = constrain( shared->Amplifier.commandedPwmC - 450, AMPLIFIER_PWM_MAX, AMPLIFIER_PWM_ZERO );
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


/**
 * @brief Send a packet to the PC
 * 
 */
void T_SerialClass::SendPacketToPC() {

	PacketStruct outgoingPacket;
	uint8_t		 outgoingType = 'i';

	// Select type based on state
	if ( shared->System.state == stateEnum::IDLE ) {
		outgoingType = 'i';
	} else if ( shared->System.state == stateEnum::DRIVING_PWM ) {
		outgoingType = 'd';
	} else if ( shared->System.state == stateEnum::MEASURING_LIMITS ) {
		outgoingType = 'l';
	} else if ( shared->System.state == stateEnum::MEASURING_CURRENTS ) {
		outgoingType = 'c';
	} else if ( shared->System.state == stateEnum::ZERO_ENCODER ) {
		outgoingType = 'z';
	} else {
		//
	};


	// Populate packet
	outgoingPacket.packetType		 = outgoingType;
	outgoingPacket.packetCounter	 = shared->Amplifier.packetCounter;
	outgoingPacket.amplifierState	 = shared->Amplifier.isEnabled;
	outgoingPacket.pwmA				 = shared->Amplifier.commandedPwmA;
	outgoingPacket.pwmB				 = shared->Amplifier.commandedPwmB;
	outgoingPacket.pwmC				 = shared->Amplifier.commandedPwmC;
	outgoingPacket.encoderA			 = shared->Amplifier.encoderMeasuredCountA;
	outgoingPacket.encoderB			 = shared->Amplifier.encoderMeasuredCountB;
	outgoingPacket.encoderC			 = shared->Amplifier.encoderMeasuredCountC;
	outgoingPacket.currentA			 = shared->Amplifier.currentMeasuredRawA;
	outgoingPacket.currentB			 = shared->Amplifier.currentMeasuredRawB;
	outgoingPacket.currentC			 = shared->Amplifier.currentMeasuredRawC;
	outgoingPacket.toggleReverseFlag = shared->Amplifier.toggleReverse;

	// Measure packet length
	const uint8_t packetLength = sizeof( outgoingPacket );

	// Compute checksum
	uint8_t	 checkSum = outgoingType ^ packetLength;
	uint8_t* raw	  = reinterpret_cast<uint8_t*>( &outgoingPacket );
	for ( uint8_t i = 0; i < packetLength; ++i ) {
		checkSum ^= raw[i];
	}

	// Build buffer header
	uint8_t buffer[32];
	size_t	idx	  = 0;
	buffer[idx++] = startByte;		 // 0xAA
	buffer[idx++] = packetLength;	 // sizeof(outgoingPacket)
	buffer[idx++] = checkSum;		 // sizeof(outgoingPacket)

	// Copy packet into buffer
	memcpy( &buffer[idx], &outgoingPacket, packetLength );

	// Build buffer footer
	idx += packetLength;
	buffer[idx++] = endByte;	// sizeof(outgoingPacket)



	// Send over serial
	ssize_t bytesWritten = SerialOut.write( buffer, idx );
	if ( bytesWritten < 0 ) {
		PrintDebug( "Nothing sent!" );

	} else {
		PrintDebug( "Sent: " + String( outgoingPacket.packetType ) );
		// Success
	}
}