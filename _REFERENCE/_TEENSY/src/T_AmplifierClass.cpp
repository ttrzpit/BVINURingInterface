// Call to class header
#include "T_AmplifierClass.h"

// System data manager
#include "T_SharedDataManagerClass.h"


T_AmplifierClass::T_AmplifierClass( SharedDataManager& ctx )
	: shared( ctx.getData() )
	, dataHandle( ctx ) {

	  };



/**
 *
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  ==================================================
 *  ================================================== 
 * 
 *    AAA    MM      MM  PPPPP
 *   AA AA   MMMM  MMMM  PP  PP
 *   AA AA   MM MMMM MM  PP  PP
 *  AAAAAAA  MM  MM  MM  PPPPP
 *  AA   AA  MM      MM  PP
 *  AA   AA  MM      MM  PP
 *  AA   AA  MM      MM  PP
 * 
 *  ================================================== 
 *  ================================================== 
*/

/**
 * @brief Initialize the incoming and outgoing serial ports
 * 
 */
void T_AmplifierClass::Begin() {

	// Configure pin directions
	pinMode( AMPLIFIER_PIN_ENABLE_A, OUTPUT );
	pinMode( AMPLIFIER_PIN_ENABLE_B, OUTPUT );
	pinMode( AMPLIFIER_PIN_ENABLE_C, OUTPUT );
	pinMode( AMPLIFIER_PIN_PWM_A, OUTPUT );
	pinMode( AMPLIFIER_PIN_PWM_B, OUTPUT );
	pinMode( AMPLIFIER_PIN_PWM_C, OUTPUT );

	// Disable amplfiers
	Disable();

	// Set analog resolution
	analogWriteResolution( 12 );

	ZeroAmplifierOutput();

	// Establish hardware serial connection to amplifiers
	HWSerialA.begin( 9600 );
	delay( 100 );
	HWSerialB.begin( 9600 );
	delay( 100 );
	HWSerialC.begin( 9600 );
	delay( 100 );

	// shared->PrintDebug( "Amplifier: HWSerial initialized." );
	dataHandle.getData()->PrintDebug( "HWSerial Initialized." );

	// Reset into PWM mode (for cogging)
	ResetIntoPwmMode();

	// Enable amplifiers
	Enable();

	// shared->PrintDebug("Amplifier: Ready!") ;
	// HWSerial_EnqueueQueryA( asciiGetName );

	// Delay to ensure reset successful
	delay( 500 );
}



/**
 * @brief Disable amplifiers
 * 
 */
void T_AmplifierClass::Disable() {

	// Send enable commands
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_A, LOW );
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_B, LOW );
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_C, LOW );

	// Update enable flags
	shared->Amplifier.isEnabled = false;
}



/**
 * @brief Enable amplifiers
 * 
 */
void T_AmplifierClass::Enable() {

	// Send enable commands
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_A, HIGH );
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_B, HIGH );
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_C, HIGH );

	// Update enable flags
	shared->Amplifier.isEnabled = true;
}



/**
 * @brief Reset amplifiers 
 * 
 */
void T_AmplifierClass::Reset() {

	// Send reset commands to amp A (high->low->high)
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_A, HIGH );
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_A, LOW );
	delay( 500 );
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_A, HIGH );

	// Send reset commands to amp B (high->low->high)
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_B, HIGH );
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_B, LOW );
	delay( 500 );
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_B, HIGH );

	// Send reset commands to amp C (high->low->high)
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_C, HIGH );
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_C, LOW );
	delay( 500 );
	digitalWriteFast( AMPLIFIER_PIN_ENABLE_C, HIGH );
}



void T_AmplifierClass::ResetIntoPwmMode() {

	Reset();

	// Send reset codes
	HWSerial_EnqueueQueryA( asciiSetCurrentMode );
	delay( 600 );
	HWSerial_EnqueueQueryB( asciiSetCurrentMode );
	delay( 600 );
	HWSerial_EnqueueQueryC( asciiSetCurrentMode );
	delay( 600 );
}



void T_AmplifierClass::ZeroEncoder() {

	// Send encoder zero commands
	HWSerial_EnqueueQueryA( asciiSetEncoderZero );
	HWSerial_EnqueueQueryB( asciiSetEncoderZero );
	HWSerial_EnqueueQueryC( asciiSetEncoderZero );
}

/**
 * @brief Sends PWM signals to amplifier
 * 
 */
void T_AmplifierClass::DrivePWM() {

	// Constrain values as a precaution
	shared->Amplifier.commandedPwmA = constrain( shared->Amplifier.commandedPwmA, AMPLIFIER_PWM_MAX, AMPLIFIER_PWM_ZERO );
	shared->Amplifier.commandedPwmB = constrain( shared->Amplifier.commandedPwmB, AMPLIFIER_PWM_MAX, AMPLIFIER_PWM_ZERO );
	shared->Amplifier.commandedPwmC = constrain( shared->Amplifier.commandedPwmC, AMPLIFIER_PWM_MAX, AMPLIFIER_PWM_ZERO );

	// shared->Amplifier.commandedPwmA = constrain( shared->Amplifier.commandedPwmA, AMPLIFIER_PWM_ZERO, AMPLIFIER_PWM_MAX );
	// shared->Amplifier.commandedPwmB = constrain( shared->Amplifier.commandedPwmB, AMPLIFIER_PWM_ZERO, AMPLIFIER_PWM_MAX );
	// shared->Amplifier.commandedPwmC = constrain( shared->Amplifier.commandedPwmC, AMPLIFIER_PWM_ZERO, AMPLIFIER_PWM_MAX );

	// Toggle vibrating reverse
	if ( shared->Amplifier.toggleReverse == 2 ) {
		shared->Amplifier.commandedPwmA = constrain( shared->Amplifier.commandedPwmA - 250, AMPLIFIER_PWM_MAX, AMPLIFIER_PWM_ZERO );
		shared->Amplifier.commandedPwmB = constrain( shared->Amplifier.commandedPwmB - 250, AMPLIFIER_PWM_MAX, AMPLIFIER_PWM_ZERO );
		shared->Amplifier.commandedPwmC = constrain( shared->Amplifier.commandedPwmC - 250, AMPLIFIER_PWM_MAX, AMPLIFIER_PWM_ZERO );
	}

	// Send command to amplifiers
	analogWrite( AMPLIFIER_PIN_PWM_A, shared->Amplifier.commandedPwmA );
	analogWrite( AMPLIFIER_PIN_PWM_B, shared->Amplifier.commandedPwmB );
	analogWrite( AMPLIFIER_PIN_PWM_C, shared->Amplifier.commandedPwmC );
}



/**
 *
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  ==================================================
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
 * @brief Main update functions, handles amplifier operations
 * 
 */
void T_AmplifierClass::UpdateHWSerial() {

	HWSerial_EnqueueQueryA( asciiGetEncoderCount );
	HWSerial_EnqueueQueryB( asciiGetEncoderCount );
	HWSerial_EnqueueQueryC( asciiGetEncoderCount );
	HWSerial_EnqueueQueryA( asciiGetCurrent );
	HWSerial_EnqueueQueryB( asciiGetCurrent );
	HWSerial_EnqueueQueryC( asciiGetCurrent );
}


/**
 * @brief Main update functions, handles amplifier operations
 * 
 */
void T_AmplifierClass::Update() {

	switch ( shared->System.state ) {

		// Waiting
		case stateEnum::WAITING: {
			ZeroAmplifierOutput();
			shared->Amplifier.isEnabled = false;
			break;
		}

		// Idle
		case stateEnum::IDLE: {

			// Drive PWM
			ZeroAmplifierOutput();
			DrivePWM();
			break;
		}

		// Drive through PWM
		case stateEnum::DRIVING_PWM: {

			// Drive PWM
			DrivePWM();
			// }

			break;
		}

		// Measure limits
		case stateEnum::MEASURING_LIMITS: {

			break;
		}

		// Measure current
		case stateEnum::MEASURING_CURRENTS: {

			break;
		}

		// Reset encoder
		case stateEnum::ZERO_ENCODER: {

			// Send zero command
			ZeroEncoder();
			shared->System.state = stateEnum::IDLE;

			break;
		}

		// Default
		default: {


			break;
		}
	}

	// Callback to HWSerialA
	HWSerial_ProcessQueryA();
	if ( HWSerialA.available() ) {
		T_AmplifierClass::HWSerial_ReadQueryA();
	}

	// Callback to HWSerialB
	HWSerial_ProcessQueryB();
	if ( HWSerialB.available() ) {
		T_AmplifierClass::HWSerial_ReadQueryB();
	}

	// Callback to HWSerialC
	HWSerial_ProcessQueryC();
	if ( HWSerialC.available() ) {
		T_AmplifierClass::HWSerial_ReadQueryC();
	}
}



/**
 * @brief Set amplifier values to zero (no current)
 * 
 */
void T_AmplifierClass::ZeroAmplifierOutput() {
	shared->Amplifier.commandedPwmA = AMPLIFIER_PWM_ZERO;
	shared->Amplifier.commandedPwmB = AMPLIFIER_PWM_ZERO;
	shared->Amplifier.commandedPwmC = AMPLIFIER_PWM_ZERO;

	// Send command to amplifiers
	analogWrite( AMPLIFIER_PIN_PWM_A, shared->Amplifier.commandedPwmA );
	analogWrite( AMPLIFIER_PIN_PWM_B, shared->Amplifier.commandedPwmB );
	analogWrite( AMPLIFIER_PIN_PWM_C, shared->Amplifier.commandedPwmC );
}



/**
 *
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  ===================================================================
 *  ===================================================================
 * 
 *  H   H  W     W     SSSS   EEEEEE  RRRRR    IIIIII   AAAAA   LL
 *  H   H  W     W    SS      EE      RR  RR     II    AA   AA  LL
 *  H   H  W     W    SS      EE      RR  RR     II    AA   AA  LL
 *  HHHHH  W  W  W     SSSS   EEEE    RRRRR      II    AAAAAAA  LL
 *  H   H  W W W W        SS  EE      RR  RR     II    AA   AA  LL
 *  H   H  WW WW W        SS  EE      RR  RR     II    AA   AA  LL
 *  H   H   W   W      SSSS   EEEEEE  RR   RR  IIIIII  AA   AA  LLLLL
 * 
 *  =================================================================== 
 *  =================================================================== 
 */



/***************************
 *  HWSerial_EnqueueQuery  *
 ***************************/

/**
 * @brief Add a new HWSerial query the queue for amplifier A
 * 
 * @param cmd ASCII command string to send to A
 */
void T_AmplifierClass::HWSerial_EnqueueQueryA( const String& cmd ) {
	queryQueueA.push( cmd );
}



/**
 * @brief Add a new HWSerial query the queue for amplifier B
 * 
 * @param cmd ASCII command string to send to B
 */
void T_AmplifierClass::HWSerial_EnqueueQueryB( const String& cmd ) {
	queryQueueB.push( cmd );
}



/**
 * @brief Add a new HWSerial query the queue for amplifier C
 * 
 * @param cmd ASCII command string to send to C
 */
void T_AmplifierClass::HWSerial_EnqueueQueryC( const String& cmd ) {
	queryQueueC.push( cmd );
}



/***************************
 *  HWSerial_ProcessQuery  *
 ***************************/

/**
  * @brief Process the next query in queue A
  * 
  */
void T_AmplifierClass::HWSerial_ProcessQueryA() {

	// Check if there isn't already a request in waiting, and the queue isn't empty
	if ( !isAwaitingResponseA && !queryQueueA.empty() ) {

		// Extract first query
		queryStateA = queryQueueA.front();
		queryQueueA.pop();

		// Update
		queryResponseA		= "";
		isAwaitingResponseA = true;

		// Send query
		HWSerialA.print( queryStateA );
	}
}



/**
  * @brief Process the next query in queue B
  * 
  */
void T_AmplifierClass::HWSerial_ProcessQueryB() {

	// Check if there isn't already a request in waiting, and the queue isn't empty
	if ( !isAwaitingResponseB && !queryQueueB.empty() ) {

		// Extract first query
		queryStateB = queryQueueB.front();
		queryQueueB.pop();

		// Update
		queryResponseB		= "";
		isAwaitingResponseB = true;

		// Send query
		HWSerialB.print( queryStateB );
	}
}



/**
  * @brief Process the next query in queue C
  * 
  */
void T_AmplifierClass::HWSerial_ProcessQueryC() {

	// Check if there isn't already a request in waiting, and the queue isn't empty
	if ( !isAwaitingResponseC && !queryQueueC.empty() ) {

		// Extract first query
		queryStateC = queryQueueC.front();
		queryQueueC.pop();

		// Update
		queryResponseC		= "";
		isAwaitingResponseC = true;

		// Send query
		HWSerialC.print( queryStateC );
	}
}



/************************
 *  HWSerial_ReadQuery  *
 ************************/

/**
  * @brief Read the response for amplifier A 
  * 
  */
void T_AmplifierClass::HWSerial_ReadQueryA() {


	while ( HWSerialA.available() > 0 ) {

		// Read character
		char incomingChar = ( char )HWSerialA.read();

		// Look for line return
		if ( incomingChar == '\r' ) {

			// Parse string
			HWSerial_ParseResponseA();

		} else {

			// Build string
			queryResponseA += incomingChar;
		}
	}
}



/**
  * @brief Read the response for amplifier B
  * 
  */
void T_AmplifierClass::HWSerial_ReadQueryB() {


	while ( HWSerialB.available() > 0 ) {

		// Read character
		char incomingChar = ( char )HWSerialB.read();

		// Look for line return
		if ( incomingChar == '\r' ) {

			// Parse string
			HWSerial_ParseResponseB();

		} else {

			// Build string
			queryResponseB += incomingChar;
		}
	}
}



/**
  * @brief Read the response for amplifier C
  * 
  */
void T_AmplifierClass::HWSerial_ReadQueryC() {


	while ( HWSerialC.available() > 0 ) {

		// Read character
		char incomingChar = ( char )HWSerialC.read();

		// Look for line return
		if ( incomingChar == '\r' ) {

			// Parse string
			HWSerial_ParseResponseC();

		} else {

			// Build string
			queryResponseC += incomingChar;
		}
	}
}



/****************************
 *  HWSerial_ParseResponse  *
 ****************************/

/**
  * @brief Parse serial response and save values
  * 
  */
void T_AmplifierClass::HWSerial_ParseResponseA() {

	// Remove "> " if it exists
	if ( queryResponseA.startsWith( "v " ) ) {
		queryResponseA.remove( 0, 2 );
	}

	// Extract response
	String response = queryResponseA;

	// GetBaud
	if ( queryStateA == asciiGetBaud ) {
		shared->Amplifier.baudA = response.toInt();
		dataHandle.getData()->PrintDebug( "GetBaud[A]: " + String( shared->Amplifier.baudA ) );

	}
	// SetCurrentMode
	else if ( queryStateA == asciiSetCurrentMode ) {
		dataHandle.getData()->PrintDebug( "SetCurrentMode[A]: " + response );
	}
	// GetName
	else if ( queryStateA == asciiGetName ) {
		shared->Amplifier.nameA = response.substring( 2, queryResponseA.length() );
		dataHandle.getData()->PrintDebug( "GetName[A]: " + shared->Amplifier.nameA );
	}
	// GetEncoderCount
	else if ( queryStateA == asciiGetEncoderCount ) {
		shared->Amplifier.encoderMeasuredCountA = response.toInt();
		// dataHandle.getData()->PrintDebug( "GetCount[A]: " + String( shared->Amplifier.encoderMeasuredCountA ) );
	}
	// GetCurrent
	else if ( queryStateA == asciiGetCurrent ) {
		shared->Amplifier.currentMeasuredRawA = response.toInt();
		// dataHandle.getData()->PrintDebug( "GetCurrent[A]: " + String( shared->Amplifier.currentMeasuredRawA ) );
	}
	// Set encoder zero
	else if ( queryStateA == asciiSetEncoderZero ) {

		dataHandle.getData()->PrintDebug( "SetEncoderZero[A]: " + String( queryResponseA ) );
	}

	queryStateA			= "";
	queryResponseA		= "";
	isAwaitingResponseA = false;
}



/**
  * @brief Parse serial response and save values for amplifier B
  * 
  */
void T_AmplifierClass::HWSerial_ParseResponseB() {

	// Remove "> " if it exists
	if ( queryResponseB.startsWith( "v " ) ) {
		queryResponseB.remove( 0, 2 );
	}

	// Extract response
	String response = queryResponseB;

	// GetBaud
	if ( queryStateB == asciiGetBaud ) {

		shared->Amplifier.baudB = response.toInt();
		dataHandle.getData()->PrintDebug( "GetBaud[B]: " + String( shared->Amplifier.baudB ) );
	}
	// SetCurrentMode
	else if ( queryStateB == asciiSetCurrentMode ) {

		dataHandle.getData()->PrintDebug( "SetCurrentMode[B]: " + response );
	}
	// GetName
	else if ( queryStateB == asciiGetName ) {

		shared->Amplifier.nameB = response.substring( 2, queryResponseB.length() );
		dataHandle.getData()->PrintDebug( "GetName[B]: " + shared->Amplifier.nameB );
	}
	// GetEncoderCount
	else if ( queryStateB == asciiGetEncoderCount ) {

		shared->Amplifier.encoderMeasuredCountB = response.toInt();
		// dataHandle.getData()->PrintDebug( "GetCount[B]: " + String( shared->Amplifier.encoderMeasuredCountB ) );
	}
	// GetCurrent
	else if ( queryStateB == asciiGetCurrent ) {

		shared->Amplifier.currentMeasuredRawB = response.toInt();
		// dataHandle.getData()->PrintDebug( "GetCurrent[B]: " + String( shared->Amplifier.currentMeasuredRawB ) );
	}
	// Set encoder zero
	else if ( queryStateB == asciiSetEncoderZero ) {

		dataHandle.getData()->PrintDebug( "SetEncoderZero[B]: " + String( queryResponseB ) );
	}

	queryStateB			= "";
	queryResponseB		= "";
	isAwaitingResponseB = false;
}



/**
  * @brief Parse serial response and save values for amplifier C
  * 
  */
void T_AmplifierClass::HWSerial_ParseResponseC() {

	// Remove "> " if it exists
	if ( queryResponseC.startsWith( "v " ) ) {
		queryResponseC.remove( 0, 2 );
	}

	// Extract response
	String response = queryResponseC;

	// GetBaud
	if ( queryStateC == asciiGetBaud ) {

		shared->Amplifier.baudC = response.toInt();
		dataHandle.getData()->PrintDebug( "GetBaud[C]: " + String( shared->Amplifier.baudC ) );
	}
	// SetCurrentMode
	else if ( queryStateC == asciiSetCurrentMode ) {

		dataHandle.getData()->PrintDebug( "SetCurrentMode[C]: " + response );
	}
	// GetName
	else if ( queryStateC == asciiGetName ) {

		shared->Amplifier.nameC = response.substring( 2, queryResponseC.length() );
		dataHandle.getData()->PrintDebug( "GetName[C]: " + shared->Amplifier.nameC );
	}
	// GetEncoderCount
	else if ( queryStateC == asciiGetEncoderCount ) {

		shared->Amplifier.encoderMeasuredCountC = response.toInt();
		// dataHandle.getData()->PrintDebug( "GetCount[C]: " + String( shared->Amplifier.encoderMeasuredCountC ) );
	}
	// GetCurrent
	else if ( queryStateC == asciiGetCurrent ) {

		shared->Amplifier.currentMeasuredRawC = response.toInt();
		// dataHandle.getData()->PrintDebug( "GetCurrent[C]: " + String( shared->Amplifier.currentMeasuredRawC ) );
	}
	// Set encoder zero
	else if ( queryStateC == asciiSetEncoderZero ) {

		dataHandle.getData()->PrintDebug( "SetEncoderZero[C]: " + String( queryResponseC ) );
	}


	queryStateC			= "";
	queryResponseC		= "";
	isAwaitingResponseC = false;
}
