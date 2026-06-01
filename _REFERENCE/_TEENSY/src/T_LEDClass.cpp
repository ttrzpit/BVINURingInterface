// Call to class header
#include "T_LEDClass.h"



T_LEDClass::T_LEDClass( SharedDataManager& ctx )
	: dataHandle( ctx )
	, shared( ctx.getData() ) {};


/**
 * @brief Initialize the incoming and outgoing serial ports
 * 
 */
void T_LEDClass::Begin() {

	// Configure pin directions
	pinMode( LED_PIN_READY, OUTPUT );
	pinMode( LED_PIN_DRIVING, OUTPUT );
	pinMode( LED_PIN_MEASURING, OUTPUT );

	// Set initial states
	digitalWriteFast( LED_PIN_READY, LOW );
	digitalWriteFast( LED_PIN_DRIVING, LOW );
	digitalWriteFast( LED_PIN_MEASURING, LOW );
}



/**
 * @brief Main callback function for updating LEDs
 * 
 */
void T_LEDClass::Update() {

	// Set LED states
	digitalWriteFast( LED_PIN_READY, shared->LED.isCommunicatingWithPC );
	digitalWriteFast( LED_PIN_DRIVING, shared->LED.isDrivingMotors );
	digitalWriteFast( LED_PIN_MEASURING, shared->LED.isMeasuringLimits );
}