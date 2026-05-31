// Call to class header
#include "InputClass.h"
#include <chrono>
#include <iomanip>	  // For padding zeroes
#include <thread>

// System data manager
#include "SystemDataManager.h"


// Constructor
InputClass::InputClass( SystemDataManager& ctx )
	: dataHandle( ctx )
	, shared( ctx.getData() ) {

	// Register key bindings
	RegisterKeyBindings();

	std::cout << "Input:   Input initialized.\n";
}


void InputClass::ParseInput( int key ) {

	// Ensure only keyboard input accepted
	if ( key != 255 ) {

		// Uncomment to be able to read detected key presses
		// std::cout << "Input:   Detected key press, index = " << key << "\n";

		// Check if binding present
		if ( keyBindings.count( key ) ) {

			// Run bound function
			keyBindings[key]();
		}
	}
}



// Functions
void InputClass::IncrementValueF( std::string name, float& target, float increment ) {

	if ( ( target + increment ) >= 0 ) {
		// Increment
		target = target + increment;
	}

	// Update
	shared->Display.statusString = "Input: " + name + ( ( increment > 0 ) ? " increased to " : " decreased to " ) + shared->FormatDecimal( target, 1, 2 );
}



/*
 *
 *
 * 
 *
 * 
 * 
 *  
 *  =========================================================================================
 *  ========================================================================================= 
 * 
 *   BBBBBBB    IIIII   NNN   NN  DDDDDDD   IIIII  NNN   NN  GGGGGGG   SSSSS
 *   BB   BBB    III    NNNN  NN  DD   DDD   III   NNNN  NN  GG       SS
 *   BB   BBB    III    NN NN NN  DD   DDD   III   NN NN NN  GG       SS
 *   BBBBBBB     III    NN  NNNN  DD   DDD   III   NN  NNNN  GG  GGG   SSSSS
 *   BB   BBB    III    NN   NNN  DD   DDD   III   NN   NNN  GG   GG       SS
 *   BB   BBB    III    NN   NNN  DD   DDD   III   NN   NNN  GG   GG       SS
 *   BBBBBBB    IIIII   NN   NNN  DDDDDDD   IIIII  NN   NNN   GGGGG   SSSSSS
 * 
 *  ========================================================================================= 
 *  ========================================================================================= 
 */



/** KEY BINDINGS */

void InputClass::RegisterKeyBindings() {
	keyBindings[27]	 = [this]() { K_Exit(); };
	keyBindings['1'] = [this]() { K_SetActive1(); };
	keyBindings['2'] = [this]() { K_SetActive2(); };
	keyBindings['3'] = [this]() { K_SetActive3(); };
	keyBindings['4'] = [this]() { K_SetActive4(); };
	keyBindings['5'] = [this]() { K_SetActive5(); };
	keyBindings['`'] = [this]() { K_SetActiveNone(); };
	keyBindings['a'] = [this]() { K_AmplifierToggle(); };
	keyBindings['t'] = [this]() { K_AmplifierTensionToggle(); };
	keyBindings['s'] = [this]() { K_SerialToggle(); };
	keyBindings[180] = [this]() { K_DirSelect_Abduction(); };
	keyBindings[182] = [this]() { K_DirSelect_Adduction(); };
	keyBindings[178] = [this]() { K_DirSelect_Flexion(); };
	keyBindings[184] = [this]() { K_DirSelect_Extension(); };
	keyBindings['p'] = [this]() { K_GainSelect_Proportional(); };
	keyBindings['i'] = [this]() { K_GainSelect_Integral(); };
	keyBindings['d'] = [this]() { K_GainSelect_Derivative(); };
	keyBindings[174] = [this]() { K_GainsZero(); };
	keyBindings[61]	 = [this]() { K_TenSelect_A(); };
	keyBindings[175] = [this]() { K_TenSelect_B(); };
	keyBindings[170] = [this]() { K_TenSelect_C(); };
	keyBindings[171] = [this]() { K_Increment(); };
	keyBindings[173] = [this]() { K_Decrement(); };
	keyBindings['z'] = [this]() { K_EncoderZero(); };
	keyBindings['x'] = [this]() { K_EncoderMeasureLimit(); };
	keyBindings['c'] = [this]() { K_EncoderSetLimit(); };
	keyBindings['f'] = [this]() { K_FittsStart(); };
	keyBindings['g'] = [this]() { K_FittsStop(); };
	keyBindings[196] = [this]() { K_TaskCalibrationStart(); };
	keyBindings[183] = [this]() { K_LimitsSelectA(); };
	keyBindings[185] = [this]() { K_LimitsSelectB(); };
	keyBindings[181] = [this]() { K_LimitsSelectC(); };
	keyBindings[176] = [this]() { K_LimitsSelectAll(); };
	keyBindings[141] = [this]() { K_LimitsReset(); };
	keyBindings[8]	 = [this]() { K_DeselectAdjustment(); };
	keyBindings['r'] = [this]() { K_SetReverseType(); };
	keyBindings['q'] = [this]() { K_RotateCamera(); };
}



/** 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  =========================================================================================
 *  ========================================================================================= 
 * 
 *   CCCCCCC    AAAA    LL       LL       BBBBBBBB     AAAA     CCCCCCC  KK    KK    SSSSS
 *  CC         AA  AA   LL       LL       BB     BB   AA  AA   CC        KK   KK   SS
 *  CC         AA  AA   LL       LL       BB     BB   AA  AA   CC        KK KK     SS
 *  CC        AAAAAAAA  LL       LL       BBBBBBBB   AAAAAAAA  CC        KKKK        SSSSS
 *  CC        AA    AA  LL       LL       BB     BB  AA    AA  CC        KK KK            SS
 *  CC        AA    AA  LL       LL       BB     BB  AA    AA  CC        KK  KK           SS
 *   CCCCCCC  AA    AA  LLLLLLL  LLLLLLL  BBBBBBBB   AA    AA   CCCCCCC  KK   KK    SSSSSS
 * 
 *  ========================================================================================= 
 *  ========================================================================================= 
 */


/** KEYBOUND FUNCTIONS */

// Exit program
void InputClass::K_Exit() {
	shared->Display.statusString		= "Input: Shutting down...";
	shared->System.state				= stateEnum::IDLE;
	shared->Amplifier.isAmplifierActive = false;
	shared->Serial.isSerialSending		= false;
	shared->Serial.isSerialSendOpen		= false;
	cv::destroyAllWindows();
	std::cout << "Shutdown initiated.\n";
	shared->System.isShuttingDown = true;
}

void InputClass::K_SetActive1() {
	shared->Target.activeID		 = 1;
	shared->Display.statusString = "Input: Updated active marker to #" + shared->Target.activeID;
}

void InputClass::K_SetActive2() {
	shared->Target.activeID		 = 2;
	shared->Display.statusString = "Input: Updated active marker to #" + shared->Target.activeID;
}

void InputClass::K_SetActive3() {
	shared->Target.activeID		 = 3;
	shared->Display.statusString = "Input: Updated active marker to #" + shared->Target.activeID;
}

void InputClass::K_SetActive4() {
	shared->Target.activeID		 = 4;
	shared->Display.statusString = "Input: Updated active marker to #" + shared->Target.activeID;
}

void InputClass::K_SetActive5() {
	shared->Target.activeID		 = 5;
	shared->Display.statusString = "Input: Updated active marker to #" + shared->Target.activeID;
}

void InputClass::K_SetActiveNone() {
	shared->Target.activeID		 = 0;
	shared->Display.statusString = "Input: Updated active marker to #" + shared->Target.activeID;
}

void InputClass::K_AmplifierToggle() {

	// Toggle
	shared->Amplifier.isAmplifierActive = !shared->Amplifier.isAmplifierActive;

	if ( shared->Amplifier.isTensionOnly ) {
		shared->Amplifier.isAmplifierActive = true;
		shared->Amplifier.isTensionOnly		= false;
	}

	if ( shared->Amplifier.isAmplifierActive ) {
		shared->Target.isTargetReset	  = true;
		shared->Controller.isRampingUp	  = true;
		shared->Controller.rampPercentage = 0.0f;
		shared->Controller.rampStartTime  = shared->Timing.elapsedRunningTime;
		shared->System.state			  = stateEnum::DRIVING_PWM;
		shared->Display.statusString	  = "Input: Amplifiers enabled.";
	} else {
		// shared->Serial.packetOut = "DX\n";
		// shared->FLAG_PACKET_WAITING = true;
		shared->System.state		 = stateEnum::IDLE;
		shared->Display.statusString = "Input: Amplifiers disabled.";
	}
}



void InputClass::K_SerialToggle() {

	// Toggle serial flags
	shared->Serial.isSerialSending	 = !shared->Serial.isSerialSending;
	shared->Serial.isSerialReceiving = !shared->Serial.isSerialReceiving;

	if ( shared->Serial.isSerialSending ) {
		shared->Display.statusString	 = "Input: Enabling serial IO.";
		shared->Serial.isSerialSending	 = true;
		shared->Serial.isSerialReceiving = true;
	} else {
		shared->Serial.isSerialSending	 = false;
		shared->Serial.isSerialReceiving = false;
		shared->Display.statusString	 = "Input: Disabling serial IO.";
	}
}


void InputClass::K_DeselectAdjustment() {

	shared->Input.selectedAdjustmentSystem	  = selectSystemEnum::NONE;
	shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
	shared->Display.statusString			  = "Input: Adjustment systems deselected.";
}


// Set type of reversal
void InputClass::K_SetReverseType() {
	shared->Amplifier.isReverseConstant = !shared->Amplifier.isReverseConstant;
	shared->Display.statusString		= ( shared->Amplifier.isReverseConstant ? "Input: Sustained reverse cue activated." : "Input: Vibrating reverse cue activated." );
}

// Rotate camera
void InputClass::K_RotateCamera() {
	shared->Capture.rotateCamera = !shared->Capture.rotateCamera;
}

/*
 *
 *
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  =========================================================================================
 *  ========================================================================================= 
 * 
 *   IIIIII  NN   NN   CCCCCC  RRRRRR   EEEEEEE  MM   MM  EEEEEEE  NN   NN  TTTTTTTT
 *     II    NNN  NN  CC       RR   RR  EE       MMM MMM  EE       NNN  NN     TT
 *     II    NN N NN  CC       RR   RR  EE       MM M MM  EE       NN N NN     TT
 *     II    NN  NNN  CC       RRRRRR   EEEEE    MM   MM  EEEEE    NN  NNN     TT
 *     II    NN   NN  CC       RR  RR   EE       MM   MM  EE       NN   NN     TT
 *     II    NN   NN  CC       RR   RR  EE       MM   MM  EE       NN   NN     TT
 *   IIIIII  NN   NN   CCCCCC  RR   RR  EEEEEEE  MM   MM  EEEEEEE  NN   NN     TT
 * 
 *  ========================================================================================= 
 *  ========================================================================================= 
 */



void InputClass::K_Increment() {

	PrintStates();

	// Shortcut handlers
	auto& system = shared->Input.selectedAdjustmentSystem;

	// No system selected
	if ( system == selectSystemEnum::NONE ) {
		// Deselect subsystems
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
	}
	// Gain system
	else if ( system == selectSystemEnum::GAIN_PROPORTIONAL || system == selectSystemEnum::GAIN_INTEGRAL || system == selectSystemEnum::GAIN_DERIVATIVE ) {
		AdjustGain( 1.0f );
	}
	// Tension system
	else if ( system == selectSystemEnum::AMP_TENSION ) {
		AdjustTension( 1.0f );
	}
	// Power limit system
	else if ( system == selectSystemEnum::AMP_LIMIT ) {
		AdjustLimits( 1.0f );
	}
	// Fallback
	else {
		std::cout << "Input: Increment error!";
	}
}



void InputClass::K_Decrement() {

	PrintStates();

	// Shortcut handlers
	auto& system = shared->Input.selectedAdjustmentSystem;


	// No system selected
	if ( system == selectSystemEnum::NONE ) {

		// Deselect subsystems
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
	}
	// Gain system
	else if ( system == selectSystemEnum::GAIN_PROPORTIONAL || system == selectSystemEnum::GAIN_INTEGRAL || system == selectSystemEnum::GAIN_DERIVATIVE ) {
		AdjustGain( -1.0f );
	}
	// Tension system
	else if ( system == selectSystemEnum::AMP_TENSION ) {

		AdjustTension( -1.0f );
	}
	// Power limit system
	else if ( system == selectSystemEnum::AMP_LIMIT ) {

		AdjustLimits( -1.0f );
	}
	// Fallback
	else {
		std::cout << "Input: System target not understood.\n";
	}
}


void InputClass::AdjustValue( float& target, float step, float min, float max ) {

	// Increment
	if ( step > 0 ) {

		// Check bounds
		if ( ( target + step ) < max ) {
			target += step;
		}

	} else if ( step < 0 ) {

		// Check bounds
		if ( target >= abs( step ) ) {
			target += step;
		}
	}
}



void InputClass::K_EncoderZero() {

	shared->System.state		 = stateEnum::ZERO_ENCODER;
	shared->Display.statusString = "Input: Encoder values set to zero.";
}


void InputClass::K_EncoderMeasureLimit() {


	shared->Amplifier.isMeasuringEncoderLimit = true;
	// shared->Amplifier.isLimitSet			  = false;
	shared->Amplifier.encoderLimitDegA = 0.0f;
	shared->Amplifier.encoderLimitDegB = 0.0f;
	shared->Amplifier.encoderLimitDegC = 0.0f;

	shared->Display.statusString = "Input: Measuring Encoder limits.";
}

void InputClass::K_EncoderSetLimit() {

	shared->Amplifier.isLimitSet			  = true;
	shared->Amplifier.isMeasuringEncoderLimit = false;
	shared->Display.statusString			  = "Input: Encoder limit set.";
}


void InputClass::K_TaskCalibrationStart() {

	// Update state
	shared->Task.state			 = taskEnum::CALIBRATE;
	shared->Display.statusString = "Input: Starting calibration.";
}


void InputClass::K_FittsStart() {
	// Run fitts-law test
	shared->Target.activeID		   = 1;
	shared->Target.isTargetReset   = true;
	shared->Controller.isRampingUp = true;
	shared->Touchscreen.isTouched  = 0;
	shared->Task.isRunning		   = false;
	shared->Task.state			   = taskEnum::FITTS;
	shared->Task.repetitionNumber++;
	shared->Display.statusString = "Input: Starting fitts test.";
}


void InputClass::K_FittsStop() {

	shared->Touchscreen.isTouched = 1;
	// shared->Task.state			  = taskEnum::IDLE;
	shared->Display.statusString = "Input: Ending fitts test.";
}


/*
 *
 *
 * 
 * 
 * 
 * 
 * 
 *  ===========================================
 *  ===========================================
 * 
 *   GGGGGG     AAA    IIIIII  NN   NN   SSSSS
 *  GG         AA AA     II    NNN  NN  SS
 *  GG         AA AA     II    NN N NN  SS
 *  GG   GGG  AAAAAAA    II    NN  NNN   SSSSS
 *  GG    GG  AA   AA    II    NN   NN       SS
 *  GG    GG  AA   AA    II    NN   NN       SS
 *   GGGGGG   AA   AA  IIIIII  NN   NN   SSSSS
 * 
 *  ===========================================
 *  ===========================================
 */



void InputClass::K_GainSelect_Proportional() {

	shared->Input.selectedAdjustmentSystem	  = ( shared->Input.selectedAdjustmentSystem == selectSystemEnum::GAIN_PROPORTIONAL ) ? selectSystemEnum::NONE : selectSystemEnum::GAIN_PROPORTIONAL;
	shared->Input.selectedAdjustmentSubsystem = ( shared->Input.selectedAdjustmentSystem == selectSystemEnum::GAIN_PROPORTIONAL ) ? selectSubsystemEnum::ALL : selectSubsystemEnum::NONE;
	shared->Display.statusString			  = "Input: Selected proportional gain...";
}

void InputClass::K_GainSelect_Integral() {
	shared->Input.selectedAdjustmentSystem	  = ( shared->Input.selectedAdjustmentSystem == selectSystemEnum::GAIN_INTEGRAL ) ? selectSystemEnum::NONE : selectSystemEnum::GAIN_INTEGRAL;
	shared->Input.selectedAdjustmentSubsystem = ( shared->Input.selectedAdjustmentSystem == selectSystemEnum::GAIN_INTEGRAL ) ? selectSubsystemEnum::ALL : selectSubsystemEnum::NONE;
	shared->Display.statusString			  = "Input: Selected integral gain...";
}

void InputClass::K_GainSelect_Derivative() {
	shared->Input.selectedAdjustmentSystem	  = ( shared->Input.selectedAdjustmentSystem == selectSystemEnum::GAIN_DERIVATIVE ) ? selectSystemEnum::NONE : selectSystemEnum::GAIN_DERIVATIVE;
	shared->Input.selectedAdjustmentSubsystem = ( shared->Input.selectedAdjustmentSystem == selectSystemEnum::GAIN_DERIVATIVE ) ? selectSubsystemEnum::ALL : selectSubsystemEnum::NONE;
	shared->Display.statusString			  = "Input: Selected derivative gain...";
}

void InputClass::K_GainsZero() {

	shared->Controller.gainKp.abd = 0.0f;
	shared->Controller.gainKp.add = 0.0f;
	shared->Controller.gainKp.flx = 0.0f;
	shared->Controller.gainKp.ext = 0.0f;
	shared->Controller.gainKi.abd = 0.0f;
	shared->Controller.gainKi.add = 0.0f;
	shared->Controller.gainKi.flx = 0.0f;
	shared->Controller.gainKi.ext = 0.0f;
	shared->Controller.gainKd.abd = 0.0f;
	shared->Controller.gainKd.add = 0.0f;
	shared->Controller.gainKd.flx = 0.0f;
	shared->Controller.gainKd.ext = 0.0f;
	shared->Display.statusString  = "Input: Gains set to zero.";
}

void InputClass::K_DirSelect_Abduction() {

	shared->Input.selectedAdjustmentSubsystem = ( shared->Input.selectedAdjustmentSubsystem == selectSubsystemEnum::ABD ) ? selectSubsystemEnum::NONE : selectSubsystemEnum::ABD;
	shared->Display.statusString			  = "Input: Selected abduction axis...";
}

void InputClass::K_DirSelect_Adduction() {
	shared->Input.selectedAdjustmentSubsystem = ( shared->Input.selectedAdjustmentSubsystem == selectSubsystemEnum::ADD ) ? selectSubsystemEnum::NONE : selectSubsystemEnum::ADD;
	shared->Display.statusString			  = "Input: Selected adduction axis...";
}

void InputClass::K_DirSelect_Flexion() {
	shared->Input.selectedAdjustmentSubsystem = ( shared->Input.selectedAdjustmentSubsystem == selectSubsystemEnum::FLEX ) ? selectSubsystemEnum::NONE : selectSubsystemEnum::FLEX;
	shared->Display.statusString			  = "Input: Selected flexion axis...";
}

void InputClass::K_DirSelect_Extension() {
	shared->Input.selectedAdjustmentSubsystem = ( shared->Input.selectedAdjustmentSubsystem == selectSubsystemEnum::EXT ) ? selectSubsystemEnum::NONE : selectSubsystemEnum::EXT;
	shared->Display.statusString			  = "Input: Selected extension axis...";
}

void InputClass::AdjustGain( float dir ) {

	// Shortcut handlers
	auto& system = shared->Input.selectedAdjustmentSystem;

	// Params
	float step = 0.0f;
	float min  = 0.0f;
	float max  = 1.0f;

	// Select gain system
	switch ( system ) {

		// Adjust proportional
		case ( selectSystemEnum::GAIN_PROPORTIONAL ): {
			std::cout << "Selecting prop\n";
			step = 0.1f * dir;
			min	 = 0.0f;
			max	 = 5.0f;
			AdjustGainP( step, min, max );
			break;
		}

		// Adjust Integral
		case ( selectSystemEnum::GAIN_INTEGRAL ): {
			step = 0.1f * dir;
			min	 = 0.0f;
			max	 = 5.0f;
			AdjustGainI( step, min, max );
			break;
		}

		// Adjust Derivative
		case ( selectSystemEnum::GAIN_DERIVATIVE ): {
			step = 0.05f * dir;
			min	 = 0.0f;
			max	 = 5.0f;
			AdjustGainD( step, min, max );
			break;
		}

		default: {
			std::cout << "Input: Error in AdjustGain!\n";
			break;
		}
	}
}



void InputClass::AdjustGainP( float step, float min, float max ) {

	// Shortcut handlers
	auto& subsystem = shared->Input.selectedAdjustmentSubsystem;
	auto& kp		= shared->Controller.gainKp;

	switch ( subsystem ) {

		// Adjust all
		case ( selectSubsystemEnum::ALL ): {
			std::cout << "all\n";
			AdjustValue( kp.abd, step, min, max );
			AdjustValue( kp.add, step, min, max );
			AdjustValue( kp.flx, step, min, max );
			AdjustValue( kp.ext, step, min, max );
			break;
		}

		// Adjust abd
		case ( selectSubsystemEnum::ABD ): {
			std::cout << "abd\n";
			AdjustValue( kp.abd, step, min, max );
			break;
		}

		// Adjust add
		case ( selectSubsystemEnum::ADD ): {
			AdjustValue( kp.add, step, min, max );
			break;
		}

		// Adjust ext
		case ( selectSubsystemEnum::EXT ): {
			AdjustValue( kp.ext, step, min, max );
			break;
		}

		// Adjust flx
		case ( selectSubsystemEnum::FLEX ): {
			AdjustValue( kp.flx, step, min, max );
			break;
		}

		// Adjust none
		case ( selectSubsystemEnum::NONE ): {
			break;
		}
	}
}



void InputClass::AdjustGainI( float step, float min, float max ) {

	// Shortcut handlers
	auto& subsystem = shared->Input.selectedAdjustmentSubsystem;
	auto& ki		= shared->Controller.gainKi;

	switch ( subsystem ) {

		// Adjust all
		case ( selectSubsystemEnum::ALL ): {
			AdjustValue( ki.abd, step, min, max );
			AdjustValue( ki.add, step, min, max );
			AdjustValue( ki.flx, step, min, max );
			AdjustValue( ki.ext, step, min, max );
			break;
		}

		// Adjust abd
		case ( selectSubsystemEnum::ABD ): {
			AdjustValue( ki.abd, step, min, max );
			break;
		}

		// Adjust add
		case ( selectSubsystemEnum::ADD ): {
			AdjustValue( ki.add, step, min, max );
			break;
		}

		// Adjust ext
		case ( selectSubsystemEnum::EXT ): {
			AdjustValue( ki.ext, step, min, max );
			break;
		}

		// Adjust flx
		case ( selectSubsystemEnum::FLEX ): {
			AdjustValue( ki.flx, step, min, max );
			break;
		}

		// Adjust none
		case ( selectSubsystemEnum::NONE ): {
			break;
		}
	}
}



void InputClass::AdjustGainD( float step, float min, float max ) {

	// Shortcut handlers
	auto& subsystem = shared->Input.selectedAdjustmentSubsystem;
	auto& kd		= shared->Controller.gainKd;

	switch ( subsystem ) {

		// Adjust all
		case ( selectSubsystemEnum::ALL ): {
			AdjustValue( kd.abd, step, min, max );
			AdjustValue( kd.add, step, min, max );
			AdjustValue( kd.flx, step, min, max );
			AdjustValue( kd.ext, step, min, max );
			break;
		}

		// Adjust abd
		case ( selectSubsystemEnum::ABD ): {
			AdjustValue( kd.abd, step, min, max );
			break;
		}

		// Adjust add
		case ( selectSubsystemEnum::ADD ): {
			AdjustValue( kd.add, step, min, max );
			break;
		}

		// Adjust ext
		case ( selectSubsystemEnum::EXT ): {
			AdjustValue( kd.ext, step, min, max );
			break;
		}

		// Adjust flx
		case ( selectSubsystemEnum::FLEX ): {
			AdjustValue( kd.flx, step, min, max );
			break;
		}

		// Adjust none
		case ( selectSubsystemEnum::NONE ): {
			break;
		}
	}
}



/* 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  ===================================================
 *  ==================================================
 * 
 *   LL    IIIIII  MM     MM  IIIIII  TTTTTTT   SSSSS
 *   LL      II    MMM   MMM    II      TT     SS
 *   LL      II    MM MMM MM    II      TT     SS
 *   LL      II    MM  M  MM    II      TT      SSSSS
 *   LL      II    MM     MM    II      TT          SS
 *   LL      II    MM     MM    II      TT          SS
 *   LLLLL IIIIII  MM     MM  IIIIII    TT     SSSSSS
 * 
 *  ==================================================
 *  ==================================================
 */



void InputClass::AdjustLimits( float dir ) {

	// Shortcut handlers
	auto& subsystem = shared->Input.selectedAdjustmentSubsystem;
	auto& amp		= shared->Amplifier.commandedLimits;

	// Params
	float step = 0.01f * dir;
	float min  = 0.0f;
	float max  = 1.0f;

	// Select gain system
	switch ( subsystem ) {

		// Adjust amp A
		case ( selectSubsystemEnum::AMP_A ): {
			AdjustValue( amp.x, step, min, max );
			break;
		}

		// Adjust amp B
		case ( selectSubsystemEnum::AMP_B ): {
			AdjustValue( amp.y, step, min, max );
			break;
		}

		// Adjust amp C
		case ( selectSubsystemEnum::AMP_C ): {
			AdjustValue( amp.z, step, min, max );
			break;
		}

		// Adjust amp C
		case ( selectSubsystemEnum::ALL ): {
			AdjustValue( amp.x, step, min, max );
			AdjustValue( amp.y, step, min, max );
			AdjustValue( amp.z, step, min, max );
			break;
		}

		default: {
			std::cout << "Input: Error in AdjustLimits!\n";
			break;
		}
	}
}

void InputClass::K_LimitsSelectAll() {

	if ( shared->Input.selectedAdjustmentSystem != selectSystemEnum::AMP_LIMIT ) {
		shared->Input.selectedAdjustmentSystem = selectSystemEnum::AMP_LIMIT;
	}

	if ( shared->Input.selectedAdjustmentSubsystem != selectSubsystemEnum::ALL ) {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::ALL;
		shared->Display.statusString			  = "Input: Setting limits for all motors...";
	} else {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
		shared->Input.selectedAdjustmentSystem	  = selectSystemEnum::NONE;
		shared->Display.statusString			  = "Input: Finished limits for Motor A.";
	}
}

void InputClass::K_LimitsSelectA() {

	if ( shared->Input.selectedAdjustmentSystem != selectSystemEnum::AMP_LIMIT ) {
		shared->Input.selectedAdjustmentSystem = selectSystemEnum::AMP_LIMIT;
	}

	if ( shared->Input.selectedAdjustmentSubsystem != selectSubsystemEnum::AMP_A ) {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::AMP_A;
		shared->Display.statusString			  = "Input: Setting limits for Motor A...";
	} else {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
		shared->Input.selectedAdjustmentSystem	  = selectSystemEnum::NONE;
		shared->Display.statusString			  = "Input: Finished limits for Motor A.";
	}
}

void InputClass::K_LimitsSelectB() {

	if ( shared->Input.selectedAdjustmentSystem != selectSystemEnum::AMP_LIMIT ) {
		shared->Input.selectedAdjustmentSystem = selectSystemEnum::AMP_LIMIT;
	}

	if ( shared->Input.selectedAdjustmentSubsystem != selectSubsystemEnum::AMP_B ) {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::AMP_B;
		shared->Display.statusString			  = "Input: Setting limits for Motor B...";
	} else {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
		shared->Input.selectedAdjustmentSystem	  = selectSystemEnum::NONE;
		shared->Display.statusString			  = "Input: Finished limits for Motor B.";
	}
}

void InputClass::K_LimitsSelectC() {

	if ( shared->Input.selectedAdjustmentSystem != selectSystemEnum::AMP_LIMIT ) {
		shared->Input.selectedAdjustmentSystem = selectSystemEnum::AMP_LIMIT;
	}

	if ( shared->Input.selectedAdjustmentSubsystem != selectSubsystemEnum::AMP_C ) {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::AMP_C;
		shared->Display.statusString			  = "Input: Setting limits for Motor C...";
	} else {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
		shared->Input.selectedAdjustmentSystem	  = selectSystemEnum::NONE;
		shared->Display.statusString			  = "Input: Finished limits for Motor C.";
	}
}


void InputClass::K_LimitsReset() {
	shared->Amplifier.isLimitSet	  = false;
	shared->Amplifier.commandedLimits = cv::Point3f( 0.0f, 0.0f, 0.0f );
	shared->Display.statusString	  = "Input: Resetting motor limits.";
}



/*
 *
 *
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  ==============================================================
 *  ==============================================================
 * 
 *   TTTTTTTT  EEEEEEE  NN   NN   SSSSS   IIIIII   OOOOO   NN   NN
 *      TT     EE       NNN  NN  SS         II    OO   OO  NNN  NN
 *      TT     EE       NN N NN  SS         II    OO   OO  NN N NN
 *      TT     EEEEE    NN  NNN   SSSSS     II    OO   OO  NN  NNN
 *      TT     EE       NN   NN       SS    II    OO   OO  NN   NN
 *      TT     EE       NN   NN       SS    II    OO   OO  NN   NN
 *      TT     EEEEEEE  NN   NN   SSSSS   IIIIII   OOOOO   NN   NN
 * 
 *  ==============================================================
 *  ==============================================================
 */



void InputClass::K_TenSelect_A() {

	if ( shared->Input.selectedAdjustmentSystem != selectSystemEnum::AMP_TENSION ) {
		shared->Input.selectedAdjustmentSystem = selectSystemEnum::AMP_TENSION;
	}

	if ( shared->Input.selectedAdjustmentSubsystem != selectSubsystemEnum::AMP_A ) {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::AMP_A;
		shared->Display.statusString			  = "Input: Setting tension for Motor A...";
	} else {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
		shared->Input.selectedAdjustmentSystem	  = selectSystemEnum::NONE;
		shared->Display.statusString			  = "Input: Finished tension for Motor A.";
	}
}


void InputClass::K_TenSelect_B() {

	if ( shared->Input.selectedAdjustmentSystem != selectSystemEnum::AMP_TENSION ) {
		shared->Input.selectedAdjustmentSystem = selectSystemEnum::AMP_TENSION;
	}

	if ( shared->Input.selectedAdjustmentSubsystem != selectSubsystemEnum::AMP_B ) {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::AMP_B;
		shared->Display.statusString			  = "Input: Setting tension for Motor B...";
	} else {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
		shared->Input.selectedAdjustmentSystem	  = selectSystemEnum::NONE;
		shared->Display.statusString			  = "Input: Finished tension for Motor B.";
	}
}

void InputClass::K_TenSelect_C() {

	if ( shared->Input.selectedAdjustmentSystem != selectSystemEnum::AMP_TENSION ) {
		shared->Input.selectedAdjustmentSystem = selectSystemEnum::AMP_TENSION;
	}

	if ( shared->Input.selectedAdjustmentSubsystem != selectSubsystemEnum::AMP_C ) {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::AMP_C;
		shared->Display.statusString			  = "Input: Setting tension for Motor C...";
	} else {
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
		shared->Input.selectedAdjustmentSystem	  = selectSystemEnum::NONE;
		shared->Display.statusString			  = "Input: Finished tension for Motor C.";
	}
}


void InputClass::K_AmplifierTensionToggle() {

	// If tension disabled, enable
	if ( !shared->Amplifier.isTensionOnly ) {

		shared->Amplifier.isTensionOnly			  = true;
		shared->Amplifier.isAmplifierActive		  = true;
		shared->System.state					  = stateEnum::DRIVING_PWM;
		shared->Input.selectedAdjustmentSystem	  = selectSystemEnum::AMP_TENSION;
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::ALL;

	}
	// If tension enabled, disable
	else {

		shared->Amplifier.isTensionOnly			  = false;
		shared->Amplifier.isAmplifierActive		  = false;
		shared->Input.selectedAdjustmentSystem	  = selectSystemEnum::NONE;
		shared->Input.selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
	}

	shared->Display.statusString = "Input: Tension-only is " + std::string( shared->Amplifier.isTensionOnly ? "enabled." : "disabled." );
}


void InputClass::AdjustTension( float dir ) {

	// Shortcut handlers
	auto& subsystem = shared->Input.selectedAdjustmentSubsystem;
	auto& amp		= shared->Controller.commandedTensionABC;

	// Params
	float step = 0.01f * dir;
	float min  = 0.0f;
	float max  = 1.0f;

	// Select gain system
	switch ( subsystem ) {

		// Adjust amp A
		case ( selectSubsystemEnum::AMP_A ): {
			AdjustValue( amp.x, step, min, max );
			break;
		}

		// Adjust amp B
		case ( selectSubsystemEnum::AMP_B ): {
			AdjustValue( amp.y, step, min, max );
			break;
		}

		// Adjust amp C
		case ( selectSubsystemEnum::AMP_C ): {
			AdjustValue( amp.z, step, min, max );
			break;
		}

		// Adjust all amps
		case ( selectSubsystemEnum::ALL ): {

			AdjustValue( amp.x, step, min, max );
			AdjustValue( amp.y, step, min, max );
			AdjustValue( amp.z, step, min, max );
			break;
		}


		default: {
			std::cout << "Input: Error in AdjustTension!\n";
			break;
		}
	}
}



/*
 *
 *
 * 
 * 
 * 
 * 
 * 
 * 
 *  =============================================
 *  =============================================
 * 
 *   PPPPPP    RRRRR    IIIIII  NN   NN  TTTTTTTT
 *   PP    PP  RR   RR    II    NNN  NN     TT
 *   PP    PP  RR   RR    II    NN N NN     TT
 *   PPPPPP    RRRRR      II    NN  NNN     TT
 *   PP        RR  RR     II    NN   NN     TT
 *   PP        RR   RR    II    NN   NN     TT
 *   PP        RR   RR  IIIIII  NN   NN     TT
 * 
 *  =============================================
 *  =============================================
 */



void InputClass::PrintStates() {

	// Handle
	auto& system	= shared->Input.selectedAdjustmentSystem;
	auto& subsystem = shared->Input.selectedAdjustmentSubsystem;

	// System state
	if ( system == selectSystemEnum::NONE ) {
		std::cout << "System::NONE";
	} else if ( system == selectSystemEnum::GAIN_PROPORTIONAL ) {
		std::cout << "System::GAIN_PROPORTIONAL";
	} else if ( system == selectSystemEnum::GAIN_INTEGRAL ) {
		std::cout << "System::GAIN_INTEGRAL";
	} else if ( system == selectSystemEnum::GAIN_DERIVATIVE ) {
		std::cout << "System::GAIN_DERIVATIVE";
	} else if ( system == selectSystemEnum::AMP_TENSION ) {
		std::cout << "System::AMP_TENSION";
	} else if ( system == selectSystemEnum::AMP_LIMIT ) {
		std::cout << "System::AMP_LIMIT";
	} else {
		std::cout << "Input: System error!";
	}

	std::cout << "\t";

	// Subsystem state
	if ( subsystem == selectSubsystemEnum::NONE ) {
		std::cout << "Subsystem::NONE";
	} else if ( subsystem == selectSubsystemEnum::ABD ) {
		std::cout << "Subsystem::ABD";
	} else if ( subsystem == selectSubsystemEnum::ADD ) {
		std::cout << "Subsystem::ADD";
	} else if ( subsystem == selectSubsystemEnum::EXT ) {
		std::cout << "Subsystem::EXT";
	} else if ( subsystem == selectSubsystemEnum::FLEX ) {
		std::cout << "Subsystem::FLX";
	} else if ( subsystem == selectSubsystemEnum::ALL ) {
		std::cout << "Subsystem::ALL";
	} else if ( subsystem == selectSubsystemEnum::AMP_A ) {
		std::cout << "Subsystem::AMP_A";
	} else if ( subsystem == selectSubsystemEnum::AMP_B ) {
		std::cout << "Subsystem::AMP_B";
	} else if ( subsystem == selectSubsystemEnum::AMP_C ) {
		std::cout << "Subsystem::AMP_C";
	} else {
		std::cout << "Input: Subsystem error!";
	}

	std::cout << "\n";
}



/*
 * 
 *
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *  =========================================================================================
 *  ========================================================================================= 
 * 
 *   OOOOO   LL      DDDDDD
 *  OO   OO  LL      DD    DD
 *  OO   OO  LL      DD    DD
 *  OO   OO  LL      DD    DD
 *  OO   OO  LL      DD    DD
 *  OO   OO  LL      DD    DD
 *   OOOOO   LLLLLL  DDDDDD
 * 
 *  ========================================================================================= 
 *  ========================================================================================= 
 */



// void InputClass::K_Increment() {

// 	// Handler for gains
// 	auto& gains = shared->Controller;

// 	// Increment value
// 	float increment = 0.0f;

// 	// Select appropriate increment for PID gains
// 	if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 		increment = 0.1f;

// 	} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 		increment = 0.1f;

// 	} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 		increment = 0.01f;

// 	} else if ( shared->Input.selectGain == selectGainEnum::NONE ) {
// 		increment = 0.0f;

// 	} else {
// 		increment = 0.01f;
// 	}

// 	// Select appropriate increment for tension gains
// 	if ( shared->Input.selectGainTarget == selectGainTargetEnum::AMPA || shared->Input.selectGainTarget == selectGainTargetEnum::AMPB || shared->Input.selectGainTarget == selectGainTargetEnum::AMPC ) {
// 		increment = 0.01;
// 	}

// 	switch ( shared->Input.selectGainTarget ) {

// 		// Abduction
// 		case selectGainTargetEnum::ABD: {

// 			if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 				gains.gainKp.abd += increment;
// 				shared->Display.statusString = "Input: Abduction Kp increased to " + shared->FormatDecimal( shared->Controller.gainKp.abd, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 				gains.gainKi.abd += increment;
// 				shared->Display.statusString = "Input: Abduction Ki increased to " + shared->FormatDecimal( shared->Controller.gainKi.abd, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 				gains.gainKd.abd += increment;
// 				shared->Display.statusString = "Input: Abduction Kd increased to " + shared->FormatDecimal( shared->Controller.gainKd.abd, 2, 2 );
// 			}
// 			break;
// 		}

// 		// Adduction
// 		case selectGainTargetEnum::ADD: {

// 			if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 				gains.gainKp.add += increment;
// 				shared->Display.statusString = "Input: Adduction Kp increased to " + shared->FormatDecimal( shared->Controller.gainKp.add, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 				gains.gainKi.add += increment;
// 				shared->Display.statusString = "Input: Adduction Ki increased to " + shared->FormatDecimal( shared->Controller.gainKi.add, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 				gains.gainKd.add += increment;
// 				shared->Display.statusString = "Input: Adduction Kd increased to " + shared->FormatDecimal( shared->Controller.gainKd.add, 2, 2 );
// 			}
// 			break;
// 		}

// 		// Flexion
// 		case selectGainTargetEnum::FLEX: {

// 			if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 				gains.gainKp.flx += increment;
// 				shared->Display.statusString = "Input: Flexion Kp increased to " + shared->FormatDecimal( shared->Controller.gainKp.flx, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 				gains.gainKi.flx += increment;
// 				shared->Display.statusString = "Input: Flexion Ki increased to " + shared->FormatDecimal( shared->Controller.gainKi.flx, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 				gains.gainKd.flx += increment;
// 				shared->Display.statusString = "Input: Flexion Kd increased to " + shared->FormatDecimal( shared->Controller.gainKd.flx, 2, 2 );
// 			}
// 			break;
// 		}

// 		// Extension
// 		case selectGainTargetEnum::EXT: {

// 			if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 				gains.gainKp.ext += increment;
// 				shared->Display.statusString = "Input: Extension Kp increased to " + shared->FormatDecimal( shared->Controller.gainKp.ext, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 				gains.gainKi.ext += increment;
// 				shared->Display.statusString = "Input: Extension Ki increased to " + shared->FormatDecimal( shared->Controller.gainKi.ext, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 				gains.gainKd.ext += increment;
// 				shared->Display.statusString = "Input: Extension Kd increased to " + shared->FormatDecimal( shared->Controller.gainKd.ext, 2, 2 );
// 			}
// 			break;
// 		}


// 		// Amp A
// 		case selectGainTargetEnum::AMPA: {

// 			gains.commandedTensionABC.x += increment;
// 			shared->Display.statusString = "Input: Tension A increased to " + shared->FormatDecimal( shared->Controller.commandedTensionABC.x, 2, 2 );

// 			break;
// 		}

// 		// Amp B
// 		case selectGainTargetEnum::AMPB: {

// 			gains.commandedTensionABC.y += increment;
// 			shared->Display.statusString = "Input: Tension B increased to " + shared->FormatDecimal( shared->Controller.commandedTensionABC.y, 2, 2 );

// 			break;
// 		}

// 		// Amp C
// 		case selectGainTargetEnum::AMPC: {

// 			gains.commandedTensionABC.z += increment;
// 			shared->Display.statusString = "Input: Tension C increased to " + shared->FormatDecimal( shared->Controller.commandedTensionABC.z, 2, 2 );

// 			break;
// 		}

// 		// Torque limits
// 		case selectGainTargetEnum::TORQUE: {

// 			// Deselect active marker
// 			shared->Target.activeID = 0;

// 			// Pick direction
// 			switch ( shared->Input.selectTorqueTarget ) {

// 				// Abduction (left)
// 				case selectTorqueTargetEnum::ABD: {

// 					// Increment virtual error
// 					shared->Target.positionFilteredNewMM.x -= increment;

// 					if ( shared->Controller.commandedPercentageABC.y > shared->Amplifier.commandedLimits.y ) {
// 						shared->Amplifier.commandedLimits.y = shared->Controller.commandedPercentageABC.y;
// 					}

// 					if ( shared->Controller.commandedPercentageABC.z > shared->Amplifier.commandedLimits.z ) {
// 						shared->Amplifier.commandedLimits.z = shared->Controller.commandedPercentageABC.z;
// 					}

// 					break;
// 				}
// 			}
// 		}

// 		// Amplifier Limits
// 		case selectGainTargetEnum::LIMITS: {

// 			switch ( shared->Input.selectLimit ) {

// 				// Amplifier A
// 				case selectLimitEnum::AMP_A: {

// 					// Only increment if under limit
// 					if ( shared->Amplifier.commandedLimits.x < 0.99f ) {
// 						shared->Amplifier.commandedLimits.x += 0.01f;
// 						shared->Display.statusString = "Input: Motor A max power increased to " + shared->FormatDecimal( shared->Amplifier.commandedLimits.x, 2, 2 );
// 					}

// 					break;
// 				}

// 				// Amplifier B
// 				case selectLimitEnum::AMP_B: {

// 					// Only increment if under limit
// 					if ( shared->Amplifier.commandedLimits.y < 0.99f ) {
// 						shared->Amplifier.commandedLimits.y += 0.01f;
// 						shared->Display.statusString = "Input: Motor B max power increased to " + shared->FormatDecimal( shared->Amplifier.commandedLimits.y, 2, 2 );
// 					}

// 					break;
// 				}

// 				// Amplifier C
// 				case selectLimitEnum::AMP_C: {

// 					// Only increment if under limit
// 					if ( shared->Amplifier.commandedLimits.z < 0.99f ) {
// 						shared->Amplifier.commandedLimits.z += 0.01f;
// 						shared->Display.statusString = "Input: Motor C max power increased to " + shared->FormatDecimal( shared->Amplifier.commandedLimits.z, 2, 2 );
// 					}

// 					break;
// 				}
// 			}


// 			break;
// 		}

// 		// None
// 		case selectGainTargetEnum::NONE: {

// 			if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 				gains.gainKp.abd += increment;
// 				gains.gainKp.add += increment;
// 				gains.gainKp.flx += increment;
// 				gains.gainKp.ext += increment;
// 				shared->Display.statusString = "Input: Kp increased.";

// 			} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 				gains.gainKi.abd += increment;
// 				gains.gainKi.add += increment;
// 				gains.gainKi.flx += increment;
// 				gains.gainKi.ext += increment;
// 				shared->Display.statusString = "Input: Ki increased.";

// 			} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 				gains.gainKd.abd += increment;
// 				gains.gainKd.add += increment;
// 				gains.gainKd.flx += increment;
// 				gains.gainKd.ext += increment;
// 				shared->Display.statusString = "Input: Kd increased.";
// 			}
// 			break;
// 		}

// 		default: {

// 			// Shouldn't be here...
// 			break;
// 		}
// 	}
// // }

// void InputClass::K_Decrement() {

// 	// Handler for gains
// 	auto& gains = shared->Controller;

// 	// Increment value
// 	float increment = 0.0f;

// 	// Select appropriate increment for PID gains
// 	if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 		increment = 0.1f;

// 	} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 		increment = 0.1f;

// 	} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 		increment = 0.01f;

// 	} else if ( shared->Input.selectGain == selectGainEnum::NONE ) {
// 		increment = 0.0f;

// 	} else {
// 		increment = 0.0f;
// 	}

// 	// Select appropriate increment for tension gains
// 	if ( shared->Input.selectGainTarget == selectGainTargetEnum::AMPA || shared->Input.selectGainTarget == selectGainTargetEnum::AMPB || shared->Input.selectGainTarget == selectGainTargetEnum::AMPC ) {
// 		increment = 0.01;
// 	}

// 	increment = increment * -1.0f;

// 	switch ( shared->Input.selectGainTarget ) {

// 		// Abduction
// 		case selectGainTargetEnum::ABD: {

// 			if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 				SafeDecrement( gains.gainKp.abd, increment );
// 				shared->Display.statusString = "Input: Abduction Kp increased to " + shared->FormatDecimal( shared->Controller.gainKp.abd, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 				SafeDecrement( gains.gainKi.abd, increment );
// 				shared->Display.statusString = "Input: Abduction Ki increased to " + shared->FormatDecimal( shared->Controller.gainKi.abd, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 				SafeDecrement( gains.gainKd.abd, increment );
// 				shared->Display.statusString = "Input: Abduction Kd increased to " + shared->FormatDecimal( shared->Controller.gainKd.abd, 2, 2 );
// 			}
// 			break;
// 		}

// 		// Adduction
// 		case selectGainTargetEnum::ADD: {

// 			if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 				SafeDecrement( gains.gainKp.add, increment );
// 				shared->Display.statusString = "Input: Adduction Kp increased to " + shared->FormatDecimal( shared->Controller.gainKp.add, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 				SafeDecrement( gains.gainKi.add, increment );
// 				shared->Display.statusString = "Input: Adduction Ki increased to " + shared->FormatDecimal( shared->Controller.gainKi.add, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 				SafeDecrement( gains.gainKd.add, increment );
// 				shared->Display.statusString = "Input: Adduction Kd increased to " + shared->FormatDecimal( shared->Controller.gainKd.add, 2, 2 );
// 			}
// 			break;
// 		}

// 		// Flexion
// 		case selectGainTargetEnum::FLEX: {

// 			if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 				SafeDecrement( gains.gainKp.flx, increment );
// 				shared->Display.statusString = "Input: Flexion Kp increased to " + shared->FormatDecimal( shared->Controller.gainKp.flx, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 				SafeDecrement( gains.gainKi.flx, increment );
// 				shared->Display.statusString = "Input: Flexion Ki increased to " + shared->FormatDecimal( shared->Controller.gainKi.flx, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 				SafeDecrement( gains.gainKd.flx, increment );
// 				shared->Display.statusString = "Input: Flexion Kd increased to " + shared->FormatDecimal( shared->Controller.gainKd.flx, 2, 2 );
// 			}
// 			break;
// 		}

// 		// Extension
// 		case selectGainTargetEnum::EXT: {

// 			if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 				SafeDecrement( gains.gainKp.ext, increment );
// 				shared->Display.statusString = "Input: Extension Kp increased to " + shared->FormatDecimal( shared->Controller.gainKp.ext, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 				SafeDecrement( gains.gainKi.ext, increment );
// 				shared->Display.statusString = "Input: Extension Ki increased to " + shared->FormatDecimal( shared->Controller.gainKi.ext, 2, 2 );

// 			} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 				SafeDecrement( gains.gainKd.ext, increment );
// 				shared->Display.statusString = "Input: Extension Kd increased to " + shared->FormatDecimal( shared->Controller.gainKd.ext, 2, 2 );
// 			}
// 			break;
// 		}


// 		// Amp A
// 		case selectGainTargetEnum::AMPA: {

// 			SafeDecrement( gains.commandedTensionABC.x, increment );
// 			// gains.commandedTensionABC.x += increment;
// 			shared->Display.statusString = "Input: Tension A increased to " + shared->FormatDecimal( shared->Controller.commandedTensionABC.x, 2, 2 );

// 			break;
// 		}

// 		// Amp B
// 		case selectGainTargetEnum::AMPB: {

// 			SafeDecrement( gains.commandedTensionABC.y, increment );
// 			shared->Display.statusString = "Input: Tension B increased to " + shared->FormatDecimal( shared->Controller.commandedTensionABC.y, 2, 2 );

// 			break;
// 		}

// 		// Amp C
// 		case selectGainTargetEnum::AMPC: {

// 			SafeDecrement( gains.commandedTensionABC.z, increment );
// 			shared->Display.statusString = "Input: Tension C increased to " + shared->FormatDecimal( shared->Controller.commandedTensionABC.z, 2, 2 );

// 			break;
// 		}



// 		// Amplifier Limits
// 		case selectGainTargetEnum::LIMITS: {

// 			switch ( shared->Input.selectLimit ) {

// 				// Amplifier A
// 				case selectLimitEnum::AMP_A: {

// 					// Only decrement if under limit
// 					if ( shared->Amplifier.commandedLimits.x > 0.01f ) {
// 						SafeDecrement( shared->Amplifier.commandedLimits.x, -0.01f );
// 						shared->Display.statusString = "Input: Motor A max power decreased to " + shared->FormatDecimal( shared->Amplifier.commandedLimits.x, 2, 2 );
// 					}

// 					break;
// 				}

// 				// Amplifier B
// 				case selectLimitEnum::AMP_B: {

// 					// Only decrement if under limit
// 					if ( shared->Amplifier.commandedLimits.y > 0.01f ) {
// 						SafeDecrement( shared->Amplifier.commandedLimits.y, -0.01f );
// 						shared->Display.statusString = "Input: Motor A max power decreased to " + shared->FormatDecimal( shared->Amplifier.commandedLimits.x, 2, 2 );
// 					}

// 					break;
// 				}

// 				// Amplifier C
// 				case selectLimitEnum::AMP_C: {

// 					// Only decrement if under limit
// 					if ( shared->Amplifier.commandedLimits.z > 0.01f ) {
// 						SafeDecrement( shared->Amplifier.commandedLimits.z, -0.01f );
// 						shared->Display.statusString = "Input: Motor A max power decreased to " + shared->FormatDecimal( shared->Amplifier.commandedLimits.x, 2, 2 );
// 					}

// 					break;
// 				}
// 			}
// 		}



// 		// None
// 		case selectGainTargetEnum::NONE: {

// 			if ( shared->Input.selectGain == selectGainEnum::KP ) {
// 				SafeDecrement( gains.gainKp.add, increment );
// 				SafeDecrement( gains.gainKp.abd, increment );
// 				SafeDecrement( gains.gainKp.flx, increment );
// 				SafeDecrement( gains.gainKp.ext, increment );
// 				shared->Display.statusString = "Input: Kp decreased.";

// 			} else if ( shared->Input.selectGain == selectGainEnum::KI ) {
// 				SafeDecrement( gains.gainKi.add, increment );
// 				SafeDecrement( gains.gainKi.abd, increment );
// 				SafeDecrement( gains.gainKi.flx, increment );
// 				SafeDecrement( gains.gainKi.ext, increment );
// 				shared->Display.statusString = "Input: Ki decreased.";

// 			} else if ( shared->Input.selectGain == selectGainEnum::KD ) {
// 				SafeDecrement( gains.gainKd.add, increment );
// 				SafeDecrement( gains.gainKd.abd, increment );
// 				SafeDecrement( gains.gainKd.flx, increment );
// 				SafeDecrement( gains.gainKd.ext, increment );
// 				shared->Display.statusString = "Input: Kd decreased.";
// 			}
// 			break;
// 		}


// 		default: {

// 			// Shouldn't be here...
// 			break;
// 		}
// 	}
// }