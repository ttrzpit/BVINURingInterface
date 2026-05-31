// Library for managing interrupt signals
#include <csignal>

// Configuration files
#include "include/config.h"

// Data manager for exclusive read/write
#include "include/SystemDataManager.h"
SystemDataManager dataHandle;						// System data manager object handle
auto			  shared = dataHandle.getData();	// Handle to shared data

// Custom class objects
#include "include/ArucoClass.h"
#include "include/CaptureClass.h"
#include "include/ControllerClass.h"
#include "include/DisplayClass.h"
#include "include/InputClass.h"
#include "include/KalmanClass.h"
#include "include/LoggingClass.h"
#include "include/SerialClass.h"
#include "include/TasksClass.h"
#include "include/TimingClass.h"
#include "include/TouchscreenClass.h"



/**
 * @class ArucoClass
 * @brief Detects ArUco markers in the captured frames.
 * 
 * This class processes frames captured by CaptureClass to detect
 * ArUco markers and updates the system state accordingly.
 */

/**
 * @class TasksClass
 * @brief Manages the execution of system tasks.
 * 
 * This class coordinates the execution of various tasks, such as
 * calibration, limit measurement, and Fitts' law experiments.
 * 
 * @note Depends on TimingClass and LoggingClass.
 */


// New class objects
CaptureClass	 Capture( dataHandle );					  // Camera capture
ArucoClass		 Aruco( dataHandle );					  // Aruco detector
DisplayClass	 Canvas( dataHandle );					  // Display output
InputClass		 Input( dataHandle );					  // Keyboard input
TimingClass		 Timing( dataHandle );					  // Loop timing measurement
TouchscreenClass Touch( dataHandle );					  // Touchscreen position reading
SerialClass		 Serial( dataHandle, 2 );				  // Serial interface
LoggingClass	 Logging( dataHandle );					  // Logging interface
ControllerClass	 Controller( dataHandle );				  // Controller
KalmanClass		 Kalman( dataHandle );					  // Kalman filter
TasksClass		 Tasks( dataHandle, Timing, Logging );	  // Tasks interface



// Function prototypes
void SignalHandler( int signum );
void UpdateSystem();
void SelectTask();
void UpdateState();
void PrintState();



// SYSTEM FLAGS
bool FLAG_PrintOpenCVBuildInfo = false;
bool FLAG_PrintState		   = false;



/**
 * @brief Main program loop
 * 
 * @return int 
 */
int main() {

	// Reverse type
	shared->Amplifier.isReverseConstant = false;	// ( constant = 1 )


	// Testing
	shared->Amplifier.commandedLimits = cv::Point3f( 0.2f, 0.2f, 0.2f );
	shared->Controller.isLimitSet	  = true;
	shared->Calibration.isCalibrated  = true;

	// Settings
	shared->Logging.isEnabled	 = false;
	shared->Task.userID			 = 123;
	shared->Target.isTargetFound = 1;

	shared->System.useRing = false;

	// Optimization for C/C++ data types
	std::ios_base::sync_with_stdio( false );
	std::cout.setf( std::ios::unitbuf );

	// For debugging
	if ( FLAG_PrintOpenCVBuildInfo ) {
		std::cout << "OpenCV Build: " << cv::getBuildInformation() << std::endl;
		cv::utils::logging::setLogLevel( cv::utils::logging::LOG_LEVEL_VERBOSE );
	}

	// Status update
	std::cout << "\nMain:         Program running...\n\n";

	// Start timer for measuring loop frequency
	Timing.StartTimer();

	// Add shortcut panel
	// Canvas.BuildKeyboardShortcuts();
	Canvas.AddStaticDisplayPanels();

	// Initialize kalman filter
	shared->KalmanFilter.pMatrix = cv::Mat::eye( 6, 6, CV_32F ) * 1.0f;

	// Set default system state
	shared->System.state = stateEnum::IDLE;

	// Main loop
	while ( shared->System.isMainRunning ) {

		// For debugging
		if ( FLAG_PrintState ) {
			PrintState();
		}

		// Update timer (for measuring loop frequency)
		Timing.Update();

		// Parse any input and use OpenCV WaitKey()
		Input.ParseInput( cv::pollKey() & 0xFF );

		// Capture frame
		Capture.GetFrame();

		// Run the appropriate task
		SelectTask();

		// Detect ArUco tags
		Aruco.FindTags();

		// Update system
		UpdateSystem();

		// Update controller
		Controller.Update();
		Controller.UpdateAmplifier();
		Controller.UpdateVibrotactile();

		// Check touchscreen input
		Touch.GetCursorPosition();

		// Update serial messages
		Serial.Update();

		// Update display
		Canvas.Update();

		// Update shutdown flags for clean shutdown
		if ( shared->System.isShuttingDown ) {
			shared->System.isMainRunning = false;
		}
	}
	cv::destroyAllWindows();
	return 0;
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
 *  =========================================================================================
 *  ========================================================================================= 
 * 
 *   TTTTTTT   AAAA   SSSSS  KK   KK  SSSSS
 *     T      AA  AA SS      KK  KK  SS
 *     T      AA  AA  SS     KK KK    SS
 *     T     AAAAAAAA  SSSSS  KKKK     SSSSS
 *     T     AA    AA      SS KK KK         SS
 *     T     AA    AA      SS KK  KK        SS
 *     T     AA    AA  SSSSS  KK   KK  SSSSSS
 * 
 *  ========================================================================================= 
 *  ========================================================================================= 
 */



/**
 * @brief State machine for running tasks
 * 
 */
void SelectTask() {

	switch ( shared->Task.state ) {

		case taskEnum::IDLE: {

			// Defaults
			shared->Task.command = 'z';
			shared->Task.name	 = "";

			// Do nothing
			break;
		}

		case taskEnum::FITTS: {

			/** Select how to generate marker
			 * 		x = random x position
			 * 		y = random y position
			 * 		z = random xy position
			 * 		v = constant velocity
			 */
			shared->Task.command = 'z';

			// Update name and start
			shared->Task.name = "FITTS";
			Tasks.Fitts();

			break;
		}

		case taskEnum::CALIBRATE: {

			shared->Task.name = "CALIBRATE";
			Tasks.Calibration();

			break;
		}

		case taskEnum::LIMIT: {
			shared->Task.name = "LIMITS";
			Tasks.Limits();
			break;
		}

		default: {

			// Defaults
			shared->Task.command = 'z';
			shared->Task.name	 = "";
		}
	}
}


/**
 * @brief Update system state using kalman filter
 * 
 */
void UpdateSystem() {

	if ( shared->Target.isTargetFound && shared->Capture.isFrameReady ) {

		if ( cv::norm( cv::Point2f( shared->Target.positionUnfilteredMM.x, shared->Target.positionUnfilteredMM.y ) - cv::Point2f( shared->Target.positionFilteredOldMM.x, shared->Target.positionFilteredOldMM.y ) ) > 100.0f ) {
			shared->Target.isTargetReset = true;
		}


		// Save old data
		shared->Target.positionFilteredOldMM = shared->Target.positionFilteredNewMM;
		shared->Target.velocityFilteredOldMM = shared->Target.velocityFilteredNewMM;

		// Update kalman filter
		Kalman.Update( shared->Target.positionUnfilteredMM, shared->Timing.elapsedRunningTime );

		// Get updated state values
		shared->Target.positionFilteredNewMM = Kalman.GetPosition();
		shared->Target.velocityFilteredNewMM = Kalman.GetVelocity();
		shared->Target.positionIntegratedMM	 = Kalman.GetIntegralError();

		// Update marker if finger calibrated
		if ( shared->Calibration.isCalibrated ) {

			// Offset based on calibrated touch
		}
	}
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
 * 
 * 
 * 
 */

void PrintState() {

	// enum class stateEnum { IDLE, DRIVING_PWM, MEASURING_LIMITS, MEASURING_CURRENT, ZERO_ENCODER };

	std::cout << "State tracker: ";

	// System
	if ( shared->System.state == stateEnum::IDLE ) {
		std::cout << "System::IDLE";
	} else if ( shared->System.state == stateEnum::DRIVING_PWM ) {
		std::cout << "System::DRIVING_PWM";
	} else if ( shared->System.state == stateEnum::MEASURING_LIMITS ) {
		std::cout << "System::MEASURING_LIMITS";
	} else if ( shared->System.state == stateEnum::ZERO_ENCODER ) {
		std::cout << "System::ZERO_ENCODER";
	} else {
		std::cout << "System::ERROR!";
	}

	std::cout << "\t ";

	// Task
	if ( shared->Task.state == taskEnum::IDLE ) {
		std::cout << "Task::IDLE";
	} else if ( shared->Task.state == taskEnum::CALIBRATE ) {
		std::cout << "Task::CALIBRATE";
	} else if ( shared->Task.state == taskEnum::FITTS ) {
		std::cout << "Task::FITTS";
	} else if ( shared->Task.state == taskEnum::LIMIT ) {
		std::cout << "Task::LIMIT";
	} else {
		std::cout << "Task::ERROR!";
	}

	std::cout << "\t ";

	// Adjustment system selection
	auto& system = shared->Input.selectedAdjustmentSystem;

	if ( system == selectSystemEnum::NONE ) {
		std::cout << "SelectSystem::NONE";
	} else if ( system == selectSystemEnum::GAIN_PROPORTIONAL ) {
		std::cout << "SelectSystem::GAIN_PROPORTIONAL";
	} else if ( system == selectSystemEnum::GAIN_INTEGRAL ) {
		std::cout << "SelectSystem::GAIN_INTEGRAL";
	} else if ( system == selectSystemEnum::GAIN_DERIVATIVE ) {
		std::cout << "SelectSystem::GAIN_DERIVATIVE";
	} else if ( system == selectSystemEnum::AMP_LIMIT ) {
		std::cout << "SelectSystem::AMP_LIMIT";
	} else if ( system == selectSystemEnum::AMP_TENSION ) {
		std::cout << "SelectSystem::AMP_TENSION";
	} else {
		std::cout << "SelectSystem::ERROR!";
	}

	std::cout << "\t ";

	// Adjustment subsystem selection
	auto& subsystem = shared->Input.selectedAdjustmentSubsystem;

	if ( subsystem == selectSubsystemEnum::NONE ) {
		std::cout << "SelectSubsystem::NONE";
	} else if ( subsystem == selectSubsystemEnum::ABD ) {
		std::cout << "SelectSubsystem::ABD";
	} else if ( subsystem == selectSubsystemEnum::ADD ) {
		std::cout << "SelectSubsystem::ADD";
	} else if ( subsystem == selectSubsystemEnum::FLEX ) {
		std::cout << "SelectSubsystem::FLEX";
	} else if ( subsystem == selectSubsystemEnum::EXT ) {
		std::cout << "SelectSubsystem::EXT";
	} else if ( subsystem == selectSubsystemEnum::ALL ) {
		std::cout << "SelectSubsystem::ALL";
	} else if ( subsystem == selectSubsystemEnum::AMP_A ) {
		std::cout << "SelectSubsystem::AMP_A";
	} else if ( subsystem == selectSubsystemEnum::AMP_B ) {
		std::cout << "SelectSubsystem::AMP_B";
	} else if ( subsystem == selectSubsystemEnum::AMP_C ) {
		std::cout << "SelectSubsystem::AMP_C";
	} else

		std::cout << "\t ";



	std::cout << "\t ";

	// Torque target
	if ( shared->Input.selectLimit == selectLimitEnum::NONE ) {
		std::cout << "SelectLimit::NONE";
	} else if ( shared->Input.selectLimit == selectLimitEnum::AMP_A ) {
		std::cout << "SelectLimit::AMP_A";
	} else if ( shared->Input.selectLimit == selectLimitEnum::AMP_B ) {
		std::cout << "SelectLimit::AMP_B";
	} else if ( shared->Input.selectLimit == selectLimitEnum::AMP_C ) {
		std::cout << "SelectLimit::AMP_C";
	}

	std::cout << "\n";
}


// enum class stateEnum { IDLE, DRIVING_PWM, MEASURING_LIMITS, ZERO_ENCODER };
// enum class taskEnum { IDLE, CALIBRATE, FITTS, LIMIT };
// enum class selectGainTargetEnum { NONE, ABD, ADD, FLEX, EXT, AMPA, AMPB, AMPC, TORQUE, LIMITS };
// enum class selectGainEnum { NONE, KP, KI, KD };
// enum class selectTorqueTargetEnum { NONE, ABD, ADD, FLEX, EXT };
// enum class selectLimitEnum { NONE, AMP_A, AMP_B, AMP_C };

/**
 * @brief Shutdown safely if error occurs
 */
void SignalHandler( int signum ) {
	std::cout << "\nMain:         Interrupt signal (" << signum << ") received.\n";
	shared->System.isMainRunning = false;
	Touch.Close();
	exit( signum );
}
