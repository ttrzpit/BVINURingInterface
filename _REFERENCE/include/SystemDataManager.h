#pragma once

// Necessary libraries
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

// OpenCV libraries
#include <ArucoClass.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/cudaimgproc.hpp>

// Runtime configuration
#include "config.h"

// Packet
#include "PacketTypes.h"

// Constants
#define RAD2DEG 57.2958
#define DEG2RAD 0.01745

// Enum for system state
enum class stateEnum { IDLE, DRIVING_PWM, MEASURING_LIMITS, ZERO_ENCODER };
// enum class selectGainTargetEnum { NONE, ABD, ADD, FLEX, EXT, AMPA, AMPB, AMPC, TORQUE, LIMITS };
// enum class selectGainEnum { NONE, KP, KI, KD };
enum class selectTorqueTargetEnum { NONE, ABD, ADD, FLEX, EXT };
enum class taskEnum { IDLE, CALIBRATE, FITTS, LIMIT };
enum class selectLimitEnum { NONE, AMP_A, AMP_B, AMP_C };

enum class selectSystemEnum { NONE, GAIN_PROPORTIONAL, GAIN_INTEGRAL, GAIN_DERIVATIVE, AMP_TENSION, AMP_LIMIT };
enum class selectSubsystemEnum { NONE, ALL, ABD, ADD, EXT, FLEX, AMP_A, AMP_B, AMP_C };

// Structure for 4-point vector
struct Point4f {
	float abd = 0.0f;
	float add = 0.0f;
	float flx = 0.0f;
	float ext = 0.0f;
};



struct AmplifierStruct {

	// Flags
	bool isAmplifierActive		 = false;
	bool isMeasuringEncoderLimit = false;
	bool isLimitSet				 = false;
	bool isOverLimitA			 = false;
	bool isOverLimitB			 = false;
	bool isOverLimitC			 = false;
	bool isTorqueMeasuringLimit	 = false;
	bool isTorqueLimitSet		 = false;
	bool isTensionOnly			 = false;
	bool isSafetySwitchEngaged	 = false;

	bool isReverseConstant = true;

	// Packet payload
	uint8_t	 packetType			   = 0;
	uint8_t	 packetCounter		   = 0;
	uint8_t	 packetDriveState	   = 0;
	uint16_t measuredPwmA		   = 0;
	uint16_t measuredPwmB		   = 0;
	uint16_t measuredPwmC		   = 0;
	int16_t	 currentMeasuredRawA   = 0;
	int16_t	 currentMeasuredRawB   = 0;
	int16_t	 currentMeasuredRawC   = 0;
	int32_t	 encoderMeasuredCountA = 0;
	int32_t	 encoderMeasuredCountB = 0;
	int32_t	 encoderMeasuredCountC = 0;

	// Derived values
	float currentMeasuredAmpsA = 0.0f;
	float currentMeasuredAmpsB = 0.0f;
	float currentMeasuredAmpsC = 0.0f;
	float encoderMeasuredDegA  = 0.0f;
	float encoderMeasuredDegB  = 0.0f;
	float encoderMeasuredDegC  = 0.0f;
	float encoderLimitDegA	   = 0.0f;
	float encoderLimitDegB	   = 0.0f;
	float encoderLimitDegC	   = 0.0f;
	float measuredPwmPercentA  = 0.0f;
	float measuredPwmPercentB  = 0.0f;
	float measuredPwmPercentC  = 0.0f;

	// Limits
	cv::Point3f commandedLimits = cv::Point3f( 0.10f, 0.10f, 0.10f );
};

struct ArUcoStruct {

	// bool isArUcoTagFound = false;
};

struct RunningLogStruct {

	uint8_t runNumber = 0;
	uint8_t runErrX	  = 0;
};

struct CaptureStruct {
	bool	rotateCamera = false;
	bool	isFrameReady = false;
	cv::Mat frameRaw	 = cv::Mat( CONFIG_CAM_HEIGHT, CONFIG_CAM_WIDTH, CV_8UC3 );
	cv::Mat frameGray	 = cv::Mat( CONFIG_CAM_HEIGHT, CONFIG_CAM_WIDTH, CV_8UC3 );

	// OpenCV GPU matrices
	cv::cuda::GpuMat GpuMatFrameRaw			= cv::cuda::GpuMat( CONFIG_CAM_HEIGHT, CONFIG_CAM_WIDTH, CV_8UC3 );
	cv::cuda::GpuMat GpuMatFrameGray		= cv::cuda::GpuMat( CONFIG_CAM_HEIGHT, CONFIG_CAM_WIDTH, CV_8UC1 );
	cv::cuda::GpuMat GpuMatFrameUndistorted = cv::cuda::GpuMat( CONFIG_CAM_HEIGHT, CONFIG_CAM_WIDTH, CV_8UC1 );
	cv::cuda::GpuMat GpuMatRemap1;
	cv::cuda::GpuMat GpuMatRemap2;

	// OpenCV image matrices
	cv::Mat matFrameUndistorted = cv::Mat( CONFIG_CAM_HEIGHT, CONFIG_CAM_WIDTH, CV_8UC3 );
	cv::Mat matRemap1;
	cv::Mat matRemap2;
};

struct CalibrationStruct {

	bool		isCalibrated	  = false;	  // Has the finger been calibrated
	cv::Point3i calibratedOffetMM = cv::Point3i( 0, 0, 0 );
	cv::Point3i calibratedOffetPX = cv::Point3i( 0, 0, 0 );

	// bool		calibrationComplete = false;
	// cv::Point3i calibrationOffsetMM = cv::Point3i( 0, 0, 0 );
	// cv::Point3i calibrationOffsetPX = cv::Point3i( 0, 0, 0 );
	// cv::Point2i calibrationScreenPX = cv::Point2i( 0, 0 );
	// bool		calibrationLoaded	= false;
};

struct ControllerStruct {

	Point4f		gainKp;
	Point4f		gainKi;
	Point4f		gainKd;
	cv::Point3f proportionalTerm	   = cv::Point3f( 0.0f, 0.0f, 0.0f );	 // Combined proportional term
	cv::Point3f integralTerm		   = cv::Point3f( 0.0f, 0.0f, 0.0f );	 // Combined integral
	cv::Point3f derivativeTerm		   = cv::Point3f( 0.0f, 0.0f, 0.0f );	 // Combined derivative term
	cv::Point3f combinedPIDTerms	   = cv::Point3f( 0.0f, 0.0f, 0.0f );	 // Combined PID terms
	cv::Point3f combinedPIDTermPrev	   = cv::Point3f( 0.0f, 0.0f, 0.0f );	 // Combined PID terms
	cv::Point3i commandedPwmABC		   = cv::Point3i( 0, 0, 0 );			 // Commanded PWM output
	cv::Point3i commandedPwmABCLast	   = cv::Point3i( 0, 0, 0 );			 // Commanded PWM output
	cv::Point3f commandedPercentageABC = cv::Point3f( 0.0f, 0.0f, 0.0f );	 // Commanded percentage output
	cv::Point3f commandedCurrentABC	   = cv::Point3f( 0.0f, 0.0f, 0.0f );	 // Commanded current output
	cv::Point3f commandedTensionABC	   = cv::Point3f( 0.0f, 0.0f, 0.0f );	 // Commanded tension
	cv::Point3f torqueABC			   = cv::Point3f( 0.0f, 0.0f, 0.0f );	 // Commanded torque
	float		rampPercentage		   = 0.00f;								 // Counter
	float		rampStartTime		   = 0.0f;								 // [s]
	float		rampDurationTime	   = 1.0f;								 // [s]
	bool		isRampingUp			   = false;								 // Is the motor ramping up?
	bool		isLimitSet			   = false;								 // Are the motor limits set?
	int			integrationRadius	   = 100;
	cv::Point3f percentageProportional = cv::Point3f( 0.0f, 0.0f, 0.0f );
	cv::Point3f percentageIntegral	   = cv::Point3f( 0.0f, 0.0f, 0.0f );
	cv::Point3f percentageDerivative   = cv::Point3f( 0.0f, 0.0f, 0.0f );
	bool		toggleReverse		   = false;
};

struct DisplayStruct {

	// Matrix to display overlay
	cv::Mat		matFrameOverlay = cv::Mat( CONFIG_DIS_HEIGHT, CONFIG_DIS_WIDTH, CV_8UC3 );
	std::string statusString	= "";
};

struct InputStruct {

	// Enumerated gain selection
	// selectGainTargetEnum   selectGainTarget	  = selectGainTargetEnum::NONE;
	// selectGainEnum		   selectGain		  = selectGainEnum::NONE;
	// selectTorqueTargetEnum selectTorqueTarget = selectTorqueTargetEnum::NONE;
	selectLimitEnum selectLimit = selectLimitEnum::NONE;

	// Selection
	selectSystemEnum	selectedAdjustmentSystem	= selectSystemEnum::NONE;
	selectSubsystemEnum selectedAdjustmentSubsystem = selectSubsystemEnum::NONE;
};

struct KalmanFilterStruct {
	cv::Mat pMatrix;
};

struct LoggingStruct {

	// Flags
	bool isRunning = false;	   // Is logging actively running
	bool isEnabled = false;	   // Is logging enabled

	// Logging variables
	std::string filenameTxt = "NULL";
	std::string filenamePng = "NULL";
	std::string header1		= "Heading1";
	std::string header2		= "Heading2";
	std::string header3		= "Heading3";
	std::string header4		= "Heading4";
	std::string header5		= "Heading5";
	std::string header6		= "Heading5";
	std::string header7		= "Heading5";
	std::string header8		= "Heading5";
	std::string header9		= "Heading5";
	std::string header10	= "Heading5";
	std::string variable1	= "0";
	std::string variable2	= "0";
	std::string variable3	= "0";
	std::string variable4	= "0";
	std::string variable5	= "0";
	std::string variable6	= "0";
	std::string variable7	= "0";
	std::string variable8	= "0";
	std::string variable9	= "0";
	std::string variable10	= "0";
	std::string dataAndTime = "NULL";
};


struct SystemStruct {

	// Flags
	bool isMainRunning	= true;		// Global flag to shut-down safely
	bool isShuttingDown = false;	// Trigger safe shutdown
	bool useRing		= true;

	// State enum
	stateEnum state = stateEnum::IDLE;
};

struct SerialStruct {

	// Flags
	bool isSerialSendOpen	 = false;
	bool isSerialSending	 = false;
	bool isSerialReceiveOpen = false;
	bool isSerialReceiving	 = false;

	int8_t packetDelay = 0;

	// Plaintext packet
	std::string packetOut = "";
	std::string packetIn  = "";
};

struct TaskStruct {
	taskEnum	state			 = taskEnum::IDLE;
	bool		isRunning		 = false;	 // Is the task running
	float		elapsedTaskTime	 = 0.0f;	 // Task running time
	int			userID			 = 000;		 // Participant ID
	int			repetitionNumber = 0;		 // Task rep number
	char		command			 = 0;		 // Command
	std::string name			 = "IDLE";
	cv::Point2i targetPosition	 = cv::Point2i( 0, 0 );
	float		completionTime	 = 0.0f;
};


struct TargetTelemetryStruct {
	int						 activeID			   = 1;	   // ID of target currently being tracked
	bool					 isTargetReset		   = false;
	bool					 isTargetFound		   = false;
	bool					 isTargetLost		   = false;
	bool					 wasTargetDetected	   = false;
	cv::Point2i				 screenPositionPX	   = cv::Point2i( 0, 0 );																		// [px] Position of target in screen space
	cv::Point3f				 positionUnfilteredMM  = cv::Point3f( 0.0f, 0.0f, 0.0f );															// [mm] Raw (unfiltered) position of tag relative to camera
	cv::Point3f				 positionFilteredNewMM = cv::Point3f( 0.0f, 0.0f, 0.0f );															// [mm] New 3D filtered position
	cv::Point3f				 positionFilteredOldMM = cv::Point3f( 0.0f, 0.0f, 0.0f );															// [mm] Old 3D filtered position
	cv::Point3f				 velocityFilteredNewMM = cv::Point3f( 0.0f, 0.0f, 0.0f );															// [mm] New 3D filtered position
	cv::Point3f				 velocityFilteredOldMM = cv::Point3f( 0.0f, 0.0f, 0.0f );															// [mm] New 3D filtered position
	std::vector<cv::Point2i> cornersPX			   = { cv::Point2i( 0, 0 ), cv::Point2i( 0, 0 ), cv::Point2i( 0, 0 ), cv::Point2i( 0, 0 ) };	// Target corners in pixel space
	cv::Point3f				 positionIntegratedMM  = cv::Point3f( 0.0f, 0.0f, 0.0f );															// Integrated position
	cv::Point3f				 offsetMm			   = cv::Point3f( 0, -CONFIG_TARGET_OFFSET_Y_MM, 0 );											// Offset
	float					 rotationDEG		   = 0.0f;																						// Target angle
};

struct RingTelemetryStruct {

	// Ring and finger
	bool					 isRingFound		  = false;
	cv::Point3f				 ringPositionMm		  = cv::Point3i( 0, 0, 0 );
	cv::Point3f				 fingerPositionMm	  = cv::Point3i( 0, 0, 0 );
	cv::Point2i				 ringScreenPositionPX = cv::Point2i( 0, 0 );																	   // [px] Position of target in screen space
	std::vector<cv::Point2i> ringCornersPX		  = { cv::Point2i( 0, 0 ), cv::Point2i( 0, 0 ), cv::Point2i( 0, 0 ), cv::Point2i( 0, 0 ) };	   // Target corners in pixel space

	// Offsets

	cv::Point3f fingerOffsetMm = cv::Point3f( 0, 20, -40 );
	cv::Vec3d	ringRotationVector, ringTranslationVector;
};


struct TimingStruct {

	short measuredFrequency	 = 45;		// Default timing frequency
	float elapsedRunningTime = 0.0f;	// Elapsed time in seconds
	float timestepDT		 = 0.0f;
};

struct TouchscreenStruct {

	cv::Point3i positionTouched = cv::Point3i( 0, 0, 0 );
	bool		isTouched		= false;
};


struct VibrationStruct {
	bool isRunning = false;
};


// Shared system variable container
struct ManagedData {

	// Structs
	AmplifierStruct		  Amplifier;
	ArUcoStruct			  Aruco;
	CalibrationStruct	  Calibration;	  // Logging variables
	CaptureStruct		  Capture;
	ControllerStruct	  Controller;
	DisplayStruct		  Display;
	InputStruct			  Input;
	KalmanFilterStruct	  KalmanFilter;
	LoggingStruct		  Logging;
	SystemStruct		  System;
	SerialStruct		  Serial;
	TargetTelemetryStruct Target;
	TaskStruct			  Task;
	TimingStruct		  Timing;
	TouchscreenStruct	  Touchscreen;
	VibrationStruct		  Vibration;

	// Helper functions
	float		GetNorm2D( cv::Point2f pt1 );													// Calculate magnitude of 2D vector
	float		GetNorm3D( cv::Point3f pt1 );													// Calculate magnitude of 3D vector
	float		GetDist2D( cv::Point2f pt1, cv::Point2f pt2 );									// Calculate magnitude of 2D distance between 2 points
	float		GetDist3D( cv::Point3f pt1, cv::Point3f pt2 );									// Calculate magnitude of 2D distance between 2 points
	cv::Point2f GetDelta2D( cv::Point2f pt1, cv::Point2f pt2 );									// Calculate the x,y delta beteen two points
	cv::Point3f GetDelta3D( cv::Point3f pt1, cv::Point3f pt2 );									// Calculate the x,y,z delta beteen two points
	std::string FormatDecimal( float x, uint8_t p, uint8_t d );									// Formats a float to xx.xxx
	std::string PadValues( int val, int nZeroes );												// Return a padded string
	float		MapFloat( float val, float inMin, float inMax, float outMin, float outMax );	// Return a remapped float
																								// int			MapInt( int x, int inMin, int inMax, int outMin, int outMax );					// Return a remapped integer
};

class SystemDataManager {
public:
	// Class
	SystemDataManager();

	// Functions
	std::shared_ptr<ManagedData> getData();


private:
	std::shared_ptr<ManagedData> data;
};