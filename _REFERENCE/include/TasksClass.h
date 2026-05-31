#pragma once


// Memory for shared data
#include <memory>

// Library for randomization
#include <random>

// Configuration
#include <config.h>

// OpenCV core functions
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>	  // For pollKey()

// System data manager
#include "LoggingClass.h"
#include "SystemDataManager.h"
#include "TimingClass.h"

// Forward declarations
class SystemDataManager;
class TimingClass;
class LoggingClass;
struct ManagedData;

class TasksClass {

public:
	// Constructor
	TasksClass( SystemDataManager& dataHandle, TimingClass& timerHandle, LoggingClass& loggerHandle );

	void Calibration();
	void Fitts();
	void Limits();
	void AutoGains();

private:
	// Data manager handle
	SystemDataManager&			 dataHandle;
	TimingClass&				 Timer;
	LoggingClass&				 Logger;
	std::shared_ptr<ManagedData> shared;

	// Private variables
	std::string winTaskBackground = "Calibration Interface";

	// ArUco tag
	cv::Mat matAruco08 = cv::imread( "/home/tom/Code/nuring/images/tags/aruco-08-20mm-scaled.png" );
	cv::Mat matAruco01;


	// Target background
	cv::Mat matTaskBackground = cv::Mat( CONFIG_TOUCHSCREEN_HEIGHT_PX, CONFIG_TOUCHSCREEN_WIDTH_PX, CV_8UC3 );

	// Marker position variables
	unsigned short posX = ( CONFIG_TOUCHSCREEN_WIDTH_PX / 2 ) - ( matAruco08.rows / 2 );
	unsigned short posY = ( CONFIG_TOUCHSCREEN_HEIGHT_PX / 2 ) - ( matAruco08.rows / 2 );

	bool isComplete	 = false;
	bool isFinishing = false;

	unsigned short minX = 0;
	unsigned short maxX = 0;
	unsigned short minY = 0;
	unsigned short maxY = 0;


	// Calibration
	void CalibrationStart();
	void CalibrationUpdate();
	void CalibrationFinish();

	// Fitts
	void		FittsStart();
	void		FittsUpdate();
	void		FittsFinish();
	void		FittsGeneratePosition();
	void		FittsLoggingStart();
	void		FittsLoggingUpdate();
	cv::Point2i touchPosition  = cv::Point2i( 0, 0 );
	cv::Point2i targetPosition = cv::Point2i( 0, 0 );
	cv::Point2i errorPx		   = cv::Point2i( 0, 0 );
	cv::Point2i errorMm		   = cv::Point2i( 0, 0 );

	// Limits
	void LimitsStart();
	void LimitsUpdate();
	void LimitsFinish();

	// General
	void InitializeInterface( taskEnum task );



	//
	void AutoGainsStart();
	void AutoGainsUpdate();
	void AutoGainsFinish();
	void AutoGainsGenerateTarget();

	void LiveCalibrationHorizontal() ;
};
