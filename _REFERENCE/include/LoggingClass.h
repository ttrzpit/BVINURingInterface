#pragma once

// Libraries
#include <filesystem>	 // For creating folders
#include <fstream>		 // For saving files
#include <iomanip>		 // For padding zeroes

// OpenCV core functions
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>	  

// Memory for shared data
#include <memory>

// Configuration
#include <config.h>

// Forward declarations
class SystemDataManager;
struct ManagedData;

/* Struct for data to be logged*/
struct loggingStruct {

	float		timestamp;
	cv::Point3i telemetry = cv::Point3i( 0.0f, 0.0f, 0.0f );

	// Custom headers
	std::string custom1;
	std::string custom2;
	std::string custom3;
	std::string custom4;
	std::string custom5;
	std::string custom6;
	std::string custom7;
	std::string custom8;
	std::string custom9;
	std::string custom10;
};



class LoggingClass {

public:
	// Data manager
	LoggingClass( SystemDataManager& dataHandle );

	// Public functions
	void AddEntry();
	void SaveTxt();
	void SavePng( cv::Mat img );
	void Initialize();


private:
	// Private functions
	std::string PadValues( int val, int nZeroes );

	// Vector to log data
	std::vector<loggingStruct> logFile;

	// Variables
	loggingStruct newEntry;

	// File paths
	std::filesystem::path basePath				 = "";	  // Base path to save files
	std::filesystem::path userFolder			 = "";	  // User-specific folder based on userID
	std::filesystem::path fullPathAndFilenameTxt = "";	  // Full system filename for text output
	std::filesystem::path fullPathAndFilenamePng = "";	  // Full system filename for image output

	// Data manager handle
	SystemDataManager&			 dataHandle;
	std::shared_ptr<ManagedData> shared;
};