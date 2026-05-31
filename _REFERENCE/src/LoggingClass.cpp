// Call to class header
#include "LoggingClass.h"

// System data manager
#include "SystemDataManager.h"


/**
 * @brief Constructor
 */
LoggingClass::LoggingClass( SystemDataManager& ctx )
	: dataHandle( ctx )
	, shared( ctx.getData() ) {
	logFile.reserve( 6000 );
}


void LoggingClass::Initialize() {

	// Select folder based on task
	if ( shared->Task.state == taskEnum::FITTS ) {

		// Fitts folder
		basePath   = "/home/tom/Code/nuring/logging/fitts/";
		userFolder = basePath / std::to_string( shared->Task.userID );


		// Create user folder if not present
		if ( !std::filesystem::exists( userFolder ) ) {
			std::filesystem::create_directories( userFolder );
		}
	} else {

		// General folder
		basePath = "/home/tom/Code/nuring/logging/";
	}

	// Update filenames
	shared->Logging.filenameTxt	 = ( shared->Task.name + "_" + std::to_string( shared->Task.userID ) + shared->Logging.dataAndTime + ".txt" );
	shared->Logging.filenamePng	 = ( shared->Task.name + "_" + std::to_string( shared->Task.userID ) + shared->Logging.dataAndTime + ".png" );
	fullPathAndFilenameTxt		 = userFolder / shared->Logging.filenameTxt;
	fullPathAndFilenamePng		 = userFolder / shared->Logging.filenamePng;
	shared->Display.statusString = "Creating Log: " + shared->Logging.filenameTxt;

	// Clear old data
	logFile.clear();
}


/**
 * Items to log
 * timestamp, tagID, errXmm, errYmm, errZmm, errXpx, errYpx, mag, theta
 */
void LoggingClass::AddEntry() {

	// Use task time if task is running
	if ( shared->Task.isRunning ) {
		newEntry.timestamp = shared->Task.elapsedTaskTime;
	} else {
		newEntry.timestamp = shared->Timing.elapsedRunningTime;
	}

	// Update telemetry data
	newEntry.telemetry = shared->Target.positionFilteredNewMM;

	// Update variables
	newEntry.custom1  = shared->Logging.variable1;
	newEntry.custom2  = shared->Logging.variable2;
	newEntry.custom3  = shared->Logging.variable3;
	newEntry.custom4  = shared->Logging.variable4;
	newEntry.custom5  = shared->Logging.variable5;
	newEntry.custom6  = shared->Logging.variable6;
	newEntry.custom7  = shared->Logging.variable7;
	newEntry.custom8  = shared->Logging.variable8;
	newEntry.custom9  = shared->Logging.variable9;
	newEntry.custom10 = shared->Logging.variable10;

	// Save entry
	logFile.push_back( newEntry );

	// Clear entry struct
	newEntry = {};
}



void LoggingClass::SaveTxt() {

	// Stop logging
	shared->Logging.isRunning = false;

	// Open file
	std::ofstream file( fullPathAndFilenameTxt );

	// Check if file opened safely
	if ( !file.is_open() ) {
		std::cerr << "Failed to open file\n";
		shared->Display.statusString = "Logging Class: Failed to open file";
		return;
	}


	// Write header
	std::string headerString = "timestamp,Xmm,Ymm,Zmm," + shared->Logging.header1 + "," + shared->Logging.header2 + "," + shared->Logging.header3 + "," + shared->Logging.header4 + "," + shared->Logging.header5 + "," + shared->Logging.header6 + "," + shared->Logging.header7 + ","
		+ shared->Logging.header8 + "," + shared->Logging.header9 + "," + shared->Logging.header10 + "\n";
	file << headerString;

	// Write file contents
	for ( const auto& entry : logFile ) {
		file << entry.timestamp << "," << entry.telemetry.x << "," << entry.telemetry.y << "," << entry.telemetry.z << "," << entry.custom1 << "," << entry.custom2 << "," << entry.custom3 << "," << entry.custom4 << "," << entry.custom5 << "," << entry.custom6 << "," << entry.custom7 << ","
			 << entry.custom8 << "," << entry.custom9 << "," << entry.custom10 << "\n";
	}

	// Close file
	file.close();

	// Update flag
	shared->Logging.isRunning = false;
}



/**
 * @brief Pad the given value with a specified number of zeros
 * @param val Value to be padded
 * @param nZeroes Number of padding zeroes
 */
std::string LoggingClass::PadValues( int val, int nZeroes ) {
	std::ostringstream sstream;
	sstream << std::setw( nZeroes ) << std::setfill( '0' ) << val;
	return sstream.str();
}

/**
 * @brief Save provided image as png
 * 
 * @param img cv::Mat to be saved
 */
void LoggingClass::SavePng( cv::Mat img ) {

	// Save image
	if ( shared->Logging.isEnabled ) {

		shared->Display.statusString = "Saving file " + shared->Logging.filenamePng;
		cv::imwrite( fullPathAndFilenamePng, img );
		std::cout << "FittsClass:  Image saved at " << shared->Logging.filenamePng << "\n";
	}
}