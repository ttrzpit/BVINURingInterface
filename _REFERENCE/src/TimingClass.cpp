// Call to class header
#include "TimingClass.h"

// System data manager
#include "SystemDataManager.h"


// Constructor
TimingClass::TimingClass( SystemDataManager& ctx )
	: dataHandle( ctx )
	, shared( ctx.getData() ) {
	std::cout << "TimingClass:  Timer initialized.\n";
}



/**
 * @brief Start the timer
 */
void TimingClass::StartTimer() {

	// Capture current time
	previousTime = std::chrono::steady_clock::now();
}



/**
 * @brief Updates the timer, pulling in current time and calculating average frequency (once per second)
 */
void TimingClass::Update() {

	// Increment loop counter
	loopCounter++;

	// Get current time
	currentTime = std::chrono::steady_clock::now();

	// Get elapsed time
	elapsedTime						  = currentTime - previousTime;
	elapsedTimeFreq					  = currentTime - previousTimeFreq;
	shared->Timing.elapsedRunningTime = elapsedTime.count();

	// If a task is running, update timer
	if ( shared->Task.isRunning ) {
		UpdateTaskTime();
	}



	// Check if 1 second has passed
	if ( elapsedTimeFreq.count() >= 1.0 ) {
		shared->Timing.measuredFrequency = short( loopCounter / elapsedTimeFreq.count() );
		shared->Timing.timestepDT		 = 1.0f / shared->Timing.measuredFrequency;
		loopCounter						 = 0;
		previousTimeFreq				 = currentTime;	   // This resets the time every second
	}
}


/**
 * @brief Save a string of the current date and time to shared and print time
 * 
 * @return std::string MMDDYYHHMM
 */
void TimingClass::UpdateFullDateAndTime() {

	// Get current time
	std::time_t t	= std::time( nullptr );
	std::tm*	now = std::localtime( &t );

	// Format time
	std::ostringstream ssTime;
	ssTime << "_d" << std::put_time( now, "%m%d" ) << "_t" << std::put_time( now, "%H%M%S" );

	// Output
	std::cout << "Timestamp generated: " << ssTime.str() << "\n";
	shared->Logging.dataAndTime = ssTime.str();
}



/**
 * @brief Get the current timestamp
 * 
 * @param echo 
 * @return std::string 
 */
std::string TimingClass::GetFullDateAndTime( bool echo ) {

	// Get current time
	std::time_t t	= std::time( nullptr );
	std::tm*	now = std::localtime( &t );

	// Format time
	std::ostringstream ssTime;
	ssTime << "_d" << std::put_time( now, "%m%d" ) << "_h" << std::put_time( now, "%H%M%S" );

	// Return timestamp
	return ssTime.str();
}



/**
 * @brief Add a string of the current date and time
 * 
 * @return std::string MMDDYYHHMM
 */
void TimingClass::UpdateTaskTime() {

	// Get current time
	currentTaskTime = std::chrono::steady_clock::now();

	// Get elapsed time
	taskTimeElapsed				 = currentTaskTime - taskTimeStart;
	shared->Task.elapsedTaskTime = taskTimeElapsed.count();
}



void TimingClass::TaskTimerStart() {

	// Capture current time
	taskTimeStart = std::chrono::steady_clock::now();

	// Update flag
	TASK_TIMER_STARTED = true;
}


void TimingClass::TaskTimerStop() {

	// Capture final time
	taskTimeEnd = std::chrono::steady_clock::now();

	// Save final time
	if ( shared->Task.name == "FITTS" ) {
		taskTimeElapsed				= taskTimeEnd - taskTimeStart;
		shared->Task.completionTime = taskTimeElapsed.count();
	}

	// Update flag
	TASK_TIMER_STARTED = false;
}


/**
 * @brief Returns the elapsed time in seconds
 * 
 * @return float Current elapsed time
 */
float TimingClass::TaskTimerGetElapsedSeconds() {

	return taskTimeElapsed.count();
}