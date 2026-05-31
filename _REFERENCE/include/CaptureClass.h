#pragma once

// Memory for shared data
#include <memory>

// OpenCV core functions
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>



// Forward declarations
class SystemDataManager;
struct ManagedData;


/** 
 * @brief Capture class definition
 */
class CaptureClass {


public:
	// Data manager handle
	CaptureClass( SystemDataManager& dataHandle );

	// Public functions
	void GetFrame();

private:
	// Data manager handle
	SystemDataManager&			 dataHandle;
	std::shared_ptr<ManagedData> shared;

	// Capture variables
	cv::VideoCapture Capture;


	// Private functions
	void		Initialize();
	
};