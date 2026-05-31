// Call to class header
#include "CaptureClass.h"
#include <opencv2/highgui.hpp>

// System data manager
#include "SystemDataManager.h"


/**
 * @class CaptureClass
 * @brief Handles camera capture functionality.
 * 
 * This class is responsible for capturing frames from the camera.
 * It interacts with the SystemDataManager to store and retrieve data.
 */
CaptureClass::CaptureClass( SystemDataManager& ctx )
	: dataHandle( ctx )
	, shared( ctx.getData() ) {

	Initialize();
}


void CaptureClass::Initialize() {



	// Configure OpenCV Optimizations
	cv::setUseOptimized( true );

	// Initialize undistort map tool
	// cv::initUndistortRectifyMap( CONFIG_CAMERA_MATRIX, CONFIG_DISTORTION_COEFFS, cv::Mat(), CONFIG_CAMERA_MATRIX, shared->Capture.frameRaw.size(), CV_32F, shared->Capture.matRemap1, shared->Capture.matRemap2 );

	cv::Size sz( CONFIG_CAM_WIDTH, CONFIG_CAM_HEIGHT );
	cv::initUndistortRectifyMap( CONFIG_CAMERA_MATRIX, CONFIG_DISTORTION_COEFFS, cv::Mat(), CONFIG_CAMERA_MATRIX, sz, CV_32FC1, shared->Capture.matRemap1, shared->Capture.matRemap2 );



	// Try catch to open camera device
	try {
		// Capture device
		Capture.open( "/dev/video0", cv::CAP_V4L2 );

		// Capture settings
		Capture.set( cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc( 'M', 'J', 'P', 'G' ) );
		Capture.set( cv::CAP_PROP_FRAME_WIDTH, CONFIG_CAM_WIDTH );
		Capture.set( cv::CAP_PROP_FRAME_HEIGHT, CONFIG_CAM_HEIGHT );
		Capture.set( cv::CAP_PROP_FPS, CONFIG_CAM_FRAMERATE );
		Capture.set( cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY );

		// Camera settings
		Capture.set( cv::CAP_PROP_BRIGHTNESS, CONFIG_CAM_BRIGHTNESS );

		Capture.set( cv::CAP_PROP_CONTRAST, CONFIG_CAM_CONTRAST );
		Capture.set( cv::CAP_PROP_SATURATION, CONFIG_CAM_SATURATION );
		Capture.set( cv::CAP_PROP_HUE, CONFIG_CAM_HUE );
		Capture.set( cv::CAP_PROP_AUTO_WB, CONFIG_CAM_AUTO_WHITEBALANCE );
		Capture.set( cv::CAP_PROP_GAMMA, CONFIG_CAM_GAMMA );
		Capture.set( cv::CAP_PROP_GAIN, CONFIG_CAM_GAIN );
		Capture.set( cv::CAP_PROP_SHARPNESS, CONFIG_CAM_SHARPNESS );
		Capture.set( cv::CAP_PROP_BACKLIGHT, CONFIG_CAM_BACKLIGHT );
		Capture.set( cv::CAP_PROP_AUTO_EXPOSURE, CONFIG_CAM_AUTO_EXPOSURE );
		Capture.set( cv::CAP_PROP_EXPOSURE, CONFIG_CAM_EXPOSURE_LEVEL );
		Capture.set( cv::CAP_PROP_PAN, CONFIG_CAM_PAN );
		Capture.set( cv::CAP_PROP_TILT, CONFIG_CAM_TILT );
		Capture.set( cv::CAP_PROP_FOCUS, CONFIG_CAM_FOCUS_LEVEL );
		Capture.set( cv::CAP_PROP_AUTOFOCUS, CONFIG_CAM_AUTO_FOCUS );
		Capture.set( cv::CAP_PROP_ZOOM, CONFIG_CAM_ZOOM );


		// Ensure capture devices open and running
		if ( !Capture.isOpened() ) {
			std::cerr << "Error: Could not open the camera." << std::endl;
		}
	} catch ( cv::Exception& e ) {
		std::cout << "Exception in CameraInterface::Begin(): " << e.msg << "\n";
	}

	// Output confirmation
	std::cout << "CaptureClass: Camera initialized at ( " << Capture.get( cv::CAP_PROP_FRAME_HEIGHT ) << " x " << Capture.get( cv::CAP_PROP_FRAME_WIDTH ) << " ) @ " << Capture.get( cv::CAP_PROP_FPS ) << " fps, mode = " << Capture.get( cv::CAP_PROP_BACKEND )
			  << " , Accel = " << Capture.get( cv::CAP_PROP_HW_ACCELERATION ) << "\n";
}

/**
 * @brief Captures the latest frame
 */
void CaptureClass::GetFrame() {

	// Reset flag
	shared->Capture.isFrameReady = false;

	// Capture latest frame
	Capture.grab();
	Capture.retrieve( shared->Capture.frameRaw );

	// Make sure frame isn't empty
	if ( !shared->Capture.frameRaw.empty() ) {

		// Upload mats to GPU
		shared->Capture.GpuMatFrameRaw.upload( shared->Capture.frameRaw );
		shared->Capture.GpuMatRemap1.upload( shared->Capture.matRemap1 );
		shared->Capture.GpuMatRemap2.upload( shared->Capture.matRemap2 );

		// Remap using GPU
		cv::cuda::remap( shared->Capture.GpuMatFrameRaw, shared->Capture.GpuMatFrameUndistorted, shared->Capture.GpuMatRemap1, shared->Capture.GpuMatRemap2, cv::INTER_LINEAR );

		// Convert to grayscale using GPU
		cv::cuda::cvtColor( shared->Capture.GpuMatFrameUndistorted, shared->Capture.GpuMatFrameGray, cv::COLOR_BGR2GRAY );



		// Extract frames from GPU
		shared->Capture.GpuMatFrameUndistorted.download( shared->Capture.matFrameUndistorted );
		shared->Capture.GpuMatFrameGray.download( shared->Capture.frameGray );

		// Rotate 180 degrees on GPU (flip both axes)
		if ( shared->Capture.rotateCamera ) {
			cv::flip( shared->Capture.frameGray, shared->Capture.frameGray, -1 );
			cv::flip( shared->Capture.matFrameUndistorted, shared->Capture.matFrameUndistorted, -1 );
		}


		// brightness: shift (beta), contrast: scale (alpha)
		double alpha = 1.3;	   // contrast (1.0 = no change)
		double beta	 = 100;	   // brightness (0 = no change)

		// // Apply contrast and brightness adjustment
		// shared->Capture.frameGray.convertTo( shared->Capture.frameGray, -1, alpha, beta );

		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE( 2.0, cv::Size( 8, 8 ) );
		clahe->apply( shared->Capture.frameGray, shared->Capture.frameGray );


		// cv::bitwise_not(shared->Capture.frameGray, shared->Capture.frameGray);
		// cv::imshow( "Gray Frame", shared->Capture.frameGray );

		// Update flag
		shared->Capture.isFrameReady = true;

	} else {
		std::cerr << "CaptureClass: Captured frame is empty!\n";
		shared->Capture.isFrameReady = false;
	}
}
