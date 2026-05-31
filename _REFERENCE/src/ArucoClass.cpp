// Call to class header
#include "ArucoClass.h"

// System data manager
#include "SystemDataManager.h"

/**
 * @brief Construct a new Aruco Class:: Aruco Class object
 * 
 * @param ctx Handle to data manager
 */
ArucoClass::ArucoClass( SystemDataManager& ctx )
	: dataHandle( ctx )
	, shared( ctx.getData() ) {
	Initialize();
}


/**
 * @brief Initialize ArUco detector parameters
 * 
 */
void ArucoClass::Initialize() {

	// Assign dictionary
	arucoDictionary = cv::aruco::getPredefinedDictionary( cv::aruco::DICT_4X4_50 );

	// Assign ArUco detector parameters
	arucoDetectorParams.adaptiveThreshConstant		  = 7;									// 7
	arucoDetectorParams.adaptiveThreshWinSizeMin	  = 3;									// 3  KEEP THIS
	arucoDetectorParams.adaptiveThreshWinSizeMax	  = 53;									// 13	KEEP THIS
	arucoDetectorParams.adaptiveThreshWinSizeStep	  = 4;									// 10
	arucoDetectorParams.minMarkerPerimeterRate		  = 0.01;								// 0.03
	arucoDetectorParams.maxMarkerPerimeterRate		  = 4.0;								// 4.0 Not critical
	arucoDetectorParams.polygonalApproxAccuracyRate	  = 0.03;								// 0.15
	arucoDetectorParams.minCornerDistanceRate		  = 0.02;								// 0.05
	arucoDetectorParams.minDistanceToBorder			  = 1;									// 3
	arucoDetectorParams.cornerRefinementMethod		  = cv::aruco::CORNER_REFINE_SUBPIX;	// cv::aruco::CORNER_REFINE_SUBPIX;
	arucoDetectorParams.cornerRefinementMaxIterations = 50;									// 30
	arucoDetectorParams.cornerRefinementMinAccuracy	  = 0.01;
	// arucoDetectorParams.useAruco3Detection			  = true;
	arucoDetectorParams.detectInvertedMarker = true;	// 0.1

	// Re-initialize detector
	arucoDetector = cv::aruco::ArucoDetector( arucoDictionary, arucoDetectorParams );

	// Configure real-world conversion system
	arucoPoints.ptr<cv::Vec3f>( 0 )[0] = cv::Vec3f( -CONFIG_LARGE_MARKER_WIDTH / 2.f, CONFIG_LARGE_MARKER_WIDTH / 2.f, 0 );
	arucoPoints.ptr<cv::Vec3f>( 0 )[1] = cv::Vec3f( CONFIG_LARGE_MARKER_WIDTH / 2.f, CONFIG_LARGE_MARKER_WIDTH / 2.f, 0 );
	arucoPoints.ptr<cv::Vec3f>( 0 )[2] = cv::Vec3f( CONFIG_LARGE_MARKER_WIDTH / 2.f, -CONFIG_LARGE_MARKER_WIDTH / 2.f, 0 );
	arucoPoints.ptr<cv::Vec3f>( 0 )[3] = cv::Vec3f( -CONFIG_LARGE_MARKER_WIDTH / 2.f, -CONFIG_LARGE_MARKER_WIDTH / 2.f, 0 );

	// Configure tag vector sizes
	// arucoTagsPresent.resize( 11 );

	// Update console
	std::cout << "ArucoClass:   ArUco detector initialized.\n";
}


/**
 * @brief Find ArUco tag markers
 */
void ArucoClass::FindTags() {

	if ( shared->Task.isRunning ) {

		// Make sure image is ready to be processed
		if ( shared->Capture.isFrameReady ) {

			// Reset global flags
			shared->Target.isTargetFound = false;
			// shared->FLAG_FINGER_MARKER_FOUND = false;

			// Reset individual tag state
			for ( size_t t = 0; t < arucoTagsPresent.size(); t++ ) {
				arucoTagsPresent[t] = false;
			}

			// Run detector
			// arucoDetector.detectMarkers( shared->matFrameGray, arucoCorners, arucoDetectedIDs, arucoRejects );
			arucoDetector.detectMarkers( shared->Capture.frameGray, arucoCorners, arucoDetectedIDs );

			// Check if markers have been found
			if ( !arucoDetectedIDs.empty() ) {

				// Process each detected marker
				for ( int i = 0; i < arucoDetectedIDs.size(); i++ ) {

					if ( ( arucoDetectedIDs[i] > 0 && arucoDetectedIDs[i] <= 5 ) || ( arucoDetectedIDs[i] == 8 ) ) {

						if ( ( arucoDetectedIDs[i] > 0 ) && ( arucoDetectedIDs[i] == shared->Target.activeID ) ) {	  // Only process active tag


							// Extract current corner
							std::vector<std::vector<cv::Point2f>> currentCorner = { arucoCorners[i] };

							// Estimate tag pose formarkers in the valid range
							// cv::aruco::estimatePoseSingleMarkers( currentCorner, CONFIG_LARGE_MARKER_WIDTH, CONFIG_CAMERA_MATRIX, CONFIG_DISTORTION_COEFFS, arucoRotationVector, arucoTranslationVector );

							cv::Mat distZero = cv::Mat::zeros( CONFIG_DISTORTION_COEFFS.size(), CONFIG_DISTORTION_COEFFS.type() );
							cv::aruco::estimatePoseSingleMarkers( currentCorner, CONFIG_LARGE_MARKER_WIDTH, CONFIG_CAMERA_MATRIX, distZero, arucoRotationVector, arucoTranslationVector );

							if ( arucoTranslationVector.empty() ) {
								continue;
							}

							// Update flag
							shared->Target.isTargetFound		  = true;
							arucoTagsPresent[arucoDetectedIDs[i]] = true;

							// Update 2D pixel coordinates
							int avgX						= int( ( currentCorner[0][0].x + currentCorner[0][1].x + currentCorner[0][2].x + currentCorner[0][3].x ) / 4.0f );
							int avgY						= int( ( currentCorner[0][0].y + currentCorner[0][1].y + currentCorner[0][2].y + currentCorner[0][3].y ) / 4.0f );
							shared->Target.screenPositionPX = cv::Point2i( avgX, avgY );

							// Update 2D corner vector for active marker
							shared->Target.cornersPX[0] = cv::Point2i( currentCorner[0][0].x, currentCorner[0][0].y );
							shared->Target.cornersPX[1] = cv::Point2i( currentCorner[0][1].x, currentCorner[0][1].y );
							shared->Target.cornersPX[2] = cv::Point2i( currentCorner[0][2].x, currentCorner[0][2].y );
							shared->Target.cornersPX[3] = cv::Point2i( currentCorner[0][3].x, currentCorner[0][3].y );

							// Update telemetry / 3D real-world coordinates
							shared->Target.positionUnfilteredMM = cv::Point3f( arucoTranslationVector[0][0], -arucoTranslationVector[0][1], arucoTranslationVector[0][2] );
							shared->Target.rotationDEG			= arucoRotationVector[0][1] * RAD2DEG;
						}


					}	 // End process valid marker
					else {
						// shared->FLAG_TARGET_MARKER_FOUND = false;
						shared->Controller.isRampingUp = true;
					}
				}	 // For loop

			} else {
				// shared->lostCount++;
			}	 // End empty check
		} else {

			// Display error message
			std::cout << "ArucoClass: Image not ready for processing!\n";

		}	 // End is frame ready

		// Logic for reverse
		if ( !shared->Target.isTargetFound && shared->Target.wasTargetDetected && ( shared->Target.positionFilteredNewMM.z > 50.0f ) && ( shared->Target.positionFilteredNewMM.z < 300.0f ) ) {
			shared->Controller.toggleReverse = true;
		} else if ( shared->Target.isTargetFound && !shared->Target.wasTargetDetected ) {
			shared->Controller.toggleReverse = false;
		}

		shared->Target.wasTargetDetected = shared->Target.isTargetFound;
	} else {

		// Update 2D corner vector for active marker
		shared->Target.cornersPX[0] = cv::Point2i( 0, 0 );
		shared->Target.cornersPX[1] = cv::Point2i( 0, 0 );
		shared->Target.cornersPX[2] = cv::Point2i( 0, 0 );
		shared->Target.cornersPX[3] = cv::Point2i( 0, 0 );
	}

}	 // End function
