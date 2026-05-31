// Call to class header
#include "DisplayClass.h"

// System data manager
#include "SystemDataManager.h"


// For decimal formatting
#include <iomanip>

/**
 * @brief DisplayClass constructor
 */
DisplayClass::DisplayClass( SystemDataManager& ctx )
	: dataHandle( ctx )
	, shared( ctx.getData() ) {

	cv::namedWindow( winInterface, cv::WINDOW_AUTOSIZE );
	cv::moveWindow( winInterface, 3440 - CONFIG_DIS_WIDTH - 2, 0 );

	// Set font based on chosen resolution
	if ( CONFIG_TYPE == "LowResolution" ) {
		fontHeader	   = 0.65f;
		fontBody	   = 0.55f;
		key_fontHeader = 0.65f;
		key_fontBody   = 0.55f;
	} else if ( CONFIG_TYPE == "HighResolution" ) {
		fontTitle	   = 0.6f;
		fontHeader	   = 0.45f;
		fontBody	   = 0.45f;
		key_fontHeader = 0.45f;
		key_fontBody   = 0.4f;
		log_fontHeader = 0.45f;
		log_fontBody   = 0.4f;
	}
	std::cout << "DisplayClass: Display initialized.\n";
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
 *  ==================================================
 *  ================================================== 
 * 
 *  UU   UU  PPPPP    DDDDD      AAA    TTTTTT  EEEEEE 
 *  UU   UU  PP   PP  DD   DD   AA AA     TT    EE    
 *  UU   UU  PP   PP  DD   DD   AA AA     TT    EE  
 *  UU   UU  PPPPP    DD   DD  AAAAAAA    TT    EEEEE    
 *  UU   UU  PP       DD   DD  AA   AA    TT    EE    
 *  UU   UU  PP       DD   DD  AA   AA    TT    EE    
 *   UUUUU   PP       DDDDD    AA   AA    TT    EEEEEE 
 * 
 *  ================================================== 
 *  ================================================== 
*/


/**
 * @brief Updates the information on the display
 */
void DisplayClass::Update() {

	// Clear overlay frame
	// shared->matFrameOverlay = 0;

	// Copy video frame to overlay
	shared->Capture.matFrameUndistorted.copyTo( shared->Display.matFrameOverlay( cv::Rect( 0, 0, shared->Capture.matFrameUndistorted.cols, shared->Capture.matFrameUndistorted.rows ) ) );

	// Add camera elements
	AddCameraElements();

	// Add PID elements
	// AddPidElements();

	// Add gain elements
	AddGainElements();

	// Add text
	BuildReadoutInterface();

	// Add log
	// BuildLogInterface();

	// Show interface
	ShowInterface();
}



/**
 * Add text to the overlay
 */
void DisplayClass::BuildReadoutInterface() {

	// Draw black box at bottom of screen // y=1100

	cv::rectangle( shared->Display.matFrameOverlay, cv::Rect( 0, 1100, 1600, 1360 ), CONFIG_colBlack, -1 );

	// Section border
	DrawCell( "", "A1", 40, 10, 0, CONFIG_colWhite, CONFIG_colBlack, false );

	// Telemetry
	AddTextTelemetry();

	// Controller
	AddTextController();

	// System flags
	AddTextSystem();

	// Amplifier
	AddTextAmplifier();

	// Serial
	AddTextSerial();

	// Motor
	AddTextMotorOutput();

	// Task
	AddTextTask();

	// Status block
	DrawCell( shared->Display.statusString, "AO9", 10, 2, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );

	// Section borders
	DrawCellBorder( "A1", 8, 8, 2, CONFIG_colWhite );	   // Telemetry
	DrawCellBorder( "I1", 17, 7, 2, CONFIG_colWhite );	   // Controller
	DrawCellBorder( "Z1", 15, 10, 2, CONFIG_colWhite );	   // Amplifier
	DrawCellBorder( "AO1", 10, 8, 2, CONFIG_colWhite );	   // Task
	DrawCellBorder( "A9", 25, 2, 2, CONFIG_colWhite );	   // System status text
	DrawCellBorder( "I8", 3, 1, 2, CONFIG_colWhite );	   // System flags: Limits
	DrawCellBorder( "P8", 6, 1, 2, CONFIG_colWhite );	   // System flags: Serial
	DrawCellBorder( "AO9", 10, 2, 2, CONFIG_colWhite );	   // Serial packets

	// Line on the right side to satisfy my OCD
	cv::line( shared->Display.matFrameOverlay, cv::Point2i( 1598, CONFIG_PANEL_HEIGHT ), cv::Point2i( 1598, 1360 ), CONFIG_colWhite, 1 );
}



void DisplayClass::AddStaticDisplayPanels() {

	// Build shortcuts
	BuildKeyboardShortcuts();

	// Build instructions
	BuildChecklist();
}

/**
 * Display the interface on the screen
 */
void DisplayClass::ShowInterface() {

	// Catch mouse
	// cv::setMouseCallback( winInterface, DisplayClass::onMouse, this );


	// Show image
	cv::imshow( winInterface, shared->Display.matFrameOverlay );
	// cv::imshow( "Raw", shared->matFrameUndistorted );

	// Output confirmation
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
 * 
 *  
 * =========================================================================================
 *  ========================================================================================= 
 * 
 *   EEEEEEE  LL     EEEEEEE  MM   MM  EEEEEEE  NN   NN  TTTTTTTT   SSSSS
 *   EE       LL     EE       MMM MMM  EE       NNN  NN     TT     SS
 *   EE       LL     EE       MM M MM  EE       NN N NN     TT     SS
 *   EEEEE    LL     EEEEE    MM   MM  EEEEE    NN  NNN     TT      SSSSS
 *   EE       LL     EE       MM   MM  EE       NN   NN     TT          SS
 *   EE       LL     EE       MM   MM  EE       NN   NN     TT          SS
 *   EEEEEEE  LLLLL  EEEEEEE  MM   MM  EEEEEEE  NN   NN     TT      SSSSS
 * 
 *  ========================================================================================= 
 *  ========================================================================================= 
 */


void DisplayClass::BuildLogInterface() {


	// Create window
	cv::namedWindow( winChecklist, cv::WINDOW_AUTOSIZE );
	cv::moveWindow( winChecklist, 3440 - CONFIG_DIS_WIDTH - CONFIG_DIS_KEY_WIDTH - CONFIG_DIS_LOG_WIDTH - 6, 0 );

	// Log info
	// Run Number, Error X , Error Y, Time
	DrawChecklistCell( "#", "A1", 1, 1, log_fontBody, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawChecklistCell( "ErrX", "B1", 2, 1, log_fontBody, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawChecklistCell( "ErrY", "D1", 2, 1, log_fontBody, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawChecklistCell( "|Err|", "F1", 2, 1, log_fontBody, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawChecklistCell( "Time", "H1", 2, 1, log_fontBody, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawChecklistCell( "Abb", "J1", 1, 1, log_fontBody, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawChecklistCell( "Add", "K1", 1, 1, log_fontBody, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawChecklistCell( "Ext", "L1", 1, 1, log_fontBody, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawChecklistCell( "Flx", "M1", 1, 1, log_fontBody, CONFIG_colWhite, CONFIG_colGraDk, true );

	// DrawChecklistCell( "Establish serial connection", "A1", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "Set tensioning values", "A2", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Tension motor A", "A3", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Tension motor B", "A4", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Tension motor C", "A5", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Stop tensioning", "A6", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "Set torque limits", "A7", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Set torque limit A", "A8", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Set torque limit B", "A9", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Set torque limit C", "A10", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Zero torque limits", "A11", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "Start Fitts task", "A12", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "Tune PID controller", "A13", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Adjust proportional gain", "A14", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Adjust integral gain", "A15", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Adjust derivative gain", "A16", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Select abduction direction", "A17", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Select adduction direction", "A18", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Select extension direction", "A19", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Select flexion direction", "A20", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// DrawChecklistCell( "   Zero all gains", "A21", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );

	// DrawChecklistCell( "s", "F1", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "t", "F2", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "=", "F3", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "/", "F4", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "*", "F5", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "t", "F6", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "0", "F7", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "7", "F8", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "9", "F9", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "5", "F10", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "f", "F11", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "CR", "F12", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "", "F13", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "p", "F14", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "i", "F15", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "d", "F16", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "4", "F17", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "6", "F18", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "8", "F19", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( "2", "F20", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	// DrawChecklistCell( ".", "F21", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );


	// Display window
	cv::imshow( winChecklist, matChecklist );
}



/**
 * @brief Add camera elements to the main display
 * 
 * @return * void 
 */
void DisplayClass::AddCameraElements() {

	// Draw detector crosshairs, changing colors based on if the target is present
	cv::circle( shared->Display.matFrameOverlay, CONFIG_CAM_CENTER, CONFIG_DET_RADIUS, ( shared->Target.isTargetFound ? CONFIG_colGreMd : CONFIG_colRedDk ), 1 );
	cv::circle( shared->Display.matFrameOverlay, CONFIG_CAM_CENTER, shared->Controller.integrationRadius, ( shared->Target.isTargetFound ? CONFIG_colYelDk : CONFIG_colRedDk ), 1 );
	cv::line( shared->Display.matFrameOverlay, cv::Point2i( CONFIG_CAM_PRINCIPAL_X, 0 ), cv::Point2i( CONFIG_CAM_PRINCIPAL_X, CONFIG_CAM_HEIGHT ), ( shared->Target.isTargetFound ? CONFIG_colGreMd : CONFIG_colRedDk ), 1 );
	cv::line( shared->Display.matFrameOverlay, cv::Point2i( 0, CONFIG_CAM_PRINCIPAL_Y ), cv::Point2i( CONFIG_CAM_WIDTH, CONFIG_CAM_PRINCIPAL_Y ), ( shared->Target.isTargetFound ? CONFIG_colGreMd : CONFIG_colRedDk ), 1 );

	// Draw motor axis
	cv::line( shared->Display.matFrameOverlay, CONFIG_CAM_CENTER, cv::Point2i( CONFIG_CAM_PRINCIPAL_X + COS35 * CONFIG_DET_RADIUS, CONFIG_CAM_PRINCIPAL_Y - SIN35 * CONFIG_DET_RADIUS ), CONFIG_colYelMd, 1 );
	cv::line( shared->Display.matFrameOverlay, CONFIG_CAM_CENTER, cv::Point2i( CONFIG_CAM_PRINCIPAL_X + COS145 * CONFIG_DET_RADIUS, CONFIG_CAM_PRINCIPAL_Y - SIN145 * CONFIG_DET_RADIUS ), CONFIG_colYelMd, 1 );
	cv::line( shared->Display.matFrameOverlay, CONFIG_CAM_CENTER, cv::Point2i( CONFIG_CAM_PRINCIPAL_X + COS270 * CONFIG_DET_RADIUS, CONFIG_CAM_PRINCIPAL_Y - SIN270 * CONFIG_DET_RADIUS ), CONFIG_colYelMd, 1 );

	// Draw information for target marker if present
	if ( shared->Target.isTargetFound ) {

		//Draw vector to center of target
		cv::line( shared->Display.matFrameOverlay, CONFIG_CAM_CENTER, shared->Target.screenPositionPX, CONFIG_colCyaMd, 2 );

		// Draw border and axis elements of target marker
		cv::polylines( shared->Display.matFrameOverlay, shared->Target.cornersPX, true, CONFIG_colGreMd, 2 );
		// cv::drawFrameAxes( shared->matFrameUndistorted, CONFIG_CAMERA_MATRIX, CONFIG_DISTORTION_COEFFS, shared->targetMarkerRotationVector, shared->targetMarkerTranslationVector, CONFIG_LARGE_MARKER_WIDTH, 15 );

		// Draw velocity
		cv::line( shared->Display.matFrameOverlay, CONFIG_CAM_CENTER, cv::Point2i( CONFIG_CAM_CENTER.x - shared->Target.velocityFilteredNewMM.x * MM2PX / 2.0f, CONFIG_CAM_CENTER.y + shared->Target.velocityFilteredNewMM.y * MM2PX / 2.0f ), CONFIG_colMagLt, 2 );
	}



	// Draw calibrated marker
	if ( shared->Calibration.isCalibrated ) {
		// int newX = shared->calibrationOffsetPX.x - CONFIG_TOUCHSCREEN_CENTER.x + CONFIG_CAM_PRINCIPAL_X;
		// int newY = shared->calibrationOffsetPX.y - CONFIG_TOUCHSCREEN_CENTER.y + CONFIG_CAM_PRINCIPAL_Y - 20 * MM2PX;
		// cv::circle( shared->Display.matFrameOverlay, cv::Point2i( newX, newY ), 10 * MM2PX, CONFIG_colGreLt, 2 );
		// cv::circle( shared->Display.matFrameOverlay, shared->, 10 * MM2PX, CONFIG_colGreLt, 2 );
	}


	// Add reverse indicator
	if ( shared->Controller.toggleReverse ) {
		cv::putText( shared->Display.matFrameOverlay, "REVERSE", cv::Point( 10, 40 ), cv::FONT_HERSHEY_SIMPLEX, 1.0f, CONFIG_colRedMd, 2 );
	} else {
		cv::putText( shared->Display.matFrameOverlay, "FORWARD", cv::Point( 10, 40 ), cv::FONT_HERSHEY_SIMPLEX, 1.0f, CONFIG_colGreMd, 2 );
	}
}



void DisplayClass::AddGainElements() {

	// Gains viz
	// cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x, center.y + shared->Controller.gainKp.ext * 20 ), CONFIG_colBluLt, 4 );


	// PID Variables
	uint8_t		rad = 100;
	cv::Point2i center( 1480, 120 );
	cv::Point2i gainAbd = cv::Point2i( center.x - shared->Controller.gainKp.abd * rad / 4, center.y );
	cv::Point2i gainAdd = cv::Point2i( center.x + shared->Controller.gainKp.add * rad / 4, center.y );
	cv::Point2i gainExt = cv::Point2i( center.x, center.y - shared->Controller.gainKp.ext * rad / 4 );
	cv::Point2i gainFlx = cv::Point2i( center.x, center.y + shared->Controller.gainKp.flx * rad / 4 );

	// Base Prop
	cv::circle( shared->Display.matFrameOverlay, center, rad, CONFIG_colGraBk, -1 );
	cv::circle( shared->Display.matFrameOverlay, center, rad / 4, CONFIG_colGraMd, 1 );
	cv::line( shared->Display.matFrameOverlay, cv::Point2i( center.x - rad, center.y ), cv::Point2i( center.x + rad, center.y ), CONFIG_colGraMd, 1 );
	cv::line( shared->Display.matFrameOverlay, cv::Point2i( center.x, center.y - rad ), cv::Point2i( center.x, center.y + rad ), CONFIG_colGraMd, 1 );



	// Components Prop
	cv::line( shared->Display.matFrameOverlay, center, gainAbd, CONFIG_colCyaMd, 2 );
	cv::line( shared->Display.matFrameOverlay, center, gainAdd, CONFIG_colCyaMd, 2 );
	cv::line( shared->Display.matFrameOverlay, center, gainFlx, CONFIG_colCyaMd, 2 );
	cv::line( shared->Display.matFrameOverlay, center, gainExt, CONFIG_colCyaMd, 2 );
	// cv::line( shared->Display.matFrameOverlay, centerProp, propB, CONFIG_colCyaMd, 2 );
	// cv::line( shared->Display.matFrameOverlay, centerProp, propC, CONFIG_colCyaMd, 2 );
	// cv::circle( shared->Display.matFrameOverlay, propA, 5, CONFIG_colCyaMd, -1 );
	// cv::circle( shared->Display.matFrameOverlay, propB, 5, CONFIG_colCyaMd, -1 );
	// cv::circle( shared->Display.matFrameOverlay, propC, 5, CONFIG_colCyaMd, -1 );

	// Outline Prop

	cv::circle( shared->Display.matFrameOverlay, center, rad, CONFIG_colGraMd, 2 );
}

// void DisplayClass::AddPidElements() {

// 		// Gains viz
// 		cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x, center.y + shared->Controller.gainKp.ext * 20 ), CONFIG_colBluLt, 4 );


// 	// PID Variables
// 	uint8_t		rad = 80;
// 	cv::Point2i centerProp( 1500, 100 );
// 	cv::Point2i centerInt( 1500, 300 );
// 	cv::Point2i centerDeriv( 1500, 500 );
// 	cv::Point2i propA = cv::Point2i( centerProp.x + shared->Controller.percentageProportional.x * COS35 * rad, centerProp.y - shared->Controller.percentageProportional.x * SIN35 * rad );
// 	cv::Point2i propB = cv::Point2i( centerProp.x + shared->Controller.percentageProportional.y * COS145 * rad, centerProp.y - shared->Controller.percentageProportional.y * SIN145 * rad );
// 	cv::Point2i propC = cv::Point2i( centerProp.x + shared->Controller.percentageProportional.z * COS270 * rad, centerProp.y - shared->Controller.percentageProportional.z * SIN270 * rad );
// 	cv::Point2i intA  = cv::Point2i( centerInt.x + shared->Controller.percentageIntegral.x * COS35 * rad, centerInt.y - shared->Controller.percentageIntegral.x * SIN35 * rad );
// 	cv::Point2i intB  = cv::Point2i( centerInt.x + shared->Controller.percentageIntegral.y * COS145 * rad, centerInt.y - shared->Controller.percentageIntegral.y * SIN145 * rad );
// 	cv::Point2i intC  = cv::Point2i( centerInt.x + shared->Controller.percentageIntegral.z * COS270 * rad, centerInt.y - shared->Controller.percentageIntegral.z * SIN270 * rad );
// 	cv::Point2i derA  = cv::Point2i( centerDeriv.x + shared->Controller.percentageDerivative.x * COS35 * rad, centerDeriv.y - shared->Controller.percentageDerivative.x * SIN35 * rad );
// 	cv::Point2i derB  = cv::Point2i( centerDeriv.x + shared->Controller.percentageDerivative.y * COS145 * rad, centerDeriv.y - shared->Controller.percentageDerivative.y * SIN145 * rad );
// 	cv::Point2i derC  = cv::Point2i( centerDeriv.x + shared->Controller.percentageDerivative.z * COS270 * rad, centerDeriv.y - shared->Controller.percentageDerivative.z * SIN270 * rad );

// 	// Base Prop
// 	cv::circle( shared->Display.matFrameOverlay, centerProp, rad, CONFIG_colGraBk, -1 );
// 	cv::line( shared->Display.matFrameOverlay, centerProp, cv::Point2i( centerProp.x + rad * COS35, centerProp.y - rad * SIN35 ), CONFIG_colGraMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerProp, cv::Point2i( centerProp.x + rad * COS145, centerProp.y - rad * SIN145 ), CONFIG_colGraMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerProp, cv::Point2i( centerProp.x + rad * COS270, centerProp.y - rad * SIN270 ), CONFIG_colGraMd, 2 );

// 	// Components Prop
// 	cv::line( shared->Display.matFrameOverlay, centerProp, propA, CONFIG_colCyaMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerProp, propB, CONFIG_colCyaMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerProp, propC, CONFIG_colCyaMd, 2 );
// 	cv::circle( shared->Display.matFrameOverlay, propA, 5, CONFIG_colCyaMd, -1 );
// 	cv::circle( shared->Display.matFrameOverlay, propB, 5, CONFIG_colCyaMd, -1 );
// 	cv::circle( shared->Display.matFrameOverlay, propC, 5, CONFIG_colCyaMd, -1 );

// 	// Outline Prop
// 	cv::circle( shared->Display.matFrameOverlay, centerProp, rad, CONFIG_colGraMd, 2 );


// 	// Base Int
// 	cv::circle( shared->Display.matFrameOverlay, centerInt, rad, CONFIG_colGraBk, -1 );
// 	cv::line( shared->Display.matFrameOverlay, centerInt, cv::Point2i( centerInt.x + rad * COS35, centerInt.y - rad * SIN35 ), CONFIG_colGraMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerInt, cv::Point2i( centerInt.x + rad * COS145, centerInt.y - rad * SIN145 ), CONFIG_colGraMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerInt, cv::Point2i( centerInt.x + rad * COS270, centerInt.y - rad * SIN270 ), CONFIG_colGraMd, 2 );

// 	// Components Int
// 	cv::line( shared->Display.matFrameOverlay, centerInt, intA, CONFIG_colYelMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerInt, intB, CONFIG_colYelMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerInt, intC, CONFIG_colYelMd, 2 );
// 	cv::circle( shared->Display.matFrameOverlay, intA, 5, CONFIG_colYelMd, -1 );
// 	cv::circle( shared->Display.matFrameOverlay, intB, 5, CONFIG_colYelMd, -1 );
// 	cv::circle( shared->Display.matFrameOverlay, intC, 5, CONFIG_colYelMd, -1 );

// 	// Outline Int
// 	cv::circle( shared->Display.matFrameOverlay, centerInt, rad, CONFIG_colGraMd, 2 );


// 	// Base Deriv
// 	cv::circle( shared->Display.matFrameOverlay, centerDeriv, rad, CONFIG_colGraBk, -1 );
// 	cv::line( shared->Display.matFrameOverlay, centerDeriv, cv::Point2i( centerDeriv.x + rad * COS35, centerDeriv.y - rad * SIN35 ), CONFIG_colGraMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerDeriv, cv::Point2i( centerDeriv.x + rad * COS145, centerDeriv.y - rad * SIN145 ), CONFIG_colGraMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerDeriv, cv::Point2i( centerDeriv.x + rad * COS270, centerDeriv.y - rad * SIN270 ), CONFIG_colGraMd, 2 );

// 	// Components Deriv
// 	cv::line( shared->Display.matFrameOverlay, centerDeriv, derA, CONFIG_colRedMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerDeriv, derB, CONFIG_colRedMd, 2 );
// 	cv::line( shared->Display.matFrameOverlay, centerDeriv, derC, CONFIG_colRedMd, 2 );
// 	cv::circle( shared->Display.matFrameOverlay, derA, 5, CONFIG_colRedMd, -1 );
// 	cv::circle( shared->Display.matFrameOverlay, derB, 5, CONFIG_colRedMd, -1 );
// 	cv::circle( shared->Display.matFrameOverlay, derC, 5, CONFIG_colRedMd, -1 );

// 	// Outline Deriv
// 	cv::circle( shared->Display.matFrameOverlay, centerDeriv, rad, CONFIG_colGraMd, 2 );
// }



void DisplayClass::AddTextTelemetry() {

	// Telemetry
	// Position
	DrawCell( "TELEMETRY", "A1", 8, 1, fontTitle, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawCell( std::to_string( shared->Target.activeID ), "A2", 2, 2, fontBody * 3, CONFIG_colWhite, ( shared->Target.isTargetFound ? CONFIG_colGreBk : CONFIG_colBlack ), true );
	DrawCell( "Position", "C2", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[mm]", "C3", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "x", "A4", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "y", "A5", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "z", "A6", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "Rxy", "A7", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "Rxyz", "A8", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( std::to_string( int( shared->Target.positionFilteredNewMM.x ) ), "C4", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( std::to_string( int( shared->Target.positionFilteredNewMM.y ) ), "C5", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( std::to_string( int( shared->Target.positionFilteredNewMM.z ) ), "C6", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( std::to_string( int( shared->GetNorm2D( cv::Point2f( shared->Target.positionFilteredNewMM.x, shared->Target.positionFilteredNewMM.y ) ) ) ), "C7", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( std::to_string( int( shared->GetNorm3D( shared->Target.positionFilteredNewMM ) ) ), "C8", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );

	// Velocity
	DrawCell( "Velocity", "E2", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[mm/s]", "E3", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( std::to_string( int( shared->Target.velocityFilteredNewMM.x ) ), "E4", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( std::to_string( int( shared->Target.velocityFilteredNewMM.y ) ), "E5", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( std::to_string( int( shared->Target.velocityFilteredNewMM.z ) ), "E6", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( std::to_string( int( shared->GetNorm2D( cv::Point2f( shared->Target.velocityFilteredNewMM.x, shared->Target.velocityFilteredNewMM.y ) ) ) ), "E7", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( std::to_string( int( shared->GetNorm3D( shared->Target.velocityFilteredNewMM ) ) ), "E8", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );

	// Integrated Error
	DrawCell( "Integr.", "G2", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[mm]", "G3", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Target.positionIntegratedMM.x, 1, 0 ), "G4", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( shared->Target.positionIntegratedMM.y, 1, 0 ), "G5", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( shared->Target.positionIntegratedMM.z, 1, 0 ), "G6", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( "xx", "G7", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( "xx", "G8", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
}


void DisplayClass::AddTextController() {


	// Handler for gains
	auto& system	= shared->Input.selectedAdjustmentSystem;
	auto& subsystem = shared->Input.selectedAdjustmentSubsystem;

	// Controller
	DrawCell( "CONTROLLER", "I1", 17, 1, fontTitle, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawCell( "Freq", "I2", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( std::to_string( int( shared->Timing.measuredFrequency ) ), "I3", 2, 1, fontBody, CONFIG_colWhite, ( shared->Timing.measuredFrequency > 60 ? CONFIG_colGreBk : CONFIG_colRedBk ), true );

	// Direction
	DrawCell( "ABD", "I4", 2, 1, fontHeader, CONFIG_colWhite, ( subsystem == selectSubsystemEnum::ABD ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( "ADD", "I5", 2, 1, fontHeader, CONFIG_colWhite, ( subsystem == selectSubsystemEnum::ADD ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( "FLEX", "I6", 2, 1, fontHeader, CONFIG_colWhite, ( subsystem == selectSubsystemEnum::FLEX ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( "EXT", "I7", 2, 1, fontHeader, CONFIG_colWhite, ( subsystem == selectSubsystemEnum::EXT ? CONFIG_colYelDk : CONFIG_colGraBk ), true );

	// Proportional
	DrawCell( "Kp", "K2", 2, 1, fontHeader, CONFIG_colWhite, ( system == selectSystemEnum::GAIN_PROPORTIONAL ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( "[u]", "K3", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKp.abd, 1, 1 ), "K4", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_PROPORTIONAL ) && ( subsystem == selectSubsystemEnum::ABD ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKp.add, 1, 1 ), "K5", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_PROPORTIONAL ) && ( subsystem == selectSubsystemEnum::ADD ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKp.flx, 1, 1 ), "K6", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_PROPORTIONAL ) && ( subsystem == selectSubsystemEnum::FLEX ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKp.ext, 1, 1 ), "K7", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_PROPORTIONAL ) && ( subsystem == selectSubsystemEnum::EXT ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );

	// Integral
	DrawCell( "Ki", "M2", 2, 1, fontHeader, CONFIG_colWhite, ( system == selectSystemEnum::GAIN_INTEGRAL ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( "[u]", "M3", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKi.abd, 1, 1 ), "M4", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_INTEGRAL ) && ( subsystem == selectSubsystemEnum::ABD ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKi.add, 1, 1 ), "M5", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_INTEGRAL ) && ( subsystem == selectSubsystemEnum::ADD ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKi.flx, 1, 1 ), "M6", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_INTEGRAL ) && ( subsystem == selectSubsystemEnum::FLEX ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKi.ext, 1, 1 ), "M7", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_INTEGRAL ) && ( subsystem == selectSubsystemEnum::EXT ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );

	// Derivative
	DrawCell( "Kd", "O2", 2, 1, fontHeader, CONFIG_colWhite, ( system == selectSystemEnum::GAIN_DERIVATIVE ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( "[u]", "O3", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKd.abd, 1, 2 ), "O4", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_DERIVATIVE ) && ( subsystem == selectSubsystemEnum::ABD ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKd.add, 1, 2 ), "O5", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_DERIVATIVE ) && ( subsystem == selectSubsystemEnum::ADD ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKd.flx, 1, 2 ), "O6", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_DERIVATIVE ) && ( subsystem == selectSubsystemEnum::FLEX ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( shared->FormatDecimal( shared->Controller.gainKd.ext, 1, 2 ), "O7", 2, 1, fontBody, CONFIG_colWhite, ( ( system == selectSystemEnum::GAIN_DERIVATIVE ) && ( subsystem == selectSubsystemEnum::EXT ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );

	// Term axis
	DrawCell( "x", "Q4", 1, 2, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "y", "Q6", 1, 2, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );

	// P Term
	DrawCell( "Kp*E", "R2", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[u]", "R3", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Controller.proportionalTerm.x, 1, 1 ), "R4", 2, 2, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( shared->Controller.proportionalTerm.y, 1, 1 ), "R6", 2, 2, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );

	// I Term
	DrawCell( "Ki*IE", "T2", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[u]", "T3", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Controller.integralTerm.x, 1, 1 ), "T4", 2, 2, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( shared->Controller.integralTerm.y, 1, 1 ), "T6", 2, 2, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );

	// D Term
	DrawCell( "Kp*dE", "V2", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[u]", "V3", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Controller.derivativeTerm.x, 1, 1 ), "V4", 2, 2, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( shared->Controller.derivativeTerm.y, 1, 1 ), "V6", 2, 2, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );

	// Total
	DrawCell( "Total", "X2", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[u]", "X3", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Controller.combinedPIDTerms.x, 1, 1 ), "X4", 2, 2, fontBody * 1.5f, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( shared->Controller.combinedPIDTerms.y, 1, 1 ), "X6", 2, 2, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
}


void DisplayClass::AddTextSystem() {

	// System Flags
	DrawCell( shared->Amplifier.isLimitSet ? "Limited" : "No Limit", "I8", 3, 1, fontHeader, CONFIG_colWhite, shared->Amplifier.isLimitSet ? CONFIG_colGreBk : CONFIG_colRedBk, true );
	DrawCell( shared->Logging.isEnabled ? "Logging On" : "Logging Off", "L8", 4, 1, fontHeader, CONFIG_colWhite, shared->Logging.isEnabled ? CONFIG_colGreBk : CONFIG_colRedBk, true );
	DrawCell( "Teensy In", "P8", 3, 1, fontHeader, CONFIG_colWhite, shared->Serial.isSerialReceiveOpen ? CONFIG_colGreBk : CONFIG_colRedBk, true );
	DrawCell( "Teensy Out", "S8", 3, 1, fontHeader, CONFIG_colWhite, shared->Serial.isSerialSendOpen ? CONFIG_colGreBk : CONFIG_colRedBk, true );

	if ( shared->Amplifier.isAmplifierActive ) {
		if ( shared->Amplifier.isTensionOnly ) {

			DrawCell( "Tension Only ", "V8", 4, 1, fontHeader, CONFIG_colWhite, shared->Amplifier.isAmplifierActive ? CONFIG_colGreBk : CONFIG_colRedBk, true );
		} else {
			DrawCell( "Driving ", "V8", 4, 1, fontHeader, CONFIG_colWhite, shared->Amplifier.isAmplifierActive ? CONFIG_colGreBk : CONFIG_colRedBk, true );
		}
	} else {
		DrawCell( "Amplifier Off ", "V8", 4, 1, fontHeader, CONFIG_colWhite, shared->Amplifier.isAmplifierActive ? CONFIG_colGreBk : CONFIG_colRedBk, true );
	}
}


void DisplayClass::AddTextAmplifier() {

	// Handler for gains
	auto& system	= shared->Input.selectedAdjustmentSystem;
	auto& subsystem = shared->Input.selectedAdjustmentSubsystem;

	// Amplifier
	DrawCell( "AMPLIFIER", "Z1", 15, 1, fontTitle, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawCell( ( shared->Amplifier.isSafetySwitchEngaged ? "Safe" : "Released" ), "Z2", 3, 1, fontBody * 0.7f, CONFIG_colWhite, ( shared->Amplifier.isSafetySwitchEngaged ? CONFIG_colGreDk : CONFIG_colRedBk ), true );

	// Motor selection
	DrawCell( "Motor", "Z3", 3, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "A", "AC2", 2, 1, fontHeader, CONFIG_colWhite, ( ( ( ( system == selectSystemEnum::AMP_LIMIT ) || ( system == selectSystemEnum::AMP_TENSION ) ) && ( ( subsystem == selectSubsystemEnum::AMP_A ) || ( subsystem == selectSubsystemEnum::ALL ) ) ) ? CONFIG_colYelDk : CONFIG_colGraBk ),
			  true );
	DrawCell( "B", "AE2", 2, 1, fontHeader, CONFIG_colWhite, ( ( ( ( system == selectSystemEnum::AMP_LIMIT ) || ( system == selectSystemEnum::AMP_TENSION ) ) && ( ( subsystem == selectSubsystemEnum::AMP_B ) || ( subsystem == selectSubsystemEnum::ALL ) ) ) ? CONFIG_colYelDk : CONFIG_colGraBk ),
			  true );
	DrawCell( "C", "AG2", 2, 1, fontHeader, CONFIG_colWhite, ( ( ( ( system == selectSystemEnum::AMP_LIMIT ) || ( system == selectSystemEnum::AMP_TENSION ) ) && ( ( subsystem == selectSubsystemEnum::AMP_C ) || ( subsystem == selectSubsystemEnum::ALL ) ) ) ? CONFIG_colYelDk : CONFIG_colGraBk ),
			  true );

	// Tension
	DrawCell( "Tension", "Z3", 2, 1, fontHeader, CONFIG_colWhite, ( ( system == selectSystemEnum::AMP_TENSION ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( "[%]", "AB3", 1, 1, fontHeader * 0.8f, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Controller.commandedTensionABC.x * 100.0, 3, 1 ), "AC3", 2, 1, fontBody, CONFIG_colWhite,
			  ( ( system == selectSystemEnum::AMP_TENSION ) && ( ( subsystem == selectSubsystemEnum::AMP_A ) || ( subsystem == selectSubsystemEnum::ALL ) ) ? CONFIG_colYelDk : CONFIG_colBlack ), true );
	DrawCell( shared->FormatDecimal( shared->Controller.commandedTensionABC.y * 100.0, 3, 1 ), "AE3", 2, 1, fontBody, CONFIG_colWhite,
			  ( ( system == selectSystemEnum::AMP_TENSION ) && ( ( subsystem == selectSubsystemEnum::AMP_B ) || ( subsystem == selectSubsystemEnum::ALL ) ) ? CONFIG_colYelDk : CONFIG_colBlack ), true );
	DrawCell( shared->FormatDecimal( shared->Controller.commandedTensionABC.z * 100.0, 3, 1 ), "AG3", 2, 1, fontBody, CONFIG_colWhite,
			  ( ( system == selectSystemEnum::AMP_TENSION ) && ( ( subsystem == selectSubsystemEnum::AMP_C ) || ( subsystem == selectSubsystemEnum::ALL ) ) ? CONFIG_colYelDk : CONFIG_colBlack ), true );

	// Drive command
	DrawCell( "Drive", "Z4", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[%]", "AB4", 1, 1, fontHeader * 0.6f, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( ( shared->Controller.commandedPercentageABC.x - shared->Controller.commandedTensionABC.x ) * 100.0, 3, 1 ), "AC4", 2, 1, fontBody, ( shared->Amplifier.isTensionOnly ? CONFIG_colGraDk : CONFIG_colWhite ), CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( ( shared->Controller.commandedPercentageABC.y - shared->Controller.commandedTensionABC.y ) * 100.0, 3, 1 ), "AE4", 2, 1, fontBody, ( shared->Amplifier.isTensionOnly ? CONFIG_colGraDk : CONFIG_colWhite ), CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( ( shared->Controller.commandedPercentageABC.z - shared->Controller.commandedTensionABC.z ) * 100.0, 3, 1 ), "AG4", 2, 1, fontBody, ( shared->Amplifier.isTensionOnly ? CONFIG_colGraDk : CONFIG_colWhite ), CONFIG_colBlack, true );

	// Total command
	DrawCell( "Total", "Z5", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[%]", "AB5", 1, 1, fontHeader * 0.6f, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Controller.commandedPercentageABC.x * 100.0, 3, 1 ), "AC5", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( shared->Controller.commandedPercentageABC.y * 100.0, 3, 1 ), "AE5", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( shared->Controller.commandedPercentageABC.z * 100.0, 3, 1 ), "AG5", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );

	// Max command
	DrawCell( "Max", "Z6", 2, 1, fontHeader, CONFIG_colWhite, ( ( system == selectSystemEnum::AMP_LIMIT ) ? CONFIG_colYelDk : CONFIG_colGraBk ), true );
	DrawCell( "[%]", "AB6", 1, 1, fontHeader * 0.6f, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.commandedLimits.x * 100.0f, 3, 1 ), "AC6", 2, 1, fontBody, CONFIG_colWhite,
			  ( ( system == selectSystemEnum::AMP_LIMIT ) && ( ( subsystem == selectSubsystemEnum::AMP_A ) || ( subsystem == selectSubsystemEnum::ALL ) ) ? CONFIG_colYelDk : CONFIG_colBlack ), true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.commandedLimits.y * 100.0f, 3, 1 ), "AE6", 2, 1, fontBody, CONFIG_colWhite,
			  ( ( system == selectSystemEnum::AMP_LIMIT ) && ( ( subsystem == selectSubsystemEnum::AMP_B ) || ( subsystem == selectSubsystemEnum::ALL ) ) ? CONFIG_colYelDk : CONFIG_colBlack ), true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.commandedLimits.z * 100.0f, 3, 1 ), "AG6", 2, 1, fontBody, CONFIG_colWhite,
			  ( ( system == selectSystemEnum::AMP_LIMIT ) && ( ( subsystem == selectSubsystemEnum::AMP_C ) || ( subsystem == selectSubsystemEnum::ALL ) ) ? CONFIG_colYelDk : CONFIG_colBlack ), true );

	// PWM mapping
	DrawCell( "PWM", "Z7", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[DC]", "AB7", 1, 1, fontHeader * 0.6f, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( std::to_string( shared->Controller.commandedPwmABC.x ), "AC7", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( std::to_string( shared->Controller.commandedPwmABC.y ), "AE7", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( std::to_string( shared->Controller.commandedPwmABC.z ), "AG7", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );

	// Current mapping
	DrawCell( "Current", "Z8", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[A]", "AB8", 1, 1, fontHeader * 0.6f, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.currentMeasuredAmpsA, 1, 2 ), "AC8", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.currentMeasuredAmpsB, 1, 2 ), "AE8", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.currentMeasuredAmpsC, 1, 2 ), "AG8", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );

	// Motor angles
	DrawCell( "Angle", "Z9", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[deg]", "AB9", 1, 1, fontHeader * 0.6f, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.encoderMeasuredDegA, 2, 2 ), "AC9", 2, 1, fontBody, CONFIG_colWhite, ( ( shared->Amplifier.isLimitSet && shared->Amplifier.isOverLimitA ) ? CONFIG_colRedDk : CONFIG_colGreBk ), true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.encoderMeasuredDegB, 2, 2 ), "AE9", 2, 1, fontBody, CONFIG_colWhite, ( ( shared->Amplifier.isLimitSet && shared->Amplifier.isOverLimitB ) ? CONFIG_colRedDk : CONFIG_colGreBk ), true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.encoderMeasuredDegC, 2, 2 ), "AG9", 2, 1, fontBody, CONFIG_colWhite, ( ( shared->Amplifier.isLimitSet && shared->Amplifier.isOverLimitC ) ? CONFIG_colRedDk : CONFIG_colGreBk ), true );

	// Motor angle limits
	DrawCell( "Limit", "Z10", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( "[deg]", "AB10", 1, 1, fontHeader * 0.6f, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.encoderLimitDegA, 2, 2 ), "AC10", 2, 1, fontBody, CONFIG_colWhite, ( shared->Amplifier.isMeasuringEncoderLimit ? CONFIG_colYelDk : CONFIG_colBlack ), true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.encoderLimitDegB, 2, 2 ), "AE10", 2, 1, fontBody, CONFIG_colWhite, ( shared->Amplifier.isMeasuringEncoderLimit ? CONFIG_colYelDk : CONFIG_colBlack ), true );
	DrawCell( shared->FormatDecimal( shared->Amplifier.encoderLimitDegC, 2, 2 ), "AG10", 2, 1, fontBody, CONFIG_colWhite, ( shared->Amplifier.isMeasuringEncoderLimit ? CONFIG_colYelDk : CONFIG_colBlack ), true );
}



void DisplayClass::AddTextSerial() {

	// Serial I/O
	DrawCell( "PC Out", "A9", 2, 1, fontHeader, CONFIG_colWhite, shared->Serial.isSerialSending ? CONFIG_colGreBk : CONFIG_colRedBk, true );
	DrawCell( "PC In", "A10", 2, 1, fontHeader, CONFIG_colWhite, shared->Serial.isSerialReceiving ? CONFIG_colGreBk : CONFIG_colRedBk, true );
	DrawCell( shared->Serial.isSerialSending ? shared->Serial.packetOut : "Not sending", "C9", 23, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawCell( shared->Serial.isSerialReceiving ? shared->Serial.packetIn : "Not receiving", "C10", 23, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	// Serial delay
	DrawCell( "Lag", "X9", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( std::to_string( shared->Serial.packetDelay ), "X10", 1, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( "[ms]", "Y10", 1, 1, fontBody * 0.6f, CONFIG_colWhite, CONFIG_colGraBk, true );
}


void DisplayClass::AddTextMotorOutput() {


	// Motor Output Block
	DrawCell( "", "AI2", 6, 7, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );

	cv::Point2i center( 1184, 1100 + 117 );
	float		motorR = 80.0f;

	limA = shared->Amplifier.commandedLimits.x * motorR;
	limB = shared->Amplifier.commandedLimits.y * motorR;
	limC = shared->Amplifier.commandedLimits.z * motorR;



	// Motor limits
	if ( shared->Controller.isLimitSet ) {
		cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x + COS35 * limA, center.y - SIN35 * limA ), CONFIG_colGraDk, 10 );
		cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x + COS145 * limB, center.y - SIN145 * limB ), CONFIG_colGraDk, 10 );
		cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x, center.y - SIN270 * limC ), CONFIG_colGraDk, 10 );
	}

	// Motor segments
	cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x + COS35 * motorR, center.y - SIN35 * motorR ), CONFIG_colGraLt, 1 );
	cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x + COS145 * motorR, center.y - SIN145 * motorR ), CONFIG_colGraLt, 1 );
	cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x, center.y - SIN270 * motorR ), CONFIG_colGraLt, 1 );



	// /// Lines connecting motor pairs
	// cv::line( shared->Display.matFrameOverlay, cv::Point2i( center.x + COS35 * ( shared->Controller.commandedPercentage.x * 60.0f ), center.y - SIN35 * ( shared->Controller.commandedPercentage.x * 60.0f ) ), cv::Point2i( center.x, center.y - SIN270 * ( shared->Controller.commandedPercentage.z * 60.0f ) ),
	// 		  CONFIG_colRedMd, 1 );
	// cv::line( shared->Display.matFrameOverlay, cv::Point2i( center.x + COS145 * ( shared->Controller.commandedPercentage.y * 60.0f ), center.y - SIN145 * ( shared->Controller.commandedPercentage.y * 60.0f ) ), cv::Point2i( center.x, center.y - SIN270 * ( shared->Controller.commandedPercentage.z * 60.0f ) ),
	// 		  CONFIG_colRedMd, 1 );
	// cv::line( shared->Display.matFrameOverlay, cv::Point2i( center.x + COS35 * ( shared->Controller.commandedPercentage.x * 60.0f ), center.y - SIN35 * ( shared->Controller.commandedPercentage.x * 60.0f ) ),
	// 		  cv::Point2i( center.x + COS145 * ( shared->Controller.commandedPercentage.y * 60.0f ), center.y - SIN145 * ( shared->Controller.commandedPercentage.y * 60.0f ) ), CONFIG_colRedMd, 1 );
	// Active lines
	cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x + std::clamp( float( COS35 * ( shared->Controller.commandedPercentageABC.x * motorR ) ), 0.0f, motorR ), center.y - std::clamp( float( SIN35 * ( shared->Controller.commandedPercentageABC.x * motorR ) ), 0.0f, motorR ) ),
			  CONFIG_colGreDk, 10 );
	cv::line( shared->Display.matFrameOverlay, center,
			  cv::Point2i( center.x + std::clamp( float( COS145 * ( shared->Controller.commandedPercentageABC.y * motorR ) ), -60.0f, motorR ), center.y - std::clamp( float( SIN145 * ( shared->Controller.commandedPercentageABC.y * motorR ) ), 0.0f, motorR ) ), CONFIG_colGreDk, 10 );
	cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x, center.y - SIN270 * ( shared->Controller.commandedPercentageABC.z * motorR ) ), CONFIG_colGreDk, 10 );

	// Teensy response lines
	if ( shared->Amplifier.isAmplifierActive ) {
		cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x + COS35 * ( shared->Amplifier.measuredPwmPercentA * motorR ), center.y - SIN35 * ( shared->Amplifier.measuredPwmPercentA * motorR ) ), CONFIG_colBluMd, 4 );
		cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x + COS145 * ( shared->Amplifier.measuredPwmPercentB * motorR ), center.y - SIN145 * ( shared->Amplifier.measuredPwmPercentB * motorR ) ), CONFIG_colBluMd, 4 );
		cv::line( shared->Display.matFrameOverlay, center, cv::Point2i( center.x, center.y - SIN270 * ( shared->Amplifier.measuredPwmPercentC * motorR ) ), CONFIG_colBluMd, 4 );
	}



	// Axis labels
	cv::putText( shared->Display.matFrameOverlay, "A", cv::Point2i( center.x + 58, center.y - 25 ), cv::FONT_HERSHEY_SIMPLEX, 0.5f, CONFIG_colRedLt, 1 );
	cv::putText( shared->Display.matFrameOverlay, "B", cv::Point2i( center.x - 67, center.y - 25 ), cv::FONT_HERSHEY_SIMPLEX, 0.5f, CONFIG_colGreLt, 1 );
	cv::putText( shared->Display.matFrameOverlay, "C", cv::Point2i( center.x + 10, center.y + 70 ), cv::FONT_HERSHEY_SIMPLEX, 0.5f, CONFIG_colBluLt, 1 );

	// Motor output circles
	cv::circle( shared->Display.matFrameOverlay, center, motorR, ( shared->Amplifier.isAmplifierActive ? CONFIG_colGreDk : CONFIG_colGraDk ), 2 );
	cv::circle( shared->Display.matFrameOverlay, center, ( shared->Amplifier.isAmplifierActive ? 6 : 2 ), ( shared->Amplifier.isAmplifierActive ? CONFIG_colGreDk : CONFIG_colGraDk ), -1 );


	// Encoder output block
	DrawCell( "", "AI9", 6, 2, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );

	// // Draw motor power box
	// cv::rectangle( shared->Display.matFrameOverlay, cv::Rect( 1100, 1320, 170, 26 ), CONFIG_colBluLt, 2 );



	// cv::Point2i encoderB( 1120, 1332 );
	// cv::Point2i encoderC( 1184, 1332 );
	// cv::Point2i encoderA( 1248, 1332 );
	// uint8_t		radEnc = 14;
	// // Angle lines
	// cv::line( shared->Display.matFrameOverlay, encoderA, cv::Point2i( encoderA.x + cos( shared->Amplifier.encoderMeasuredDegA * DEG2RAD * -1 ) * radEnc, encoderA.y + sin( shared->Amplifier.encoderMeasuredDegA * DEG2RAD * -1 ) * radEnc ), CONFIG_colRedLt, 2 );
	// cv::line( shared->Display.matFrameOverlay, encoderB, cv::Point2i( encoderB.x + cos( shared->Amplifier.encoderMeasuredDegB * DEG2RAD ) * radEnc, encoderB.y + sin( shared->Amplifier.encoderMeasuredDegB * DEG2RAD ) * radEnc ), CONFIG_colGreLt, 2 );
	// cv::line( shared->Display.matFrameOverlay, encoderC, cv::Point2i( encoderC.x + cos( shared->Amplifier.encoderMeasuredDegC * DEG2RAD ) * radEnc, encoderC.y + sin( shared->Amplifier.encoderMeasuredDegC * DEG2RAD ) * radEnc ), CONFIG_colBluLt, 2 );
	// cv::circle( shared->Display.matFrameOverlay, encoderA, 18, CONFIG_colGraDk, 2 );
	// cv::circle( shared->Display.matFrameOverlay, encoderB, 18, CONFIG_colGraDk, 2 );
	// cv::circle( shared->Display.matFrameOverlay, encoderC, 18, CONFIG_colGraDk, 2 );
}



void DisplayClass::AddTextTask() {

	// Task
	DrawCell( "TASK MONITOR", "AO1", 10, 1, fontTitle, CONFIG_colWhite, CONFIG_colGraDk, true );
	DrawCell( "Name", "AO2", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( shared->Task.isRunning ? shared->Task.name : "No task running", "AQ2", 4, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( "User ID", "AU2", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( std::to_string( shared->Task.userID ), "AW2", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( "Time", "AO3", 2, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( ( shared->Task.state == taskEnum::FITTS ) ? shared->FormatDecimal( shared->Task.elapsedTaskTime, 2, 3 ) : "0.00", "AQ3", 3, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( "Touchscreen", "AO8", 4, 1, fontHeader, CONFIG_colWhite, shared->Touchscreen.isTouched ? CONFIG_colGreBk : CONFIG_colGraBk, true );
	DrawCell( "x", "AS8", 1, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( std::to_string( shared->Touchscreen.positionTouched.x ), "AT8", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawCell( "y", "AV8", 1, 1, fontHeader, CONFIG_colWhite, CONFIG_colGraBk, true );
	DrawCell( std::to_string( shared->Touchscreen.positionTouched.y ), "AW8", 2, 1, fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
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
 * =========================================================================================
 *  ========================================================================================= 
 * 
 *   KK   KK EEEEEEE  YY    YY  BBBBBBB    OOOOO     AAA    RRRRR    DDDDD
 *   KK  KK  EE        YY  YY   BB   BBB  OO   OO   AA AA   RR   RR  DD   DD
 *   KK KK   EE         YYYY    BB   BBB  OO   OO   AA AA   RR   RR  DD   DD
 *   KKKK    EEEEE       YY     BBBBBBB   OO   OO  AAAAAAA  RRRRR    DD   DD
 *   KK KK   EE          YY     BB   BBB  OO   OO  AA   AA  RR  RR   DD   DD
 *   KK  KK  EE          YY     BB   BBB  OO   OO  AA   AA  RR   RR  DD   DD
 *   KK   KK EEEEEEE     YY     BBBBBBB    OOOOO   AA   AA  RR   RR  DDDDD
 * 
 *  ========================================================================================= 
 *  ========================================================================================= 
 */


void DisplayClass::BuildKeyboardShortcuts() {

	// Create window
	cv::namedWindow( winShortcuts, cv::WINDOW_AUTOSIZE );
	cv::moveWindow( winShortcuts, 3440 - CONFIG_DIS_WIDTH - CONFIG_DIS_KEY_WIDTH - 4, 0 );

	DrawKeyCell( "Exit", "A1", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "SetActive1", "A2", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "SetActive2", "A3", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "SetActive3", "A4", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "SetActive4", "A5", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "SetActive5", "A6", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "SetActiveNone", "A7", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "AmplifierToggle", "A8", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "AmplifierTensionToggle", "A9", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "SerialToggle", "A10", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "DirSelect_Abduction", "A11", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "DirSelect_Adduction", "A12", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "DirSelect_Flexion", "A13", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "DirSelect_Extension", "A14", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "GainSelect_Proportional", "A15", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "GainSelect_Integral", "A16", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "GainSelect_Derivative", "A17", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "GainsZero", "A18", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "TenSelect_A", "A19", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "TenSelect_B", "A20", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "TenSelect_C", "A21", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "Increment", "A22", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "Decrement", "A23", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "EncoderZero", "A24", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "EncoderMeasureLimit", "A25", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "EncoderSetLimit", "A26", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "FittsStart", "A27", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "FittsStop", "A28", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "TaskCalibrationStart", "A29", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "LimitsSelectA", "A30", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "LimitsSelectB", "A31", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "LimitsSelectC", "A32", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "LimitsStart", "A33", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "LimitsReset", "A34", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "Change reverse mode", "A35", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawKeyCell( "Rotate camera", "A36", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );

	DrawKeyCell( "Esc", "F1", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "1", "F2", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "2", "F3", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "3", "F4", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "4", "F5", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "5", "F6", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "`", "F7", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "a", "F8", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "t", "F9", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "s", "F10", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n4", "F11", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n6", "F12", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n2", "F13", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n8", "F14", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "p", "F15", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "i", "F16", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "d", "F17", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n5", "F18", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n=", "F19", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n/", "F20", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n*", "F21", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n+", "F22", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n-", "F23", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "z", "F24", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "x", "F25", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "c", "F26", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "f", "F27", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "g", "F28", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "F7", "F29", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n9", "F30", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n7", "F31", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n5", "F32", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "n0", "F33", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "nCR", "F34", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "r", "F35", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawKeyCell( "q", "F36", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );


	// Display window
	cv::imshow( winShortcuts, matShortcuts );
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
 *    CCCCCC  HH   HH  EEEEEEE   CCCCCC  KK    KK  LL      IIIIII   SSSSS  TTTTTTTT
 *  CC        HH   HH  EE      CC        KK   KK   LL        II    SS         TT
 *  CC        HH   HH  EE      CC        KK  KK    LL        II    SS         TT
 *  CC        HHHHHHH  EEEEE   CC        KKKKK     LL        II     SSSSS     TT
 *  CC        HH   HH  EE      CC        KK  KK    LL        II         SS    TT
 *  CC        HH   HH  EE      CC        KK   KK   LL        II         SS    TT
 *    CCCCCC  HH   HH  EEEEEEE   CCCCCC  KK    KK  LLLLLL  IIIIII   SSSSS     TT
 * 
 *  ========================================================================================= 
 *  ========================================================================================= 
 */


void DisplayClass::BuildChecklist() {


	// Create window
	cv::namedWindow( winChecklist, cv::WINDOW_AUTOSIZE );
	cv::moveWindow( winChecklist, 3440 - CONFIG_DIS_WIDTH - CONFIG_DIS_KEY_WIDTH - CONFIG_DIS_KEY_WIDTH - 6, 0 );

	// Checklist
	DrawChecklistCell( "Establish serial connection", "A1", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "Set tensioning values", "A2", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Tension motor A", "A3", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Tension motor B", "A4", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Tension motor C", "A5", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Stop tensioning", "A6", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "Set torque limits", "A7", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Set torque limit A", "A8", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Set torque limit B", "A9", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Set torque limit C", "A10", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Zero torque limits", "A11", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "Start Fitts task", "A12", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "Tune PID controller", "A13", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Adjust proportional gain", "A14", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Adjust integral gain", "A15", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Adjust derivative gain", "A16", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Select abduction direction", "A17", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Select adduction direction", "A18", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Select extension direction", "A19", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Select flexion direction", "A20", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );
	DrawChecklistCell( "   Zero all gains", "A21", 5, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, false );

	DrawChecklistCell( "s", "F1", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "t", "F2", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "=", "F3", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "/", "F4", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "*", "F5", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "t", "F6", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "0", "F7", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "7", "F8", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "9", "F9", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "5", "F10", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "f", "F11", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "CR", "F12", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "", "F13", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "p", "F14", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "i", "F15", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "d", "F16", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "4", "F17", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "6", "F18", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "8", "F19", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( "2", "F20", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );
	DrawChecklistCell( ".", "F21", 1, 1, key_fontBody, CONFIG_colWhite, CONFIG_colBlack, true );


	// Display window
	cv::imshow( winChecklist, matChecklist );
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
 *  =============================================================================================================================
 *  ============================================================================================================================ 
 *   
 *   DDDDDD    RRRRRR     AAA    WW      WW      FFFFFFF  UU   UU  NN    NN   CCCCCC  TTTTTTT  IIIIII   OOOOO   NN    NN   SSSSS
 *   DD    DD  RR   RR   AA AA   WW      WW      FF       UU   UU  NNN   NN  CC         TT       II    OO   OO  NNN   NN  SS
 *   DD    DD  RR   RR   AA AA   WW  WW  WW      FF       UU   UU  NN NN NN  CC         TT       II    OO   OO  NN NN NN  SS
 *   DD    DD  RRRRRR   AAAAAAA  WW  WW  WW      FFFFF    UU   UU  NN  NNNN  CC         TT       II    OO   OO  NN  NNNN   SSSSS
 *   DD    DD  RR  RR   AA   AA   WW WW WW       FF       UU   UU  NN   NNN  CC         TT       II    OO   OO  NN   NNN       SS
 *   DD    DD  RR   RR  AA   AA   WWW  WWW       FF       UU   UU  NN    NN  CC         TT       II    OO   OO  NN    NN       SS
 *   DDDDDD    RR   RR  AA   AA   WW    WW       FF        UUUUU   NN    NN   CCCCCC    TT     IIIIII   OOOOO   NN    NN   SSSSS
 *  
 *  ============================================================================================================================= 
 *  ============================================================================================================================= 
 */



/**
 * @brief Add text in a cell 
 * @param str Text to add [string]
 * @param cell0 Starting cell (e.g., "A1") [string]
 * @param width Number of columns across
 * @param height Number of columns down
 * @param sz Font size
 * @param textColor Color of body text
 * @param fillColor Color of background fill
 * @param centered Flag to center text
 */
void DisplayClass::DrawCell( std::string str, std::string cell0, short width, short height, float sz, cv::Scalar textColor, cv::Scalar fillColor, bool centered ) {

	// Check if using double letters (e.g., AA, AB)
	if ( std::isalpha( cell0[0] ) && !std::isalpha( cell0[1] ) ) {	  // Only one letter
		c0 = ( cell0[0] - 'A' ) * WIDTH;
		r0 = ( CONFIG_PANEL_HEIGHT + 1 ) + ( ( std::stoi( cell0.substr( 1 ) ) - 1 ) * HEIGHT - 1 );
		rH = height * HEIGHT;
		cW = std::floor( width * WIDTH );
	} else if ( std::isalpha( cell0[0] ) && std::isalpha( cell0[1] ) ) {	// Two letters)
		c0 = ( 26 + ( cell0[1] - 'A' ) ) * WIDTH;
		r0 = ( CONFIG_PANEL_HEIGHT + 1 ) + ( ( std::stoi( cell0.substr( 2 ) ) - 1 ) * HEIGHT - 1 );
		rH = height * HEIGHT;
		cW = std::floor( width * WIDTH );
	}


	// Draw cell frame
	cv::rectangle( shared->Display.matFrameOverlay, cv::Rect( c0, r0, cW, rH ), fillColor, -1 );
	cv::rectangle( shared->Display.matFrameOverlay, cv::Rect( c0, r0 - 1, cW + 1, rH + 1 ), CONFIG_colWhite, 1 );

	// Calculate text dimensions
	if ( sz == fontBody ) {
		textSize = cv::getTextSize( str, cv::FONT_HERSHEY_SIMPLEX, sz, 1, 0 );
	} else {
		textSize = cv::getTextSize( str, cv::FONT_HERSHEY_DUPLEX, sz, 1, 0 );
	}

	// Calculate position for center
	if ( centered ) {
		textX = c0 + ( cW - textSize.width ) / 2;
		textY = r0 + ( rH + textSize.height ) / 2 - 1;	  //+ ( cH - textSize.height ) / 2;
	} else {
		textX = c0 + 10;
		textY = r0 + ( rH + textSize.height ) / 2 - 1;	  //+ ( cH - textSize.height ) / 2;
	}

	// Place text
	if ( sz == fontBody ) {
		cv::putText( shared->Display.matFrameOverlay, str, cv::Point( textX, textY ), cv::FONT_HERSHEY_SIMPLEX, sz, textColor, 1 );
	} else {
		cv::putText( shared->Display.matFrameOverlay, str, cv::Point( textX, textY ), cv::FONT_HERSHEY_DUPLEX, sz, textColor, 1 );
	}
}



void DisplayClass::DrawCellBorder( std::string cell0, short width, short height, uint8_t thickness, cv::Scalar color ) {

	// Check if using double letters (e.g., AA, AB)
	if ( std::isalpha( cell0[0] ) && !std::isalpha( cell0[1] ) ) {	  // Only one letter
		c0 = ( cell0[0] - 'A' ) * WIDTH;
		r0 = ( CONFIG_PANEL_HEIGHT + 1 ) + ( ( std::stoi( cell0.substr( 1 ) ) - 1 ) * HEIGHT - 1 );
		rH = height * HEIGHT;
		cW = std::floor( width * WIDTH );
	} else if ( std::isalpha( cell0[0] ) && std::isalpha( cell0[1] ) ) {	// Two letters)
		c0 = ( 26 + ( cell0[1] - 'A' ) ) * WIDTH;
		r0 = ( CONFIG_PANEL_HEIGHT + 1 ) + ( ( std::stoi( cell0.substr( 2 ) ) - 1 ) * HEIGHT - 1 );
		rH = height * HEIGHT;
		cW = std::floor( width * WIDTH );
	}


	// Draw cell frame
	cv::rectangle( shared->Display.matFrameOverlay, cv::Rect( c0, r0 - 1, cW + 1, rH + 1 ), CONFIG_colWhite, thickness );
}



/**
 * @brief Add text in a cell 
 * @param str Text to add [string]
 * @param cell0 Starting cell (e.g., "A1") [string]
 * @param width Number of columns across
 * @param height Number of columns down
 * @param sz Font size
 * @param textColor Color of body text
 * @param fillColor Color of background fill
 * @param centered Flag to center text
 */
void DisplayClass::DrawKeyCell( std::string str, std::string cell0, short width, short height, float sz, cv::Scalar textColor, cv::Scalar fillColor, bool centered ) {

	// Check if using double letters (e.g., AA, AB)
	if ( std::isalpha( cell0[0] ) && !std::isalpha( cell0[1] ) ) {	  // Only one letter
		key_c0 = ( cell0[0] - 'A' ) * key_WIDTH;
		key_r0 = ( 1 ) + ( ( std::stoi( cell0.substr( 1 ) ) - 1 ) * key_HEIGHT - 1 );
		key_rH = height * key_HEIGHT;
		key_cW = width * key_WIDTH;
	} else if ( std::isalpha( cell0[0] ) && std::isalpha( cell0[1] ) ) {	// Two letters)
		key_c0 = ( 26 + ( cell0[1] - 'A' ) ) * key_WIDTH;
		key_r0 = ( 1 ) + ( ( std::stoi( cell0.substr( 2 ) ) - 1 ) * key_HEIGHT - 1 );
		key_rH = height * key_HEIGHT;
		key_cW = width * key_WIDTH;
	}


	// Draw cell frame
	cv::rectangle( matShortcuts, cv::Rect( key_c0, key_r0, key_cW, key_rH ), fillColor, -1 );
	cv::rectangle( matShortcuts, cv::Rect( key_c0, key_r0, key_cW + 1, key_rH + 1 ), CONFIG_colWhite, 1 );

	// Calculate text dimensions
	if ( sz >= key_fontHeader ) {
		key_textSize = cv::getTextSize( str, cv::FONT_HERSHEY_DUPLEX, sz, 1, 0 );
	} else {
		key_textSize = cv::getTextSize( str, cv::FONT_HERSHEY_SIMPLEX, sz, 1, 0 );
	}

	// Calculate position for center
	if ( centered ) {
		key_textX = key_c0 + ( key_cW - key_textSize.width ) / 2;
		key_textY = key_r0 + ( key_rH + key_textSize.height ) / 2 - 1;	  //+ ( cH - textSize.height ) / 2;
	} else {
		key_textX = key_c0 + 10;
		key_textY = key_r0 + ( key_rH + key_textSize.height ) / 2 - 1;	  //+ ( cH - textSize.height ) / 2;
	}

	// Place text
	if ( sz >= key_fontHeader ) {
		cv::putText( matShortcuts, str, cv::Point( key_textX, key_textY ), cv::FONT_HERSHEY_DUPLEX, sz, textColor, 1 );
	} else {
		cv::putText( matShortcuts, str, cv::Point( key_textX, key_textY ), cv::FONT_HERSHEY_SIMPLEX, sz, textColor, 1 );
	}

	// std::cout << key_c0 << "," << key_r0 << "," << key_rH << "," << key_cW << "\n";
}



/**
 * @brief Add text in a cell 
 * @param str Text to add [string]
 * @param cell0 Starting cell (e.g., "A1") [string]
 * @param width Number of columns across
 * @param height Number of columns down
 * @param sz Font size
 * @param textColor Color of body text
 * @param fillColor Color of background fill
 * @param centered Flag to center text
 */
void DisplayClass::DrawChecklistCell( std::string str, std::string cell0, short width, short height, float sz, cv::Scalar textColor, cv::Scalar fillColor, bool centered ) {

	// Check if using double letters (e.g., AA, AB)
	if ( std::isalpha( cell0[0] ) && !std::isalpha( cell0[1] ) ) {	  // Only one letter
		log_c0 = ( cell0[0] - 'A' ) * log_WIDTH;
		log_r0 = ( 1 ) + ( ( std::stoi( cell0.substr( 1 ) ) - 1 ) * log_HEIGHT - 1 );
		log_rH = height * log_HEIGHT;
		log_cW = width * log_WIDTH;
	} else if ( std::isalpha( cell0[0] ) && std::isalpha( cell0[1] ) ) {	// Two letters)
		log_c0 = ( 26 + ( cell0[1] - 'A' ) ) * log_WIDTH;
		log_r0 = ( 1 ) + ( ( std::stoi( cell0.substr( 2 ) ) - 1 ) * log_HEIGHT - 1 );
		log_rH = height * log_HEIGHT;
		log_cW = width * log_WIDTH;
	}


	// Draw cell frame
	cv::rectangle( matChecklist, cv::Rect( log_c0, log_r0, log_cW, log_rH ), fillColor, -1 );
	cv::rectangle( matChecklist, cv::Rect( log_c0, log_r0, log_cW + 1, log_rH + 1 ), CONFIG_colWhite, 1 );

	// Calculate text dimensions
	if ( sz >= log_fontHeader ) {
		log_textSize = cv::getTextSize( str, cv::FONT_HERSHEY_DUPLEX, sz, 1, 0 );
	} else {
		log_textSize = cv::getTextSize( str, cv::FONT_HERSHEY_SIMPLEX, sz, 1, 0 );
	}

	// Calculate position for center
	if ( centered ) {
		log_textX = log_c0 + ( log_cW - log_textSize.width ) / 2;
		log_textY = log_r0 + ( log_rH + log_textSize.height ) / 2 - 1;	  //+ ( cH - textSize.height ) / 2;
	} else {
		log_textX = log_c0 + 10;
		log_textY = log_r0 + ( log_rH + log_textSize.height ) / 2 - 1;	  //+ ( cH - textSize.height ) / 2;
	}

	// Place text
	if ( sz >= log_fontHeader ) {
		cv::putText( matChecklist, str, cv::Point( log_textX, log_textY ), cv::FONT_HERSHEY_DUPLEX, sz, textColor, 1 );
	} else {
		cv::putText( matChecklist, str, cv::Point( log_textX, log_textY ), cv::FONT_HERSHEY_SIMPLEX, sz, textColor, 1 );
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



// void DisplayClass::ShowVisualizer() {

// 	// Pre-calculate
// 	ProjectedCorners.clear();	 // <-- Important: clear it each frame!

// 	// Project corners
// 	for ( const auto& pt : cubeCorners ) {
// 		ProjectedCorners.push_back( ProjectIsometric( cv::Point3f( pt.z, pt.y, pt.x ) ) );	  // Swap x <-> z
// 	}
// 	// for ( const auto& pt : cubeCorners ) {
// 	// 	ProjectedCorners.push_back( ProjectIsometric( cv::Point3f( pt.z, pt.y, pt.x ) ) );	  // Swap x <-> z
// 	// }

// 	// Create window
// 	cv::namedWindow( winVisualizer, cv::WINDOW_AUTOSIZE );
// 	cv::moveWindow( winVisualizer, 3440 - CONFIG_DIS_WIDTH - CONFIG_DIS_VIZ_WIDTH - 6 - CONFIG_DIS_KEY_WIDTH, 0 );
// 	cv::imshow( winVisualizer, matVisualizer );
// }


// // Working visualizer
// void DisplayClass::UpdateVisualizer() {


// 	// Check if trail should be cleared
// 	if ( shared->vizClear ) {
// 		trailPoints.clear();
// 		shared->vizClear = false;
// 	}

// 	// Clear canvas
// 	matVisualizer = CONFIG_colWhite;


// 	// Draw cube
// 	for ( const auto& edge : edges ) {
// 		cv::line( matVisualizer, ProjectedCorners[edge.first], ProjectedCorners[edge.second], CONFIG_colGraLt, 1 );
// 	}

// 	uint8_t markerSize = 20;
// 	// Draw target square
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( 0, -markerSize, markerSize ) ), ProjectIsometric( cv::Point3i( 0, markerSize, markerSize ) ), CONFIG_colGraLt, 1 );
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( 0, -markerSize, -markerSize ) ), ProjectIsometric( cv::Point3i( 0, markerSize, -markerSize ) ), CONFIG_colGraLt, 1 );
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( 0, markerSize, -markerSize ) ), ProjectIsometric( cv::Point3i( 0, markerSize, markerSize ) ), CONFIG_colGraLt, 1 );
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( 0, -markerSize, -markerSize ) ), ProjectIsometric( cv::Point3i( 0, -markerSize, markerSize ) ), CONFIG_colGraLt, 1 );

// 	// X axis
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( 0, -vizLimXY, 0 ) ), ProjectIsometric( cv::Point3i( 0, vizLimXY, 0 ) ), CONFIG_colRedLt, 1 );
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( 0, vizLimXY, 0 ) ), ProjectIsometric( cv::Point3i( vizLimZ, vizLimXY, 0 ) ), CONFIG_colRedLt, 1 );
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( 0, -vizLimXY, 0 ) ), ProjectIsometric( cv::Point3i( 1000, -vizLimXY, 0 ) ), CONFIG_colRedLt, 1 );

// 	// Y axis
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( 0, 0, -vizLimXY ) ), ProjectIsometric( cv::Point3i( 0, 0, vizLimXY ) ), CONFIG_colGreLt, 1 );
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( 0, 0, vizLimXY ) ), ProjectIsometric( cv::Point3i( 1000, 0, vizLimXY ) ), CONFIG_colGreLt, 1 );

// 	// Calculate camera + marker positions
// 	cv::Point3i p3D		= cv::Point3i( shared->Target.positionFilteredNewMM.x, shared->Target.positionFilteredNewMM.y, shared->Target.positionFilteredNewMM.z );
// 	cv::Point3i p3DInv	= cv::Point3i( shared->Target.positionFilteredNewMM.z, -shared->Target.positionFilteredNewMM.y, shared->Target.positionFilteredNewMM.x );
// 	int			ptSizeX = int( float( ( 1000.0 - ( p3D.x + vizLimXY ) ) / 1000.0 ) * 10.0 );
// 	int			ptSizeY = int( float( ( 1000.0 - ( -1 * p3D.y + vizLimXY ) ) / 1000.0 ) * 10.0 );
// 	int			ptSizeZ = int( float( ( 1000.0 - p3D.z ) / 1000.0 ) * 10.0 );

// 	// Line from marker to target
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( 0, 0, 0 ) ), ProjectIsometric( p3DInv ), CONFIG_colCyaMd, 2 );

// 	// Moving local axis
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( p3DInv.x, p3DInv.y, vizLimXY ) ), ProjectIsometric( p3DInv ), CONFIG_colGreMd, 1 );
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( p3DInv.x, vizLimXY, p3DInv.z ) ), ProjectIsometric( p3DInv ), CONFIG_colRedMd, 1 );
// 	cv::line( matVisualizer, ProjectIsometric( cv::Point3i( 0, p3DInv.y, p3DInv.z ) ), ProjectIsometric( p3DInv ), CONFIG_colBluMd, 1 );



// 	// Create color gradient
// 	float zClamped = std::clamp( shared->Target.positionFilteredNewMM.z, 0.0f, 1000.0f );
// 	// float intensity = 128.0f * ( zClamped / 1000.0f );

// 	// Add current point to trail
// 	if ( trailCounter++ >= trailInterval ) {
// 		trailPoints.push_back( p3DInv );
// 		// trailColor.push_back( ( int )intensity );
// 		trailCounter = 0;

// 		// Limit trail length
// 		if ( trailPoints.size() > trailLimit ) {
// 			trailPoints.erase( trailPoints.begin() );
// 			// trailColor.erase( trailColor.begin() );
// 		}
// 	}

// 	// Draw trail points
// 	for ( int i = 0; i < trailPoints.size(); i++ ) {

// 		float ratio		= static_cast<float>( i ) / static_cast<float>( trailPoints.size() );
// 		int	  intensity = static_cast<int>( 191 * ( 1.0f - ratio ) );

// 		// Z trail
// 		cv::circle( matVisualizer, ProjectIsometric( cv::Point3i( 0, trailPoints.at( i ).y, trailPoints.at( i ).z ) ), 2, cv::Scalar( 255, 64 + intensity, 64 + intensity ), -1 );

// 		// Y trail
// 		cv::circle( matVisualizer, ProjectIsometric( cv::Point3i( trailPoints.at( i ).x, trailPoints.at( i ).y, vizLimXY ) ), 2, cv::Scalar( 64 + intensity, 255, 64 + intensity ), -1 );

// 		// X trail
// 		cv::circle( matVisualizer, ProjectIsometric( cv::Point3f( trailPoints.at( i ).x, vizLimXY, trailPoints.at( i ).z ) ), 2, cv::Scalar( 64 + intensity, 64 + intensity, 255 ), -1 );
// 	}

// 	// Shadow on wall
// 	cv::circle( matVisualizer, ProjectIsometric( cv::Point3i( p3DInv.x, p3DInv.y, vizLimXY ) ), ptSizeX, CONFIG_colGreLt, -1 );	   // Shadow on YZ axis (right)
// 	cv::circle( matVisualizer, ProjectIsometric( cv::Point3i( 0, p3DInv.y, p3DInv.z ) ), ptSizeZ, CONFIG_colBluWt, -1 );		   // Shadow on XY plane (bottom)
// 	cv::circle( matVisualizer, ProjectIsometric( cv::Point3i( p3DInv.x, vizLimXY, p3DInv.z ) ), ptSizeY, CONFIG_colRedLt, -1 );	   // Shadow on XZ (screen)

// 	// // Draw marker (last to place over other elements)
// 	cv::circle( matVisualizer, ProjectIsometric( p3DInv ), 10, CONFIG_colWhite, -1 );
// 	cv::circle( matVisualizer, ProjectIsometric( p3DInv ), 10, CONFIG_colBlack, 1 );


// 	// Show
// 	cv::imshow( "3D Visualizer", matVisualizer );
// }



// cv::Point2i DisplayClass::ProjectPoint( const cv::Point3f& p3d ) {
// 	if ( p3d.z == 0 )
// 		return cameraCenter;	// Avoid divide by zero
// 	int x2d = ( p3d.x * focalLength ) / ( float )p3d.z + cameraCenter.x;
// 	int y2d = ( p3d.y * focalLength ) / ( float )p3d.z + cameraCenter.y;
// 	return ( cv::Point2i( x2d, y2d ) );
// }


// // Apply isometric camera transform and perspective
// cv::Point2i DisplayClass::ProjectIsometric( const cv::Point3i& p3d ) {
// 	// Step 1: rotate the 3D point around X and Y axes


// 	// Rotation matrices
// 	float cx = cos( elevation ), sx = sin( elevation );
// 	float cy = cos( azimuth ), sy = sin( azimuth );

// 	// Rotate around Y axis (azimuth)
// 	float x1 = p3d.x * cy + p3d.z * sy;
// 	float y1 = p3d.y;
// 	float z1 = -p3d.x * sy + p3d.z * cy;

// 	// Then around X axis (elevation)
// 	float x2 = x1;
// 	float y2 = y1 * cx - z1 * sx;
// 	float z2 = y1 * sx + z1 * cx;

// 	// Step 2: perspective projection (same as before)
// 	float x_proj = ( x2 * focalLength ) / ( z2 + 2500.0f ) + ( ( CONFIG_DIS_VIZ_WIDTH / 2 ) - 300 );
// 	float y_proj = ( y2 * focalLength ) / ( z2 + 2500.0f ) + ( ( CONFIG_DIS_VIZ_HEIGHT / 2 ) - 150 );

// 	return cv::Point2f( x_proj, y_proj );
// }


// cv::Point2i DisplayClass::GetForwardDirectionFromPose( const cv::Vec3d rvec, const cv::Vec3d tvec, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, float axisLength ) {
// 	// Define 3D points: origin and Z-axis point
// 	std::vector<cv::Point3f> axisPoints;
// 	axisPoints.push_back( cv::Point3f( 0, 0, 0 ) );				// Origin
// 	axisPoints.push_back( cv::Point3f( 0, 0, axisLength ) );	// Forward along Z

// 	// Project to 2D
// 	std::vector<cv::Point2f> imagePoints;
// 	cv::projectPoints( axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints );

// 	// Direction vector from origin to Z point
// 	cv::Point2i dir = imagePoints[1] - imagePoints[0];
// 	return dir;
// }



// void DisplayClass::ShowAngle() {

// 	// Create window
// 	cv::namedWindow( winAngle, cv::WINDOW_AUTOSIZE );
// 	cv::moveWindow( winAngle, 3440 - CONFIG_DIS_WIDTH - CONFIG_DIS_VIZ_WIDTH - 6 - CONFIG_DIS_KEY_WIDTH, 0 );
// 	cv::imshow( winAngle, matAngles );
// }


// void DisplayClass::UpdateAngle() {

// 	matAngles = CONFIG_colWhite;

// 	// Centerpoint
// 	cv::Point2i midPt = cv::Point2i( CONFIG_DIS_ANGLE_WIDTH / 2, CONFIG_DIS_ANGLE_HEIGHT / 2 + 200 );

// 	// Draw middle line
// 	cv::line( matAngles, cv::Point2i( 0, midPt.y ), cv::Point2i( CONFIG_DIS_ANGLE_WIDTH, midPt.y ), CONFIG_colGraLt, 2 );
// 	// Draw circle
// 	cv::ellipse( matAngles, midPt, cv::Size( 400, 400 ), 0, 180, 360, CONFIG_colGraLt, 2 );

// 	// Radius
// 	int radius = 400;

// 	// Target line
// 	cv::Point2i endPtDesired( int( cos( ( shared->angleDesired + 90 ) * DEG2RAD ) * radius ), int( sin( ( shared->angleDesired + 270 ) * DEG2RAD ) * radius ) );
// 	cv::line( matAngles, midPt, midPt + endPtDesired, CONFIG_colGreWt, 10 );

// 	// Velocity Line
// 	// cv::Point2i endPtVel( int( cos( ( shared->angleFiltered + shared->angleVelocity + 90 ) * DEG2RAD ) * radius ), int( sin( ( shared->angleFiltered + shared->angleVelocity + 270 ) * DEG2RAD ) * radius ) );
// 	cv::Point2i endPtVel( int( shared->Telemetry.velocityFilteredNewMM.x * MM2PX ), int( shared->Telemetry.velocityFilteredNewMM.x * MM2PX ) );
// 	cv::line( matAngles, midPt, midPt + endPtVel, CONFIG_colBluWt, 2 );



// 	// Measured angle
// 	cv::Point2i endPt( int( cos( ( shared->angleFiltered + 90 ) * DEG2RAD ) * radius ), int( sin( ( shared->angleFiltered + 270 ) * DEG2RAD ) * radius ) );
// 	cv::line( matAngles, midPt, midPt + endPt, CONFIG_colBluLt, 3 );

// 	// Encoder center
// 	cv::circle( matAngles, midPt, 10, CONFIG_colGraDk, -1 );

// 	// Text
// 	cv::putText( matAngles, "Desired [deg]: ", cv::Point2i( 20, 40 ), cv::FONT_HERSHEY_DUPLEX, 1.0, CONFIG_colBlack, 1 );
// 	cv::putText( matAngles, shared->FormatDecimal( shared->angleDesired, 2, 2 ), cv::Point2i( 300, 40 ), cv::FONT_HERSHEY_DUPLEX, 1.0, CONFIG_colBlack, 1 );
// 	cv::putText( matAngles, "Measured [deg]: ", cv::Point2i( 20, 80 ), cv::FONT_HERSHEY_DUPLEX, 1.0, CONFIG_colBlack, 1 );
// 	cv::putText( matAngles, shared->FormatDecimal( shared->angleFiltered, 2, 2 ), cv::Point2i( 300, 80 ), cv::FONT_HERSHEY_DUPLEX, 1.0, CONFIG_colBlack, 1 );
// 	cv::putText( matAngles, "Error [deg]: ", cv::Point2i( 20, 120 ), cv::FONT_HERSHEY_DUPLEX, 1.0, CONFIG_colBlack, 1 );
// 	cv::putText( matAngles, shared->FormatDecimal( shared->angleFiltered - shared->angleDesired, 2, 2 ), cv::Point2i( 300, 120 ), cv::FONT_HERSHEY_DUPLEX, 1.0, CONFIG_colBlack, 1 );

// 	if ( shared->Touchscreen.isTouched == 1 ) {

// 		// Save image
// 		if ( shared->Logging.isEnabled ) {
// 			std::string imageFilename	 = "/home/tom/Code/nuring/logging/" + shared->Logging.filename + ".png";
// 			shared->Display.statusString = "Saving file " + imageFilename;
// 			cv::imwrite( imageFilename, matAngles );
// 			std::cout << "FittsClass:  Image saved at " << imageFilename << "\n";
// 		}
// 	}

// 	cv::imshow( winAngle, matAngles );
// }



// void DisplayClass::CheckOptions() {

// 	// Launch viz if requested
// 	if ( shared->vizEnabled ) {

// 		// Check if window loaded
// 		if ( shared->vizLoaded ) {
// 			// Update
// 			UpdateVisualizer();
// 		} else {
// 			ShowVisualizer();
// 			shared->vizLoaded = true;
// 		}
// 	} else {

// 		// Check if window loaded
// 		if ( shared->vizLoaded ) {
// 			shared->vizLoaded = false;
// 			cv::destroyWindow( winVisualizer );
// 		}
// 	}
// }
