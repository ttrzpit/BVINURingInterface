#pragma once

// OpenCV Header
// #include <opencv2/op
#include <opencv2/core.hpp>
#include <opencv2/core/utils/logger.hpp>	// For verbose debugging


#include <string>

// Configuration selection
#define HIGH_RESOLUTION 1

#if HIGH_RESOLUTION == 1

// Marker constants
inline constexpr short CONFIG_LARGE_MARKER_WIDTH = 18;	  // [mm] Marker dimension
// inline constexpr short CONFIG_LARGE_MARKER_WIDTH  = 30;	   // [mm] Marker dimension
inline constexpr short CONFIG_MEDIUM_MARKER_WIDTH = 80;
inline constexpr short CONFIG_SMALL_MARKER_WIDTH  = 8;
inline constexpr short CONFIG_RING_MARKER_WIDTH	  = 15;
inline constexpr short CONFIG_TARGET_OFFSET_Y_MM  = 35;

// Config name
inline const std::string CONFIG_TYPE = "HighResolution";

// Display properties
// Display lower panel
inline constexpr unsigned short CONFIG_PANEL_HEIGHT	   = 1100;
inline constexpr unsigned short CONFIG_DIS_WIDTH	   = 1600;
inline constexpr unsigned short CONFIG_DIS_HEIGHT	   = 1360;
inline constexpr unsigned short CONFIG_DIS_CELL_WIDTH  = 32;	// 32
inline constexpr unsigned short CONFIG_DIS_CELL_HEIGHT = 26;	// 26

// Display keyboard shortcut panel
inline constexpr unsigned short CONFIG_DIS_KEY_WIDTH	   = 301;
inline constexpr unsigned short CONFIG_DIS_KEY_HEIGHT	   = 1360;
inline constexpr unsigned short CONFIG_DIS_KEY_CELL_WIDTH  = 50;	// 20
inline constexpr unsigned short CONFIG_DIS_KEY_CELL_HEIGHT = 30;	// 20

// Display log shortcut panel
inline constexpr unsigned short CONFIG_DIS_LOG_WIDTH	   = 301;
inline constexpr unsigned short CONFIG_DIS_LOG_HEIGHT	   = 1360;
inline constexpr unsigned short CONFIG_DIS_LOG_CELL_WIDTH  = 50;	// 20
inline constexpr unsigned short CONFIG_DIS_LOG_CELL_HEIGHT = 30;	// 20

// 3D visualization
inline constexpr unsigned short CONFIG_DIS_VIZ_WIDTH  = 1540;	 // 20
inline constexpr unsigned short CONFIG_DIS_VIZ_HEIGHT = 1360;	 // 20

inline constexpr unsigned short CONFIG_DIS_ANGLE_WIDTH	= 1540;	   // 20
inline constexpr unsigned short CONFIG_DIS_ANGLE_HEIGHT = 1360;	   // 20
inline constexpr unsigned short CONFIG_FIELD_WIDTH_PX	= 802;	   // 20
inline constexpr unsigned short CONFIG_FIELD_LENGTH_PX	= 510;	   // 1110

// Touchscreen
inline constexpr unsigned short CONFIG_TOUCHSCREEN_WIDTH_PX	 = 1920;
inline constexpr unsigned short CONFIG_TOUCHSCREEN_HEIGHT_PX = 1080;
inline const cv::Point2i		CONFIG_TOUCHSCREEN_CENTER	 = cv::Point2i( CONFIG_TOUCHSCREEN_WIDTH_PX / 2, CONFIG_TOUCHSCREEN_HEIGHT_PX / 2 );

inline constexpr unsigned short CONFIG_TOUCHSCREEN_EXCLUSION_ZONE = 93;
inline constexpr unsigned short CONFIG_TOUCHSCREEN_MARKER_SIZE	  = CONFIG_LARGE_MARKER_WIDTH;


// Camera intrinsic parameters
inline constexpr unsigned short CONFIG_CAM_WIDTH	   = 1600;
inline constexpr unsigned short CONFIG_CAM_HEIGHT	   = 1200;
inline constexpr unsigned short CONFIG_CAM_FRAMERATE   = 60;
inline constexpr unsigned short CONFIG_CAM_PRINCIPAL_X = 801;
inline constexpr unsigned short CONFIG_CAM_PRINCIPAL_Y = 536;
inline const cv::Point2i		CONFIG_CAM_CENTER	   = cv::Point2i( CONFIG_CAM_PRINCIPAL_X, CONFIG_CAM_PRINCIPAL_Y );

// Detection parameters
inline constexpr unsigned short CONFIG_DET_RADIUS = 500;

// Declare matrices (defined in `config.cpp`)
inline const cv::Mat CONFIG_CAMERA_MATRIX	  = ( cv::Mat_<double>( 3, 3 ) << 600.98172, 0.00000, 801.26194, 0.00000, 599.86930, 535.69455, 0.00000, 0.00000, 1.00000 );
inline const cv::Mat CONFIG_DISTORTION_COEFFS = ( cv::Mat_<double>( 1, 5 ) << 0.01561, -0.03298, 0.00020, 0.00136, 0.00561 );


#elif HIGH_RESOLUTION == 0

inline constexpr short CONFIG_LARGE_MARKER_WIDTH = 20;	  // [mm] Marker dimension
// inline constexpr short CONFIG_LARGE_MARKER_WIDTH  = 30;	   // [mm] Marker dimension
inline constexpr short CONFIG_MEDIUM_MARKER_WIDTH = 20;
inline constexpr short CONFIG_SMALL_MARKER_WIDTH  = 8;
inline constexpr short CONFIG_RING_MARKER_WIDTH	  = 15;
inline constexpr short CONFIG_TARGET_OFFSET_Y_MM  = 35;

// Config name
inline const std::string CONFIG_TYPE = "LowResolution";

// Display properties
inline constexpr unsigned short CONFIG_PANEL_HEIGHT	   = 1100;
inline constexpr unsigned short CONFIG_DIS_WIDTH	   = 1600;
inline constexpr unsigned short CONFIG_DIS_HEIGHT	   = 1360;
inline constexpr unsigned short CONFIG_DIS_CELL_WIDTH  = 32;	// 32
inline constexpr unsigned short CONFIG_DIS_CELL_HEIGHT = 26;	// 26

// Display keyboard shortcut panel
inline constexpr unsigned short CONFIG_DIS_KEY_WIDTH	   = 301;
inline constexpr unsigned short CONFIG_DIS_KEY_HEIGHT	   = 1360;
inline constexpr unsigned short CONFIG_DIS_KEY_CELL_WIDTH  = 50;	// 20
inline constexpr unsigned short CONFIG_DIS_KEY_CELL_HEIGHT = 30;	// 20

// Display log shortcut panel
inline constexpr unsigned short CONFIG_DIS_LOG_WIDTH	   = 301;
inline constexpr unsigned short CONFIG_DIS_LOG_HEIGHT	   = 1360;
inline constexpr unsigned short CONFIG_DIS_LOG_CELL_WIDTH  = 50;	// 20
inline constexpr unsigned short CONFIG_DIS_LOG_CELL_HEIGHT = 30;	// 20

// 3D visualization
inline constexpr unsigned short CONFIG_DIS_VIZ_WIDTH  = 1540;	 // 20
inline constexpr unsigned short CONFIG_DIS_VIZ_HEIGHT = 1360;	 // 20

inline constexpr unsigned short CONFIG_DIS_ANGLE_WIDTH	= 1540;	   // 20
inline constexpr unsigned short CONFIG_DIS_ANGLE_HEIGHT = 1360;	   // 20
inline constexpr unsigned short CONFIG_FIELD_WIDTH_PX	= 802;	   // 20
inline constexpr unsigned short CONFIG_FIELD_LENGTH_PX	= 510;	   // 1110

inline constexpr unsigned short CONFIG_TOUCHSCREEN_WIDTH_PX	 = 1920;
inline constexpr unsigned short CONFIG_TOUCHSCREEN_HEIGHT_PX = 1080;
inline const cv::Point2i		CONFIG_TOUCHSCREEN_CENTER	 = cv::Point2i( CONFIG_TOUCHSCREEN_WIDTH_PX / 2, CONFIG_TOUCHSCREEN_HEIGHT_PX / 2 );

inline constexpr unsigned short CONFIG_TOUCHSCREEN_EXCLUSION_ZONE = 93;
inline constexpr unsigned short CONFIG_TOUCHSCREEN_MARKER_SIZE	  = CONFIG_LARGE_MARKER_WIDTH;



// Camera intrinsic parameters
inline constexpr unsigned short CONFIG_CAM_WIDTH	   = 1280;
inline constexpr unsigned short CONFIG_CAM_HEIGHT	   = 960;
inline constexpr unsigned short CONFIG_CAM_FRAMERATE   = 90;
inline constexpr unsigned short CONFIG_CAM_PRINCIPAL_X = 637;
inline constexpr unsigned short CONFIG_CAM_PRINCIPAL_Y = 429;
inline const cv::Point2i		CONFIG_CAM_CENTER	   = cv::Point2i( CONFIG_CAM_PRINCIPAL_X, CONFIG_CAM_PRINCIPAL_Y );


// Declare matrices (defined in `config.cpp`)
inline const cv::Mat CONFIG_CAMERA_MATRIX	  = ( cv::Mat_<double>( 3, 3 ) << 473.78479, 0.00000, 637.39461, 0.00000, 472.53074, 428.80961, 0.00000, 0.00000, 1.00000 );
inline const cv::Mat CONFIG_DISTORTION_COEFFS = ( cv::Mat_<double>( 1, 5 ) << 0.01343, -0.03333, -0.00007, 0.00033, 0.00605 );


// Detection parameters
inline constexpr unsigned short CONFIG_DET_RADIUS = 400;

#endif


// Device parameters
inline constexpr float CONFIG_DEVICE_TORQUE_CONSTANT = 14.4f;	 // Motor torque constant [nM*m/A]
inline constexpr float CONFIG_DEVICE_NOMINAL_CURRENT = 1.89f;	 // Motor nominal current [A]
inline constexpr float CONFIG_DEVICE_NOMINAL_TORQUE	 = 28.6;	 // Nominal torque [mN*m]

// Serial properties
inline std::string CONFIG_SERIAL_PORT_0 = "/dev/ttyACM0";
// inline std::string CONFIG_SERIAL_PORT_0 = "/dev/pts/10";
inline std::string CONFIG_SERIAL_PORT_1 = "/dev/ttyACM1";



// Unit conversions per touchscreen
#define MM2PX 3.66142
#define PX2MM 0.27312
#define SIN35 0.57358
#define SIN145 0.57358
#define SIN270 -1.00000
#define COS35 0.81915
#define COS145 -0.81915
#define COS270 0.00000


// Camera propertiesy
inline constexpr short			CONFIG_CAM_BRIGHTNESS		 = 0;		 // Default 0
inline constexpr unsigned short CONFIG_CAM_CONTRAST			 = 0;		 // Default 0
inline constexpr unsigned short CONFIG_CAM_SATURATION		 = 32;		 // Default 56
inline constexpr short			CONFIG_CAM_HUE				 = 0;		 // Default 0
inline constexpr bool			CONFIG_CAM_AUTO_WHITEBALANCE = false;	 // Default 1
inline constexpr unsigned short CONFIG_CAM_GAMMA			 = 120;		 // Default 110
inline constexpr unsigned short CONFIG_CAM_GAIN				 = 18;		 // Default 0
inline constexpr unsigned short CONFIG_CAM_POWERLINE_FREQ	 = 2;		 // Default 0
inline constexpr unsigned short CONFIG_CAM_WHITEBALANCE_TEMP = 4600;	 // Default 4600
inline constexpr unsigned short CONFIG_CAM_SHARPNESS		 = 0;		 // Default 0
inline constexpr unsigned short CONFIG_CAM_BACKLIGHT		 = 0;		 // Default 54
inline constexpr unsigned short CONFIG_CAM_AUTO_EXPOSURE	 = 1;		 // Default 3   1 = manual, 2 = shutter priority, 3 = aperature priority
inline constexpr unsigned short CONFIG_CAM_EXPOSURE_LEVEL	 = 32;		 // Default 32
inline constexpr int			CONFIG_CAM_PAN				 = 0;		 // Default 0
inline constexpr int			CONFIG_CAM_TILT				 = 0;		 // Default 0
inline constexpr unsigned short CONFIG_CAM_FOCUS_LEVEL		 = 0;		 // Default 0
inline constexpr bool			CONFIG_CAM_AUTO_FOCUS		 = 0;		 // Default 1
inline constexpr unsigned short CONFIG_CAM_ZOOM				 = 0;		 // Default 0

// Declare colors (defined in `config.cpp`)
extern const cv::Scalar CONFIG_colRedMd, CONFIG_colRedLt, CONFIG_colRedDk, CONFIG_colRedBk, CONFIG_colRedWt;
extern const cv::Scalar CONFIG_colOraMd, CONFIG_colOraLt, CONFIG_colOraDk, CONFIG_colOraBk, CONFIG_colOraWt;
extern const cv::Scalar CONFIG_colYelMd, CONFIG_colYelLt, CONFIG_colYelDk, CONFIG_colYelBk, CONFIG_colYelWt;
extern const cv::Scalar CONFIG_colGreMd, CONFIG_colGreLt, CONFIG_colGreDk, CONFIG_colGreBk, CONFIG_colGreWt;
extern const cv::Scalar CONFIG_colBluMd, CONFIG_colBluLt, CONFIG_colBluDk, CONFIG_colBluBk, CONFIG_colBluWt;
extern const cv::Scalar CONFIG_colVioMd, CONFIG_colVioLt, CONFIG_colVioDk, CONFIG_colVioBk, CONFIG_colVioWt;
extern const cv::Scalar CONFIG_colGraMd, CONFIG_colGraLt, CONFIG_colGraDk, CONFIG_colGraBk;
extern const cv::Scalar CONFIG_colBlack, CONFIG_colWhite;
extern const cv::Scalar CONFIG_colCyaMd, CONFIG_colCyaLt, CONFIG_colCyaDk;
extern const cv::Scalar CONFIG_colMagMd, CONFIG_colMagLt, CONFIG_colMagDk;


// Pulley
// inline constexpr unsigned short CONFIG_PULLEY_RADIUS = 2.5;	   // [mm]

// // Available Resolutions @ 90Hz
// // Resolution   1920x1080   1920x1200   1600x1200   1280x960    1280x720    1024x768    960x720     800x600
// // MPixels      2.07        2.30        1.92        1.23        0.92        0.79        0.69        0.48
// // Aspect Ratio 1.778       1.600       1.33        1.33        1.778       1.333       1.333       1.333