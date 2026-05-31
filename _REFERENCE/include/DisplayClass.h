#pragma once

// OpenCV core functions
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Memory for shared data
#include <memory>

// Configuration
#include <config.h>

// For std::clamp
#include <cmath>

// Forward declarations
class SystemDataManager;
struct ManagedData;



/** 
 * @brief Display class definition
 */
class DisplayClass {

public:
	// Data manager handle
	DisplayClass( SystemDataManager& dataHandle );

	// Public functions
	void Update();
	void AddStaticDisplayPanels();
	// void ShowVisualizer();
	// void UpdateVisualizer();
	// void ClearVisualizer();
	// void CheckOptions();


	// Public variables
	std::vector<cv::Point3i> trailPoints;
	std::vector<int>		 trailColor;
	int						 trailCounter  = 0;
	int						 trailInterval = 1;
	int						 trailLimit	   = 1000;

private:
	// Data manager handle
	SystemDataManager&			 dataHandle;
	std::shared_ptr<ManagedData> shared;



	// Window names
	std::string winAngle	  = "Angle Visualizer";
	std::string winInterface  = "NURing Interface";
	std::string winShortcuts  = "Keyboard Shortcuts";
	std::string winVisualizer = "3D Visualizer";
	std::string winChecklist  = "Log";

	// Private variables
	float	 fontTitle	= 0.0f;
	float	 fontHeader = 0.0f;
	float	 fontBody	= 0.0f;
	cv::Size textSize;
	// short	 cellPx = 40;
	short WIDTH	 = CONFIG_DIS_CELL_WIDTH;
	short HEIGHT = CONFIG_DIS_CELL_HEIGHT;
	short c0	 = 0;
	short r0	 = 0;
	short nR	 = 0;
	short nC	 = 0;
	short cW	 = 0;
	short rH	 = 0;
	short textX	 = 0;
	short textY	 = 0;

	// Keyboard shortcut variables
	float	 key_fontHeader = 0.0f;
	float	 key_fontBody	= 0.0f;
	cv::Size key_textSize;
	short	 key_WIDTH	= CONFIG_DIS_KEY_CELL_WIDTH;
	short	 key_HEIGHT = CONFIG_DIS_KEY_CELL_HEIGHT;
	short	 key_c0		= 0;
	short	 key_r0		= 0;
	short	 key_nR		= 0;
	short	 key_nC		= 0;
	short	 key_cW		= 0;
	short	 key_rH		= 0;
	short	 key_textX	= 0;
	short	 key_textY	= 0;

	// Log variables
	float	 log_fontHeader = 0.0f;
	float	 log_fontBody	= 0.0f;
	cv::Size log_textSize;
	short	 log_WIDTH	= CONFIG_DIS_LOG_CELL_WIDTH;
	short	 log_HEIGHT = CONFIG_DIS_LOG_CELL_HEIGHT;
	short	 log_c0		= 0;
	short	 log_r0		= 0;
	short	 log_nR		= 0;
	short	 log_nC		= 0;
	short	 log_cW		= 0;
	short	 log_rH		= 0;
	short	 log_textX	= 0;
	short	 log_textY	= 0;


	// Visualizer settings
	std::vector<cv::Point2i> ProjectedCorners;
	cv::Mat					 matShortcuts = cv::Mat( CONFIG_DIS_HEIGHT, CONFIG_DIS_KEY_WIDTH, CV_8UC3 );
	cv::Mat					 matChecklist = cv::Mat( CONFIG_DIS_HEIGHT, CONFIG_DIS_KEY_WIDTH, CV_8UC3 );

	// Variables for motor viz
	float limA = 0.0f;
	float limB = 0.0f;
	float limC = 0.0f;


	// Private functions
	// Interface
	void ShowInterface();
	void BuildReadoutInterface();
	void BuildLogInterface();
	void BuildKeyboardShortcuts();
	void BuildChecklist();

	// Drawing helper functions
	void DrawCell( std::string str, std::string cell0, short width, short height, float sz, cv::Scalar textColor, cv::Scalar fillColor, bool centered );
	void DrawCellBorder( std::string cell0, short width, short height, uint8_t thickness, cv::Scalar color );
	void DrawKeyCell( std::string str, std::string cell0, short width, short height, float sz, cv::Scalar textColor, cv::Scalar fillColor, bool centered );
	void DrawChecklistCell( std::string str, std::string cell0, short width, short height, float sz, cv::Scalar textColor, cv::Scalar fillColor, bool centered );

	// Add element functions
	void AddCameraElements();
	void AddPidElements();
	void AddGainElements();
	void AddTextTelemetry();
	void AddTextController();
	void AddTextSystem();
	void AddTextAmplifier();
	void AddTextSerial();
	void AddTextMotorOutput();
	void AddTextTask();


	// Outdated visualization
	cv::Point2i ProjectIsometric( const cv::Point3i& p3d );
	cv::Point2i GetForwardDirectionFromPose( const cv::Vec3d rvec, const cv::Vec3d tvec, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, float axisLength );
	static void onMouse( int event, int x, int y, int flags, void* userData );
};
