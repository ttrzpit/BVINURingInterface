#pragma once

#include <cmath>
#include <memory>
#include <opencv2/core.hpp>


class SystemDataManager;
struct ManagedData;

// Constants
#define RAD2DEG 57.2958
#define DEG2RAD 0.01745

class ControllerClass {
public:
	ControllerClass( SystemDataManager& ctx );

	void Update();
	void Update4D();
	void MapToContributionABC( cv::Point3f terms );
	void UpdateAmplifier();
	void UpdateVibrotactile();

private:
	SystemDataManager&			 dataHandle;
	std::shared_ptr<ManagedData> shared;

	// float		radius		   = 0.005f;							 // Pulley radius in [m]
	cv::Point3f contribution = cv::Point3f( 0.0f, 0.0f, 0.0f );	   // Motor contribution
	float		rTarget		 = 0.0f;							   // Distance to Target on XY plane
	float		rMax		 = 0.0f;							   // Maximum target distance (based on max angle 40-deg)
	float		rMaxXY		 = 0.0f;							   // Maximum target distance (based on max angle 40-deg)
	float		rMaxZ		 = 0.0f;							   // Maximum target distance (based on max angle 40-deg)
	float		thTarget	 = 0.0f;							   // Angle towards Target on XY plane
	float		thAT		 = 0.0f;							   // Angle between Target and motor A
	float		thBT		 = 0.0f;							   // Angle between Target and motor B
	float		thCT		 = 0.0f;							   // Angle between Target and motor C
	const float thA			 = 35.0f * DEG2RAD;					   // Angle of motor A
	const float thB			 = 145.0f * DEG2RAD;				   // Angle of motor B
	const float thC			 = 270.0f * DEG2RAD;				   // Angle of motor C
	const float thAB		 = 110.0f * DEG2RAD;				   // Arc of motors AB
	const float thBC		 = 125.0f * DEG2RAD;				   // Arc of motors BC
	const float thAC		 = 125.0f * DEG2RAD;				   // Arc of motors AC

	// PID
	cv::Point3f currentError  = cv::Point3f( 0.0f, 0.0f, 0.0f );
	cv::Point3f previousError = cv::Point3f( 0.0f, 0.0f, 0.0f );
	cv::Point3f deltaError	  = cv::Point3f( 0.0f, 0.0f, 0.0f );
	float		integralDecay = 0.9f;	 // 0.9 = slow, 0.0 = instant

	// Functions
	cv::Point3f MapToCurrent( cv::Point3f percentage, float iNominal );
	cv::Point3i MapToPWM( cv::Point3f percentage, int min, int max );
	cv::Point3f MapToContributionTerm( cv::Point3f terms );
	void		RampUp();
};
