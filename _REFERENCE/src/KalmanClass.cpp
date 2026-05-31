

// Call to class header
#include "KalmanClass.h"

// System data manager
#include "SystemDataManager.h"

// Math
#include <cmath>

/**
 * @brief Construct a new Kalman Class:: Kalman Class object
 *
 * @param ctx
 */
KalmanClass::KalmanClass( SystemDataManager& ctx )
	: dataHandle( ctx )
	, shared( ctx.getData() )
	, isInitialized( false ) {

	// Build matrices
	state				= cv::Mat::zeros( 6, 1, CV_32F );
	P					= cv::Mat::eye( 6, 6, CV_32F ) * 1.0f;
	Q					= cv::Mat::eye( 6, 6, CV_32F ) * kalmanProcessNoiseCovarianceQ;
	Q.at<float>( 3, 3 ) = 0.5f;	   // Velocity
	Q.at<float>( 4, 4 ) = 0.5f;
	Q.at<float>( 5, 5 ) = 0.5f;
	R					= cv::Mat::eye( 3, 3, CV_32F ) * kalmanMeasurementNoiseR;
	H					= cv::Mat::zeros( 3, 6, CV_32F );
	I					= cv::Mat::eye( 6, 6, CV_32F );

	// Only position is measured
	H.at<float>( 0, 0 ) = 1.0f;
	H.at<float>( 1, 1 ) = 1.0f;
	H.at<float>( 2, 2 ) = 1.0f;
}



/**
 * @brief Initialize kalman filter
 *
 * @param initialPos        Initial position of system
 * @param initialTimestamp  Initial timestamp
 */
void KalmanClass::Initialize( const cv::Point3f& initialPos, float tInitial ) {

	// std::cout << "KalmanClass: Initializing Kalman filter.\n";

	// Populate state
	state.at<float>( 0 ) = initialPos.x;	// Position
	state.at<float>( 1 ) = initialPos.y;
	state.at<float>( 2 ) = initialPos.z;
	state.at<float>( 3 ) = 0.0f;	// Velocity
	state.at<float>( 4 ) = 0.0f;
	state.at<float>( 5 ) = 0.0f;

	// Reset covariance (optional, if you want a clean slate)
	P = cv::Mat::eye( 6, 6, CV_32F ) * 1.0f;

	// Update values
	tPrevious	  = tInitial;
	integralError = cv::Point3f( 0.0f, 0.0f, 0.0f );
	prevError	  = cv::Point3f( 0.0f, 0.0f, 0.0f );


	// Update flag
	isInitialized				 = true;
	shared->Target.isTargetReset = false;
}



/**
 * @brief Update kalman filter
 *
 * @param measuredPos       Measured position
 * @param currentTimestamp  Updated timestamp
 */
void KalmanClass::Update( const cv::Point3f& measuredPos, float tCurrent ) {


	if ( shared->Target.isTargetReset ) {

		// shared->timingTimestamp = 0.0f;
		tCurrent					 = 0.0f;
		tPrevious					 = 0.0f;
		shared->Target.isTargetReset = false;
		Initialize( shared->Target.positionUnfilteredMM, shared->Timing.elapsedRunningTime );
		// std::cout << "KalmanClass: Reset\n";
		return;
	}



	// Make sure the filter is initialized
	if ( !isInitialized ) {
		Initialize( measuredPos, tCurrent );
		return;
	}

	// Normal Kalman update
	dt		  = std::clamp( tCurrent - tPrevious, 1e-5f, kalmanTimeStepDt );
	tPrevious = tCurrent;


	F					= cv::Mat::eye( 6, 6, CV_32F );
	F.at<float>( 0, 3 ) = dt;
	F.at<float>( 1, 4 ) = dt;
	F.at<float>( 2, 5 ) = dt;

	state = F * state;
	P	  = F * P * F.t() + Q;

	z = ( cv::Mat_<float>( 3, 1 ) << measuredPos.x, measuredPos.y, measuredPos.z );
	y = z - H * state;
	S = H * P * H.t() + R;

	if ( cv::determinant( S ) > 1e-6f ) {
		K	  = P * H.t() * S.inv();
		state = state + K * y;
		P	  = ( I - K * H ) * P;
	} else {
		std::cerr << "KalmanClass: S matrix not invertible, skipping update.\n";
	}

	// Calculate integral term
	cv::Point3f positionEstimate = GetPosition();
	cv::Point3f error			 = positionEstimate - ( measuredPos * -1 );
	float		errorMagnitude	 = cv::norm( cv::Point2f( error.x, error.y ) );


	// Only calculate if within a certain radius of the target
	if ( errorMagnitude < shared->Controller.integrationRadius ) {


		integralError += error * dt;
	} else {
		integralError *= 0.95f;
	}


	// Clamp to avoid windup
	integralError.x = std::clamp( integralError.x, -maxError, maxError );
	integralError.y = std::clamp( integralError.y, -maxError, maxError );
	integralError.z = std::clamp( integralError.z, -maxError, maxError );

	// Save for derivative use
	prevError = error;
}


// /**
//  * @brief Get covariance matrix
//  *
//  * @return cv::Mat&
//  */
// const cv::Mat& KalmanClass::GetCovariance() const {
// 	return P;
// }



/**
 * @brief Get filtered position
 *
 * @return cv::Point3f 3D position
 */
cv::Point3f KalmanClass::GetPosition() const {

	return cv::Point3f( state.at<float>( 0 ), state.at<float>( 1 ), state.at<float>( 2 ) );
}



/**
 * @brief Get filtered velocity
 *
 * @return cv::Point3f
 */
cv::Point3f KalmanClass::GetVelocity() const {

	return cv::Point3f( state.at<float>( 3 ), state.at<float>( 4 ), state.at<float>( 5 ) );
}



/**
 * @brief Get angle of error
 *
 * @return cv::Point2f
 */
cv::Point2f KalmanClass::GetAngle() const {

	// Get position
	cv::Point3f pos = GetPosition();

	// Calculate angle
	return cv::Point2f( std::clamp( atan2( pos.x, pos.z ), -M_PI / 4.0, M_PI / 4.0 ), std::clamp( atan2( pos.y, pos.z ), -M_PI / 4.0, M_PI / 4.0 ) );
}


/**
 * @brief Get filtered angular velocity
 *
 * @return cv::Point2f
 */
cv::Point2f KalmanClass::GetAnglularVelocity() const {

	// Get position and velocity
	cv::Point3f pos = GetPosition();
	cv::Point3f vel = GetVelocity();

	// Calculate angular velocity
	float dz	  = pos.z != 0.0f ? pos.z : 1e-6f;
	float dThetaX = ( vel.x * dz - pos.x * vel.z ) / ( dz * dz + pos.x * pos.x );
	float dThetaY = ( vel.y * dz - pos.y * vel.z ) / ( dz * dz + pos.y * pos.y );

	// Return filtered angular velocity
	return cv::Point2f( dThetaX, dThetaY );
}



cv::Point3f KalmanClass::GetIntegralError() const {

	return integralError;
}