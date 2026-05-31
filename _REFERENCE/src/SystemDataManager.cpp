// System data manager
#include "SystemDataManager.h"



SystemDataManager::SystemDataManager()
	: data( std::make_shared<ManagedData>() ) { }



/***
 * @brief Function to return pointer to data
 */
std::shared_ptr<ManagedData> SystemDataManager::getData() {
	return data;
}



/**
 * @brief Calculate the norm of a 2D vector
 * 
 * @param pt1 2D vector
 * @return [float] Norm of the 2D vector
 */
float ManagedData::GetNorm2D( cv::Point2f pt1 ) {
	return cv::norm( pt1 );
}



/**
 * @brief Calculate the norm of a 3D vector
 * 
 * @param pt1 3D vector
 * @return [float] Norm of the 3D vector
 */
float ManagedData::GetNorm3D( cv::Point3f pt1 ) {
	return cv::norm( pt1 );
}



/**
 * @brief Calculate the normalized distance between 2 points in 2D
 * 
 * @param pt1 First 2D point
 * @param pt2 Second 2D point
 * @return [float] Norm of the 2D vector
 */
float ManagedData::GetDist2D( cv::Point2f pt1, cv::Point2f pt2 ) {
	return cv::norm( pt1 - pt2 );
}



/**
 * @brief Calculate the normalized distance between 2 points in 3D
 * 
 * @param pt1 First 3D point
 * @param pt2 Second 3D point
 * @return [float] Norm of the 3D vector
 */
float ManagedData::GetDist3D( cv::Point3f pt1, cv::Point3f pt2 ) {
	return cv::norm( pt1 - pt2 );
}



/**
 * @brief Calculate the x,y components between two points
 * 
 * @param pt1 
 * @param pt2 
 * @return cv::Point2f 
 */
cv::Point2f ManagedData::GetDelta2D( cv::Point2f pt1, cv::Point2f pt2 ) {
	return ( pt1 - pt2 );
}



/**
 * @brief Calculate the x,y components between two points
 * 
 * @param pt1 
 * @param pt2 
 * @return cv::Point2f 
 */
cv::Point3f ManagedData::GetDelta3D( cv::Point3f pt1, cv::Point3f pt2 ) {
	return ( pt1 - pt2 );
}



/**
 * @brief Formats a float and returns it as xx.yyy in string format
 * 
 * @param x Input float
 * @return std::string Formatted text string
 */
std::string ManagedData::FormatDecimal( float x, uint8_t p , uint8_t d ) {

	// Build new string
	std::ostringstream oss;

	// Format string
	oss << std::setfill( '0' ) << std::setw( p ) << std::fixed << std::setprecision( d ) << x;

	return oss.str();
}

/**
 * @brief Pad the given value with a specified number of zeros
 * @param val Value to be padded
 * @param nZeroes Number of padding zeroes
 */
std::string ManagedData::PadValues( int val, int nZeroes ) {
	std::ostringstream sstream;
	sstream << std::setw( nZeroes ) << std::setfill( '0' ) << val;
	return sstream.str();
}


/**
 * @brief Remaps a float value
 * 
 * @param x 		Input value
 * @param inMin 	Input minimum
 * @param inMax		Input maximum
 * @param outMin 	Output minimum
 * @param outMax 	Output maximum 
 * @return float 	Re-mapped value
 */
float ManagedData::MapFloat( float val, float inMin, float inMax, float outMin, float outMax ) {

	float newVal = ( val - inMin ) * ( ( outMax - outMin ) / ( inMax - inMin ) ) + outMin;
	return std::clamp( newVal, outMin, outMax );
}
