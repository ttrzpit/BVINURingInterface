#include "config.h"

// // Camera matrices definition
// #if HIGH_RESOLUTION
// // const cv::Mat CONFIG_CAMERA_MATRIX	   = ( cv::Mat_<double>( 3, 3 ) << 600.98172, 0.00000, 801.26194, 0.00000, 599.86930, 535.69455, 0.00000, 0.00000, 1.00000 );
// // const cv::Mat CONFIG_DISTORTION_COEFFS = ( cv::Mat_<double>( 1, 5 ) << 0.01561, -0.03298, 0.00020, 0.00136, 0.00561 );

// #else
// const cv::Mat CONFIG_CAMERA_MATRIX = ( cv::Mat_<double>( 3, 3 ) << 473.78479, 0.00000, 637.39461, 0.00000, 472.53074, 428.80961, 0.00000, 0.00000, 1.00000 );
// const cv::Mat CONFIG_DISTORTION_COEFFS = ( cv::Mat_<double>( 1, 5 ) << 0.01343, -0.03333, -0.00007, 0.00033, 0.00605 );
// #endif

// Color definitions BGR
const cv::Scalar CONFIG_colBlack = cv::Scalar( 0, 0, 0 );
const cv::Scalar CONFIG_colWhite = cv::Scalar( 255, 255, 255 );

const cv::Scalar CONFIG_colRedWt = cv::Scalar( 192, 192, 255 );
const cv::Scalar CONFIG_colRedLt = cv::Scalar( 127, 127, 255 );
const cv::Scalar CONFIG_colRedMd = cv::Scalar( 0, 0, 255 );
const cv::Scalar CONFIG_colRedDk = cv::Scalar( 0, 0, 127 );
const cv::Scalar CONFIG_colRedBk = cv::Scalar( 0, 0, 64 );
const cv::Scalar CONFIG_colOraWt = cv::Scalar( 192, 224, 255 );
const cv::Scalar CONFIG_colOraLt = cv::Scalar( 127, 191, 255 );
const cv::Scalar CONFIG_colOraMd = cv::Scalar( 0, 127, 255 );
const cv::Scalar CONFIG_colOraDk = cv::Scalar( 0, 64, 127 );
const cv::Scalar CONFIG_colOraBk = cv::Scalar( 0, 32, 64 );
const cv::Scalar CONFIG_colYelWt = cv::Scalar( 192, 255, 255 );
const cv::Scalar CONFIG_colYelLt = cv::Scalar( 127, 255, 255 );
const cv::Scalar CONFIG_colYelMd = cv::Scalar( 0, 255, 255 );
const cv::Scalar CONFIG_colYelDk = cv::Scalar( 0, 127, 127 );
const cv::Scalar CONFIG_colYelBk = cv::Scalar( 0, 64, 64 );
const cv::Scalar CONFIG_colGreWt = cv::Scalar( 192, 255, 192 );
const cv::Scalar CONFIG_colGreLt = cv::Scalar( 127, 255, 127 );
const cv::Scalar CONFIG_colGreMd = cv::Scalar( 0, 255, 0 );
const cv::Scalar CONFIG_colGreDk = cv::Scalar( 0, 127, 0 );
const cv::Scalar CONFIG_colGreBk = cv::Scalar( 0, 64, 0 );
const cv::Scalar CONFIG_colBluWt = cv::Scalar( 255, 192, 192 );
const cv::Scalar CONFIG_colBluLt = cv::Scalar( 255, 127, 127 );
const cv::Scalar CONFIG_colBluMd = cv::Scalar( 255, 0, 0 );
const cv::Scalar CONFIG_colBluDk = cv::Scalar( 127, 0, 0 );
const cv::Scalar CONFIG_colBluBk = cv::Scalar( 64, 0, 0 );
const cv::Scalar CONFIG_colVioWt = cv::Scalar( 255, 192, 255 );
const cv::Scalar CONFIG_colVioLt = cv::Scalar( 255, 127, 255 );
const cv::Scalar CONFIG_colVioMd = cv::Scalar( 255, 0, 255 );
const cv::Scalar CONFIG_colVioDk = cv::Scalar( 127, 0, 127 );
const cv::Scalar CONFIG_colVioBk = cv::Scalar( 64, 0, 64 );

const cv::Scalar CONFIG_colGraWt = cv::Scalar( 224, 224, 224 );
const cv::Scalar CONFIG_colGraLt = cv::Scalar( 192, 192, 192 );
const cv::Scalar CONFIG_colGraMd = cv::Scalar( 127, 127, 127 );
const cv::Scalar CONFIG_colGraDk = cv::Scalar( 64, 64, 64 );
const cv::Scalar CONFIG_colGraBk = cv::Scalar( 32, 32, 32 );


const cv::Scalar CONFIG_colCyaLt = cv::Scalar( 255, 255, 127 );
const cv::Scalar CONFIG_colCyaMd = cv::Scalar( 255, 255, 0 );
const cv::Scalar CONFIG_colCyaDk = cv::Scalar( 127, 127, 0 );

const cv::Scalar CONFIG_colMagLt = cv::Scalar( 255, 127, 255 );
const cv::Scalar CONFIG_colMagMd = cv::Scalar( 255, 0, 255 );
const cv::Scalar CONFIG_colMagDk = cv::Scalar( 127, 0, 127 );
