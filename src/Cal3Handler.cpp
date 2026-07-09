#include "Cal3Handler.h"

#include <cmath>
#include <iomanip>
#include <iostream>

// =============================================================================
// Cal3Handler.cpp - Camera-to-Fingertip Offset Calibration
//
// Multi-tag solvePnP builds the pose from ALL visible ArUco markers at once.
// Each marker's 4 corners provide 4 point correspondences, so even 2 visible
// markers give 8 correspondences - more than enough for a robust solution.
//
// World coordinate frame:
//   Origin  = top-left corner of the touchscreen
//   X right, Y down (matching screen image coordinates), Z toward camera
//   Units: millimetres
//
// solvePnP returns the screen→camera transform (R, t):
//   p_cam = R * p_world + t
// Camera position in world: p_cam_world = -R^T * t
// =============================================================================

Cal3Handler::Cal3Handler( const TouchscreenConfig& touchCfg,
                          const CameraConfig& camCfg,
                          const ArucoCalibrationGridConfig& calGridCfg,
                          const Cal3Config& cal3Cfg )
    : touchCfg_( touchCfg ), camCfg_( camCfg ), calGridCfg_( calGridCfg ), cal3Cfg_( cal3Cfg ), holdSecs_( cal3Cfg.holdSecs ), cooldownSecs_( cal3Cfg.cooldownSecs ), maxSamples_( cal3Cfg.maxSamples ) {}

void Cal3Handler::Reset() {
    phase_ = Phase::WAITING;
    touchStartSecs_ = 0.0;
    cooldownEndSecs_ = 0.0;
    sampleCount_ = 0;
    offsets_.clear();
    rollSamples_.clear();
    lastOffset_ = {};
    finalOffset_ = {};
    rollReference_ = 0.0f;
    status_ = "Touch screen (0/" + std::to_string( maxSamples_ ) + ")";
    std::cout << "Cal3: Reset - ready to record " << maxSamples_ << " touches.\n";
}

// =============================================================================
// Update - called once per main loop iteration
// =============================================================================

bool Cal3Handler::Update( const TouchState& touch,
                          const std::vector<DetectedMarker>& markers,
                          double nowSecs ) {
    if ( phase_ == Phase::DONE ) return false;

    switch ( phase_ ) {
        case Phase::WAITING:
            if ( touch.isTouched ) {
                phase_ = Phase::HOLDING;
                touchStartSecs_ = nowSecs;
                status_ = "Hold steady...";
            }
            break;

        case Phase::HOLDING:
            if ( !touch.isTouched ) {
                // Lifted before hold duration - reset without penalising
                phase_ = Phase::WAITING;
                status_ = "Released too early - try again (" + std::to_string( sampleCount_ ) + "/" + std::to_string( maxSamples_ ) + ")";
            } else if ( nowSecs - touchStartSecs_ >= holdSecs_ ) {
                // Stable touch - attempt to record
                cv::Vec3d rvec;
                cv::Point3f camPos;

                if ( !ComputeCameraPoseInScreen( markers, rvec, camPos ) ) {
                    // solvePnP failed - not enough markers visible; ask to retry
                    phase_ = Phase::WAITING;
                    status_ = "Not enough markers visible - reposition and try again.";
                    std::cerr << "Cal3: solvePnP failed - need >= 2 visible markers.\n";
                    break;
                }

                // Offset = fingertip (touch) − camera, in screen-world mm
                float touch_x_mm = touch.position.x * touchCfg_.mmPerPixel;
                float touch_y_mm = touch.position.y * touchCfg_.mmPerPixel;

                cv::Point3f offset( touch_x_mm - camPos.x,
                                    touch_y_mm - camPos.y,
                                    -camPos.z );    // Z: camera is above screen (camPos.z > 0)

                offsets_.push_back( offset );
                lastOffset_ = offset;

                // Roll reference = circular mean of the per-marker corner roll
                // (ArucoHandler sets marker.rollRad from the marker's top edge).
                // This MUST use the same measure as the runtime roll so the delta
                // is zero at the reference pose; rvec[2] is avoided because the
                // planar-pose ambiguity makes it jump. All calibration-grid markers
                // are axis-aligned, so each measures the camera roll; averaging
                // (via sin/cos to handle wraparound) reduces detector noise.
                float sumSin = 0.0f, sumCos = 0.0f;
                for ( const auto& mk : markers ) {
                    sumSin += std::sin( mk.rollRad );
                    sumCos += std::cos( mk.rollRad );
                }
                float roll = std::atan2( sumSin, sumCos );
                rollSamples_.push_back( roll );

                sampleCount_++;

                std::cout << std::fixed << std::setprecision( 1 )
                          << "Cal3: Sample " << sampleCount_ << "/" << maxSamples_
                          << " - d = (" << offset.x << ", " << offset.y
                          << ", " << offset.z << ") mm"
                          << "  touch=(" << touch.position.x << ", " << touch.position.y
                          << " px)"
                          << "  cam_screen=(" << camPos.x << ", " << camPos.y << " mm)\n"
                          << std::defaultfloat;

                if ( sampleCount_ >= maxSamples_ ) {
                    // Average all offsets
                    cv::Point3f sum{ 0, 0, 0 };
                    for ( const auto& o : offsets_ ) sum += o;
                    finalOffset_ = sum * ( 1.0f / maxSamples_ );

                    // Circular mean of the per-sample rolls (each is itself a
                    // circular mean over the visible markers). An arithmetic
                    // mean would break if the samples straddled the +-pi wrap.
                    float refSin = 0.0f, refCos = 0.0f;
                    for ( float r : rollSamples_ ) {
                        refSin += std::sin( r );
                        refCos += std::cos( r );
                    }
                    rollReference_ = std::atan2( refSin, refCos );

                    phase_ = Phase::DONE;
                    status_ = "Complete!";

                    std::cout << std::fixed << std::setprecision( 2 )
                              << "Cal3: === CALIBRATION COMPLETE ===\n"
                              << "Cal3: offset_cam_to_fingertip (d) = ("
                              << finalOffset_.x << ", "
                              << finalOffset_.y << ", "
                              << finalOffset_.z << ") mm\n"
                              << "Cal3: roll_reference = " << ( rollReference_ * RAD_TO_DEG ) << " deg\n"
                              << std::defaultfloat;
                } else {
                    cooldownEndSecs_ = nowSecs + cooldownSecs_;
                    phase_ = Phase::COOLDOWN;
                    status_ = "Sample " + std::to_string( sampleCount_ ) + "/" + std::to_string( maxSamples_ ) + " - wait 2 s...";
                }
                return true;    // New sample recorded this call
            }
            break;

        case Phase::COOLDOWN:
            if ( nowSecs >= cooldownEndSecs_ ) {
                phase_ = Phase::WAITING;
                status_ = "Touch screen (" + std::to_string( sampleCount_ ) + "/" + std::to_string( maxSamples_ ) + ")";
            }
            break;

        default:
            break;
    }
    return false;
}

// =============================================================================
// Private - multi-tag solvePnP
// =============================================================================

bool Cal3Handler::ComputeCameraPoseInScreen( const std::vector<DetectedMarker>& markers,
                                             cv::Vec3d& rvecOut,
                                             cv::Point3f& camPosOut ) const {
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;

    // Mirror the layout from ArucoHandler::renderCalibrationGridImage exactly:
    // exclusion zone boundary → fixed gap between markers → 0-based IDs.
    const float sz_px = std::round( calGridCfg_.markerSizeMm * touchCfg_.pixelsPerMm );
    const float gap_px = std::round( calGridCfg_.markerPadMm * touchCfg_.pixelsPerMm );
    const float exc_px = std::round( calGridCfg_.markerExclusionMm * touchCfg_.pixelsPerMm );
    const float sz_mm = calGridCfg_.markerSizeMm;

    const int availW = touchCfg_.width - 2 * static_cast<int>( exc_px );
    const int availH = touchCfg_.height - 2 * static_cast<int>( exc_px );
    const int cols = std::max( 1, static_cast<int>( ( availW + gap_px ) / ( sz_px + gap_px ) ) );
    const int rows = std::max( 1, static_cast<int>( ( availH + gap_px ) / ( sz_px + gap_px ) ) );

    for ( const auto& m : markers ) {
        // Calibration grid uses 0-based IDs
        if ( m.id < 0 || m.id >= cols * rows ) continue;

        const int col = m.id % cols;
        const int row = m.id / cols;

        // Top-left of this marker in screen pixels, then convert to mm
        const float ox_mm = ( exc_px + col * ( sz_px + gap_px ) ) * touchCfg_.mmPerPixel;
        const float oy_mm = ( exc_px + row * ( sz_px + gap_px ) ) * touchCfg_.mmPerPixel;

        // Four corners in world mm (Z = 0, flat screen plane)
        // Order matches OpenCV ArUco: top-left, top-right, bottom-right, bottom-left
        objectPoints.push_back( { ox_mm, oy_mm, 0.0f } );
        objectPoints.push_back( { ox_mm + sz_mm, oy_mm, 0.0f } );
        objectPoints.push_back( { ox_mm + sz_mm, oy_mm + sz_mm, 0.0f } );
        objectPoints.push_back( { ox_mm, oy_mm + sz_mm, 0.0f } );

        for ( int k = 0; k < 4; k++ ) {
            imagePoints.push_back( { m.cornersPx[k].x, m.cornersPx[k].y } );
        }
    }

    // Require at least 2 markers (8 correspondences) for a reliable pose
    if ( static_cast<int>( objectPoints.size() ) < 8 ) return false;

    cv::Vec3d rvec, tvec;
    bool ok = cv::solvePnP( objectPoints, imagePoints,
                            camCfg_.cameraMatrix, camCfg_.distCoeffs,
                            rvec, tvec, false, cv::SOLVEPNP_ITERATIVE );
    if ( !ok ) return false;

    // Invert the screen→camera transform to get camera position in screen world
    // p_world = R^T * (p_cam − t)  →  camera is at p_cam=0, so:
    // cam_in_world = R^T * (-t)
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    cv::Mat t_mat = cv::Mat( tvec );
    cv::Mat camWorld = -R.t() * t_mat;    // 3×1, double

    camPosOut = cv::Point3f( static_cast<float>( camWorld.at<double>( 0 ) ),
                             static_cast<float>( camWorld.at<double>( 1 ) ),
                             static_cast<float>( camWorld.at<double>( 2 ) ) );
    rvecOut = rvec;
    return true;
}
