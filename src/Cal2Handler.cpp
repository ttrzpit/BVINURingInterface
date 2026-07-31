#include "Cal2Handler.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

// =============================================================================
// Cal2Handler.cpp
//
// See Cal2Handler.h for the ramp/fit overview.
// =============================================================================

Cal2Handler::Cal2Handler( const ControllerHandler& controller, const AromBoundary& aromBoundary,
                          const Cal2Config& cfg )
    : controller_( controller ), aromBoundary_( aromBoundary ), cfg_( cfg ) {
    for ( int i = 0; i < CONSTANT_CALIBRATION_ANGLES_COUNT; i++ ) {
        results_[i].theta = CONSTANT_CALIBRATION_ANGLES_DEG[i] * DEG_TO_RAD;
    }
}

void Cal2Handler::Reset() {
    if ( !aromBoundary_.valid ) {
        phase_ = Phase::BLOCKED;
        status_ = "Stiffness: run ARoM calibration first.";
        return;
    }

    results_ = {};
    for ( int i = 0; i < CONSTANT_CALIBRATION_ANGLES_COUNT; i++ ) {
        results_[i].theta = CONSTANT_CALIBRATION_ANGLES_DEG[i] * DEG_TO_RAD;
    }

    commandedForce_ = {};
    headingIdx_ = -1;    // StartHeading(0, ...) on first Update()
    phase_ = Phase::RAMP_UP;
    status_ = "Stiffness: starting...";
}

void Cal2Handler::Update( double nowSecs ) {
    if ( phase_ == Phase::DONE || phase_ == Phase::BLOCKED ) return;

    if ( headingIdx_ < 0 ) {
        StartHeading( 0, nowSecs );
    }

    float theta = results_[headingIdx_].theta;
    cv::Point2f dir( std::cos( theta ), std::sin( theta ) );
    float elapsed = static_cast<float>( nowSecs - phaseStartSecs_ );

    cv::Point2f pos_virtual = controller_.GetVirtualPosition();
    float posProj = pos_virtual.x * dir.x + pos_virtual.y * dir.y;

    cv::Point2f measuredForce = controller_.GetMeasuredForce();
    float forceProj = measuredForce.x * dir.x + measuredForce.y * dir.y;

    float boundaryRadius = aromBoundary_.RadiusAtAngle( theta );
    float forceMax = controller_.GetDeflectionForceMax();

    float commandedMag = 0.0f;

    switch ( phase_ ) {
        case Phase::RAMP_UP: {
            commandedMag = std::min( cfg_.forceRampRate * elapsed, forceMax );
            rampSamples_.emplace_back( posProj, forceProj );

            if ( posProj >= boundaryRadius || commandedMag >= forceMax ) {
                peakForceMag_ = commandedMag;
                phase_ = Phase::HOLD;
                phaseStartSecs_ = nowSecs;
            }
            break;
        }
        case Phase::HOLD: {
            commandedMag = peakForceMag_;
            if ( elapsed >= cfg_.holdSecs ) {
                FitStiffness( headingIdx_ );

                const Cal2HeadingResult& r = results_[headingIdx_];
                std::cout << std::fixed << std::setprecision( 3 )
                          << "Cal2: " << ( headingIdx_ + 1 ) << "/" << CONSTANT_CALIBRATION_ANGLES_COUNT
                          << " theta=" << ( r.theta * RAD_TO_DEG ) << " deg"
                          << "  K(theta)=" << r.stiffness_kP << " N/mm"
                          << "  samples=" << r.rampUpSamples
                          << "  Boundary Radius=" << boundaryRadius << " mm"
                          << ( r.valid ? "" : "  [INVALID]" ) << "\n"
                          << std::defaultfloat;

                commandedMag = 0.0f;
                phase_ = Phase::WAIT;
                phaseStartSecs_ = nowSecs;
            }
            break;
        }
        case Phase::WAIT: {
            commandedMag = 0.0f;
            if ( elapsed >= cfg_.releaseWaitSecs ) {
                if ( headingIdx_ + 1 < CONSTANT_CALIBRATION_ANGLES_COUNT ) {
                    StartHeading( headingIdx_ + 1, nowSecs );
                } else {
                    phase_ = Phase::DONE;
                    status_ = "Stiffness calibration complete.";

                    std::cout << "Cal2: === STIFFNESS CALIBRATION COMPLETE ===\n"
                              << std::fixed << std::setprecision( 3 );
                    for ( int i = 0; i < CONSTANT_CALIBRATION_ANGLES_COUNT; i++ ) {
                        const Cal2HeadingResult& res = results_[i];
                        // Per-heading boundary radius (the old printout reused
                        // the LAST heading's radius on every row).
                        std::cout << "Cal2:   theta=" << ( res.theta * RAD_TO_DEG ) << " deg"
                                  << "  K(theta)=" << res.stiffness_kP << " N/mm"
                                  << "  Boundary Radius=" << aromBoundary_.RadiusAtAngle( res.theta ) << " mm"
                                  << ( res.valid ? "" : "  [INVALID]" ) << "\n";
                    }
                    std::cout << std::defaultfloat;
                }
            }
            break;
        }
        default:
            break;
    }

    commandedForce_ = dir * commandedMag;

    if ( phase_ != Phase::DONE ) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision( 1 );
        ss << "Stiffness " << ( headingIdx_ + 1 ) << "/" << CONSTANT_CALIBRATION_ANGLES_COUNT
           << " (" << ( theta * RAD_TO_DEG ) << " deg): ";
        switch ( phase_ ) {
            case Phase::RAMP_UP:
                ss << "ramping up -- " << commandedMag << " N";
                break;
            case Phase::HOLD:
                ss << "holding -- " << commandedMag << " N";
                break;
            case Phase::WAIT:
                ss << "released -- waiting";
                break;
            default:
                break;
        }
        status_ = ss.str();
    }
}

void Cal2Handler::StartHeading( int index, double nowSecs ) {
    headingIdx_ = index;
    phase_ = Phase::RAMP_UP;
    phaseStartSecs_ = nowSecs;
    peakForceMag_ = 0.0f;
    rampSamples_.clear();
}

void Cal2Handler::FitStiffness( int index ) {
    Cal2HeadingResult& r = results_[index];
    r.rampUpSamples = static_cast<int>( rampSamples_.size() );

    if ( rampSamples_.size() < 2 ) {
        r.stiffness_kP = 0.0f;
        r.valid = false;
        return;
    }

    // Least-squares fit forceProj = K * posProj + c, solve for slope K.
    double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
    for ( const cv::Point2f& s : rampSamples_ ) {
        sumX += s.x;
        sumY += s.y;
        sumXY += static_cast<double>( s.x ) * s.y;
        sumXX += static_cast<double>( s.x ) * s.x;
    }
    double n = static_cast<double>( rampSamples_.size() );
    double denom = n * sumXX - sumX * sumX;

    if ( std::abs( denom ) < 1e-9 ) {
        r.stiffness_kP = 0.0f;
        r.valid = false;
        return;
    }

    r.stiffness_kP = static_cast<float>( ( n * sumXY - sumX * sumY ) / denom );
    r.valid = true;
}

std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT> Cal2Handler::GetStiffnessProfile() const {
    std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT> profile = {};
    for ( int i = 0; i < CONSTANT_CALIBRATION_ANGLES_COUNT; i++ ) {
        profile[i] = results_[i].stiffness_kP;
    }
    return profile;
}

void Cal2Handler::LoadStiffnessProfile( const std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT>& kTheta ) {
    for ( int i = 0; i < CONSTANT_CALIBRATION_ANGLES_COUNT; i++ ) {
        results_[i].stiffness_kP = kTheta[i];
        results_[i].valid        = true;
    }
    phase_  = Phase::DONE;
    status_ = "Stiffness: loaded from participant config.";
}
