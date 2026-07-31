#include "ParticipantConfigHandler.h"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

// =============================================================================
// ParticipantConfigHandler.cpp - see header. Uses cv::FileStorage (same YAML
// dialect as Config.cpp / RigAlignmentHandler), so no extra dependencies.
// =============================================================================

std::string ParticipantConfigHandler::PathFor( const std::string& baseDir, int userId ) {
    std::ostringstream folder;
    folder << std::setfill( '0' ) << std::setw( 3 ) << std::max( 0, userId );
    const std::string uuu = folder.str();
    std::filesystem::path p = std::filesystem::path( baseDir ) / uuu / ( "config" + uuu + ".yaml" );
    return p.string();
}

bool ParticipantConfigHandler::Load( int userId, const std::string& baseDir ) {
    hasArom_ = hasStiffness_ = hasOffset_ = false;

    const std::string path = PathFor( baseDir, userId );
    std::error_code   ec;
    if ( !std::filesystem::exists( path, ec ) ) return false;

    cv::FileStorage fs( path, cv::FileStorage::READ );
    if ( !fs.isOpened() ) {
        std::cerr << "ParticipantConfig: could not open " << path << " for reading.\n";
        return false;
    }

    // ---- Cal1 AROM boundary -------------------------------------------------
    cv::FileNode a = fs["arom"];
    if ( !a.empty() && (int)a["valid"] != 0 ) {
        std::vector<float> theta, radius, accel;
        a["theta"] >> theta;
        a["radius"] >> radius;
        a["accel"] >> accel;
        if ( (int)theta.size() == kAromBoundaryPoints &&
             (int)radius.size() == kAromBoundaryPoints &&
             (int)accel.size() == kAromBoundaryPoints ) {
            for ( int i = 0; i < kAromBoundaryPoints; ++i ) {
                arom_.theta[i]  = theta[i];
                arom_.radius[i] = radius[i];
                arom_.accel[i]  = accel[i];
            }
            arom_.valid = true;
            hasArom_    = true;
        }
    }

    // ---- Cal2 stiffness profile ---------------------------------------------
    cv::FileNode s = fs["stiffness"];
    if ( !s.empty() && (int)s["valid"] != 0 ) {
        std::vector<float> k;
        s["k_theta"] >> k;
        if ( (int)k.size() == CONSTANT_CALIBRATION_ANGLES_COUNT ) {
            for ( int i = 0; i < CONSTANT_CALIBRATION_ANGLES_COUNT; ++i ) stiffness_[i] = k[i];
            hasStiffness_ = true;
        }
    }

    // ---- Cal3 camera->fingertip offset --------------------------------------
    cv::FileNode o = fs["fingertip_offset"];
    if ( !o.empty() && (int)o["valid"] != 0 ) {
        offset_  = cv::Point3f( static_cast<float>( (double)o["x"] ),
                                static_cast<float>( (double)o["y"] ),
                                static_cast<float>( (double)o["z"] ) );
        rollRef_ = static_cast<float>( (double)o["roll_ref_rad"] );
        hasOffset_ = true;
    }

    fs.release();
    return hasArom_ || hasStiffness_ || hasOffset_;
}

bool ParticipantConfigHandler::Save( int userId, const std::string& baseDir,
                                     const AromBoundary*                                         arom,
                                     const std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT>* stiffness,
                                     const cv::Point3f*                                          offset,
                                     const float*                                                rollRef ) const {
    const std::string path = PathFor( baseDir, userId );
    std::error_code   ec;
    std::filesystem::create_directories( std::filesystem::path( path ).parent_path(), ec );

    cv::FileStorage fs( path, cv::FileStorage::WRITE );
    if ( !fs.isOpened() ) {
        std::cerr << "ParticipantConfig: could not open " << path << " for writing.\n";
        return false;
    }

    fs << "user_id" << std::max( 0, userId );
    fs << "note" << "Per-participant calibration (Cal1 AROM / Cal2 stiffness / Cal3 offset).";

    if ( arom && arom->valid ) {
        std::vector<float> theta( arom->theta.begin(), arom->theta.end() );
        std::vector<float> radius( arom->radius.begin(), arom->radius.end() );
        std::vector<float> accel( arom->accel.begin(), arom->accel.end() );
        fs << "arom" << "{"
           << "valid" << 1
           << "theta" << theta
           << "radius" << radius
           << "accel" << accel
           << "}";
    }

    if ( stiffness ) {
        std::vector<float> k( stiffness->begin(), stiffness->end() );
        fs << "stiffness" << "{"
           << "valid" << 1
           << "k_theta" << k
           << "}";
    }

    if ( offset && rollRef ) {
        fs << "fingertip_offset" << "{"
           << "valid" << 1
           << "x" << offset->x
           << "y" << offset->y
           << "z" << offset->z
           << "roll_ref_rad" << *rollRef
           << "}";
    }

    fs.release();
    std::cout << "ParticipantConfig: saved " << path << "\n";
    return true;
}
