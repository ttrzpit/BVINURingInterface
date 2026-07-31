#pragma once

// =============================================================================
// ParticipantConfigHandler.h - Per-participant calibration persistence
//
// Reads/writes logging/<UUU>/config<UUU>.yaml (OpenCV FileStorage dialect),
// storing the three calibration results keyed by participant ID:
//   * Cal1 - AROM boundary (theta/radius/accel control points)
//   * Cal2 - stiffness K(theta) profile
//   * Cal3 - camera->fingertip offset + roll reference
//
// Each section is INDEPENDENT (partial-aware): a section is written the moment
// its calibration completes, and Load() applies whichever sections are present.
// This keeps a session's finished calibrations even if it is interrupted before
// all three are done. The file is separate from the main config.yaml.
// =============================================================================

#include <array>
#include <string>

#include <opencv2/core.hpp>

#include "Cal1Handler.h"   // AromBoundary, kAromBoundaryPoints
#include "Globals.h"       // CONSTANT_CALIBRATION_ANGLES_COUNT


class ParticipantConfigHandler {
public:
    /** @brief Absolute path <baseDir>/<UUU>/config<UUU>.yaml (UUU = zero-padded
     *         3-digit participant ID, matching the TrialLogger folder naming). */
    static std::string PathFor( const std::string& baseDir, int userId );

    /**
     * @brief Load any present sections for userId into this handler.
     * @return true if the file exists AND contained at least one valid
     *         calibration section (query which via HasArom/HasStiffness/HasOffset).
     */
    bool Load( int userId, const std::string& baseDir );

    /**
     * @brief Write config<UUU>.yaml, creating the participant folder if needed.
     *        Only non-null sections are written, so partial calibration state is
     *        preserved - pass whichever results are currently available.
     */
    bool Save( int userId, const std::string& baseDir,
               const AromBoundary*                                             arom,
               const std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT>*     stiffness,
               const cv::Point3f*                                             offset,
               const float*                                                   rollRef ) const;

    // ---- Loaded-section queries (valid after Load) --------------------------
    bool HasArom()      const { return hasArom_; }
    bool HasStiffness() const { return hasStiffness_; }
    bool HasOffset()    const { return hasOffset_; }
    bool HasAny()       const { return hasArom_ || hasStiffness_ || hasOffset_; }

    const AromBoundary& GetArom() const { return arom_; }
    const std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT>& GetStiffness() const { return stiffness_; }
    cv::Point3f GetOffset()  const { return offset_; }
    float       GetRollRef() const { return rollRef_; }

private:
    bool hasArom_      = false;
    bool hasStiffness_ = false;
    bool hasOffset_    = false;

    AromBoundary                                        arom_;
    std::array<float, CONSTANT_CALIBRATION_ANGLES_COUNT> stiffness_ = {};
    cv::Point3f                                         offset_  = {};
    float                                               rollRef_ = 0.0f;
};
