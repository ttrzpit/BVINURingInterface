#include "RigAlignmentHandler.h"

#include <algorithm>
#include <iostream>
#include <sstream>

// =============================================================================
// RigAlignmentHandler.cpp - see the header for the model.
// =============================================================================

// ---- File-local helpers -----------------------------------------------------

static cv::Matx33d toMatx33(const cv::Mat& m) {
    cv::Mat md;
    m.convertTo(md, CV_64F);
    cv::Matx33d r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r(i, j) = md.at<double>(i, j);
    return r;
}

static cv::Point2f cornerCentroid(const std::vector<cv::Point2f>& c) {
    return (c[0] + c[1] + c[2] + c[3]) * 0.25f;
}

// ---- Construction -----------------------------------------------------------

RigAlignmentHandler::RigAlignmentHandler(const ArucoCalibrationGridConfig& calGridCfg,
                                         const TouchscreenConfig&          touchCfg,
                                         const ObjectWorldConfig&          objCfg,
                                         const CameraConfig&               camCfg)
    : calGridCfg_(calGridCfg), touchCfg_(touchCfg), objCfg_(objCfg), camCfg_(camCfg),
      calDictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000)),
      objDictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100)) {
    // Subpixel corner refinement: this is a one-time, low-rate capture, so the
    // extra cost is irrelevant and it tightens both pose solves.
    cv::aruco::DetectorParameters params;
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    calDetector_ = cv::aruco::ArucoDetector(calDictionary_, params);
    objDetector_ = cv::aruco::ArucoDetector(objDictionary_, params);
}

void RigAlignmentHandler::Reset() {
    sumR_             = cv::Matx33d::zeros();
    count_            = 0;
    complete_         = false;
    finalR_           = cv::Matx33d::eye();
    lastScreenOk_     = false;
    lastWorldOk_      = false;
    lastScreenMarkers_ = 0;
    lastWorldMarkers_  = 0;
    std::cout << "RigAlign: reset - show the touchscreen calibration grid AND the "
                 "world board to the camera together.\n";
}

// =============================================================================
// Update - one raw grayscale frame; accumulate one sample when both poses solve
// =============================================================================

void RigAlignmentHandler::Update(const cv::Mat& grayFrame) {
    if (complete_ || grayFrame.empty()) return;

    // Detect both dictionaries on the same frame.
    std::vector<int>                      calIds, objIds;
    std::vector<std::vector<cv::Point2f>> calCorners, objCorners;
    calDetector_.detectMarkers(grayFrame, calCorners, calIds);
    objDetector_.detectMarkers(grayFrame, objCorners, objIds);

    cv::Matx33d Rscreen, Rworld;
    lastScreenOk_ = SolveScreenPose(calIds, calCorners, Rscreen);
    lastWorldOk_  = SolveWorldPose(objIds, objCorners, Rworld);

    if (!lastScreenOk_ || !lastWorldOk_) return;

    // R_screen->world = R_world->cam^T * R_screen->cam. Accumulate for averaging.
    const cv::Matx33d Rs2w = Rworld.t() * Rscreen;
    sumR_ += Rs2w;
    ++count_;

    if (count_ >= kTargetSamples) {
        // Average the rotation matrices, then re-orthonormalise onto SO(3) via
        // SVD (R = U * Vt, with a determinant-sign fix so it stays a proper
        // rotation, not a reflection).
        const cv::Matx33d avg = sumR_ * (1.0 / static_cast<double>(count_));
        cv::Mat w, u, vt;
        cv::SVD::compute(cv::Mat(avg), w, u, vt);
        cv::Mat Rmat = u * vt;
        if (cv::determinant(Rmat) < 0) {
            u.col(2) *= -1.0;
            Rmat = u * vt;
        }
        finalR_   = toMatx33(Rmat);
        complete_ = true;
        std::cout << "RigAlign: === COMPLETE (" << count_ << " samples) ===\n"
                  << "RigAlign: R_screen->world =\n"
                  << cv::Mat(finalR_) << "\n";
    }
}

// =============================================================================
// Screen->camera pose from the calibration-grid markers
// (mirrors Cal3Handler::ComputeCameraPoseInScreen - keep both in sync)
// =============================================================================

bool RigAlignmentHandler::SolveScreenPose(const std::vector<int>&                      ids,
                                          const std::vector<std::vector<cv::Point2f>>& corners,
                                          cv::Matx33d&                                 Rout) const {
    const float sz_px  = std::round(calGridCfg_.markerSizeMm     * touchCfg_.pixelsPerMm);
    const float gap_px = std::round(calGridCfg_.markerPadMm      * touchCfg_.pixelsPerMm);
    const float exc_px = std::round(calGridCfg_.markerExclusionMm * touchCfg_.pixelsPerMm);
    const float sz_mm  = calGridCfg_.markerSizeMm;

    const int availW = touchCfg_.width  - 2 * static_cast<int>(exc_px);
    const int availH = touchCfg_.height - 2 * static_cast<int>(exc_px);
    const int cols = std::max(1, static_cast<int>((availW + gap_px) / (sz_px + gap_px)));
    const int rows = std::max(1, static_cast<int>((availH + gap_px) / (sz_px + gap_px)));

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    for (size_t m = 0; m < ids.size(); ++m) {
        const int id = ids[m];
        if (id < 0 || id >= cols * rows) continue;
        if (corners[m].size() != 4) continue;

        const int col = id % cols;
        const int row = id / cols;
        const float ox_mm = (exc_px + col * (sz_px + gap_px)) * touchCfg_.mmPerPixel;
        const float oy_mm = (exc_px + row * (sz_px + gap_px)) * touchCfg_.mmPerPixel;

        objectPoints.push_back({ox_mm,         oy_mm,         0.0f});
        objectPoints.push_back({ox_mm + sz_mm, oy_mm,         0.0f});
        objectPoints.push_back({ox_mm + sz_mm, oy_mm + sz_mm, 0.0f});
        objectPoints.push_back({ox_mm,         oy_mm + sz_mm, 0.0f});
        for (int k = 0; k < 4; ++k) imagePoints.push_back(corners[m][k]);
    }
    lastScreenMarkers_ = static_cast<int>(objectPoints.size() / 4);
    if (static_cast<int>(objectPoints.size()) < 8) return false;

    cv::Vec3d rvec, tvec;
    if (!cv::solvePnP(objectPoints, imagePoints, camCfg_.cameraMatrix, camCfg_.distCoeffs,
                      rvec, tvec, false, cv::SOLVEPNP_ITERATIVE))
        return false;
    if (!std::isfinite(rvec[0]) || !std::isfinite(rvec[1]) || !std::isfinite(rvec[2]))
        return false;
    cv::Mat Rm;
    cv::Rodrigues(rvec, Rm);
    Rout = toMatx33(Rm);
    return true;
}

// =============================================================================
// World->camera pose from the world-board markers
// (mirrors WorldObjectHandler's world PnP - config centres <-> detected centroids)
// =============================================================================

bool RigAlignmentHandler::SolveWorldPose(const std::vector<int>&                      ids,
                                         const std::vector<std::vector<cv::Point2f>>& corners,
                                         cv::Matx33d&                                 Rout) const {
    std::vector<cv::Point3f> worldObj;   // config centres
    std::vector<cv::Point2f> worldImg;   // detected centroids
    for (size_t m = 0; m < ids.size(); ++m) {
        const auto it = objCfg_.markerPositions.find(ids[m]);
        if (it == objCfg_.markerPositions.end()) continue;
        if (corners[m].size() != 4) continue;
        worldObj.push_back(it->second);
        worldImg.push_back(cornerCentroid(corners[m]));
    }
    lastWorldMarkers_ = static_cast<int>(worldObj.size());
    if (static_cast<int>(worldObj.size()) < 4) return false;

    cv::Vec3d rvec, tvec;
    // 4.0 px inlier threshold, matching WorldObjectHandler's world pose solve.
    if (!cv::solvePnPRansac(worldObj, worldImg, camCfg_.cameraMatrix, camCfg_.distCoeffs,
                            rvec, tvec, false, 100, 4.0f))
        return false;
    if (!std::isfinite(rvec[0]) || !std::isfinite(rvec[1]) || !std::isfinite(rvec[2]))
        return false;
    cv::Mat Rm;
    cv::Rodrigues(rvec, Rm);
    Rout = toMatx33(Rm);
    return true;
}

// =============================================================================
// Status + persistence
// =============================================================================

std::string RigAlignmentHandler::GetStatus() const {
    std::ostringstream ss;
    if (complete_) {
        ss << "Rig alignment COMPLETE (" << count_ << " samples) - saved. Returning to idle.";
        return ss.str();
    }
    ss << "Rig alignment: " << count_ << "/" << kTargetSamples << " captured"
       << "  |  screen grid: " << lastScreenMarkers_ << " mk " << (lastScreenOk_ ? "OK" : "--")
       << "  |  world board: " << lastWorldMarkers_ << " mk " << (lastWorldOk_ ? "OK" : "--")
       << "  -- show BOTH to the camera. [grave] cancels.";
    return ss.str();
}

bool RigAlignmentHandler::Save(const std::string& path) const {
    if (!complete_) return false;
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "RigAlign: could not open " << path << " for writing.\n";
        return false;
    }
    fs << "rig_screen_to_world" << cv::Mat(finalR_);
    fs << "samples" << count_;
    fs << "note" << "screen(Cal3)->world-board rotation; captured by RigAlignmentHandler ('R').";
    fs.release();
    std::cout << "RigAlign: saved rig alignment to " << path << "\n";
    return true;
}
