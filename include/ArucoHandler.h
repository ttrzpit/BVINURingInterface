#pragma once

// =============================================================================
// ArucoHandler.h — ArUco marker detection (threaded) and touchscreen grid
//
// Detection runs on a dedicated background thread so the main loop never
// blocks waiting for the OpenCV detector. The result is at most one frame
// stale (~11 ms at 88 Hz), which is imperceptible for a guidance system.
//
// Usage:
//   aruco.Start();                         // launch detection thread once
//   // each main loop iteration:
//   aruco.SubmitFrame(frame.gray);         // non-blocking, hands off frame
//   auto markers = aruco.GetLatestDetection(); // non-blocking, reads last result
//   aruco.Stop();                          // join thread on shutdown
//
// Display functions (showMarkerGrid, updateGridConfig) remain on the main
// thread — they must not be called from the detection thread.
// =============================================================================

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "Config.h"


// ---- Output type ------------------------------------------------------------

/**
 * @brief Data for a single detected ArUco marker in a camera frame.
 */
struct DetectedMarker {
    int                        id;          ///< Marker ID from the dictionary
    cv::Point2i                centerPx;    ///< Pixel-space centroid
    std::array<cv::Point2f, 4> cornersPx;  ///< Four corners, clockwise from top-left
    cv::Point3f                positionMm;  ///< 3D position relative to camera [mm]
    float                      rotationDeg;///< Rotation about the Y-axis [degrees]
};


// ---- Handler ----------------------------------------------------------------

class ArucoHandler {
public:
    ArucoHandler(const ArucoDetectConfig&   detectCfg,
                 const ArucoDetectorConfig& detectorCfg,
                 const ArucoDisplayConfig&  displayCfg,
                 const TouchscreenConfig&   touchCfg,
                 const cv::Mat&             camMatrix,
                 const cv::Mat&             distCoeffs);

    ~ArucoHandler();

    // ---- Detection (threaded) -----------------------------------------------

    /** @brief Launch the background detection thread. Call once at startup. */
    void Start();

    /** @brief Signal the detection thread to stop and wait for it to exit. */
    void Stop();

    /**
     * @brief Submit a new grayscale frame for detection. Non-blocking.
     *        If a previous frame is still being processed, it is replaced —
     *        the detector always works on the most recent frame.
     */
    void SubmitFrame(const cv::Mat& grayFrame);

    /**
     * @brief Return the most recently completed detection result. Non-blocking.
     *        Returns an empty vector if no detection has completed yet.
     */
    std::vector<DetectedMarker> GetLatestDetection();

    // ---- Touchscreen display (main thread only) -----------------------------

    /**
     * @brief Show or hide the ArUco marker grid on the touchscreen.
     *        Showing opens and fullscreens the window; hiding destroys it.
     *        Idempotent — safe to call repeatedly with the same value.
     */
    void SetGridVisible(bool visible);

    /**
     * @brief Replace the touchscreen image with a single marker at its grid
     *        position, hiding all others. Used for the Fitts pointing task.
     *        The grid window must already be visible (call SetGridVisible first).
     * @param id  Marker ID to display (1–45)
     */
    void ShowSingleMarker(int id);

    void updateGridConfig(int cols, int rows, float markerSizeMm, float paddingMm);

private:
    // ---- Detection thread ---------------------------------------------------
    void DetectLoop();
    std::vector<DetectedMarker> RunDetection(const cv::Mat& grayFrame);

    // ---- Display helpers ----------------------------------------------------
    void        initDetector();
    void        renderGridImage();
    cv::Point2i gridCellOrigin(int col, int row) const;

    // ---- Configuration ------------------------------------------------------
    ArucoDetectConfig   detectCfg_;
    ArucoDetectorConfig detectorCfg_;
    ArucoDisplayConfig  displayCfg_;
    TouchscreenConfig   touchCfg_;
    cv::Mat             camMatrix_;
    cv::Mat             distCoeffs_;

    // ---- OpenCV detector ----------------------------------------------------
    cv::aruco::Dictionary        dictionary_;
    cv::aruco::DetectorParameters detectorParams_;
    cv::aruco::ArucoDetector     detector_;
    cv::Mat                      markerCorners3D_{ 4, 1, CV_32FC3 };

    // ---- Display ------------------------------------------------------------
    cv::Mat markerGridImage_;
    bool    gridVisible_ = false;   // Tracks open/closed state for idempotent SetGridVisible
    static constexpr const char* TOUCHSCREEN_WIN = "ArUco Display";

    // ---- Thread infrastructure ----------------------------------------------
    std::thread             detectThread_;
    std::atomic<bool>       detectRunning_{ false };

    // Input slot: main loop writes here, detection thread reads
    std::mutex              frameMutex_;
    std::condition_variable frameCv_;
    cv::Mat                 pendingFrame_;
    bool                    frameReady_ = false;

    // Output slot: detection thread writes here, main loop reads
    std::mutex                  resultMutex_;
    std::vector<DetectedMarker> latestResult_;
};
