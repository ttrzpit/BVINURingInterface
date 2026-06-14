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
    ArucoHandler(const ArucoMarkerConfig&          detectCfg,
                 const ArucoDetectorConfig&        detectorCfg,
                 const ArucoDisplayConfig&         displayCfg,
                 const ArucoCalibrationGridConfig& calGridCfg,
                 const TouchscreenConfig&          touchCfg,
                 const cv::Mat&                    camMatrix,
                 const cv::Mat&                    distCoeffs);

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
     * @brief Open the touchscreen window and show a blank white screen.
     *        Used for the Fitts task start state — participant sees a clean
     *        white display until the first target marker is selected via 'r'.
     *        No-op if the window is already open (just redraws white).
     */
    void ShowBlankTouchscreen();

    /**
     * @brief Replace the touchscreen image with a single marker at its grid
     *        position, hiding all others. Used for the Fitts pointing task.
     *        The grid window must already be visible (call SetGridVisible first).
     * @param id  Marker ID to display (1–45)
     */
    void ShowSingleMarker(int id);

    /**
     * @brief Draw (or clear) the Fitts touch-sample overlay on top of the
     *        current Fitts target image: a red dot at the touch position
     *        plus two lines of error-readout text in the top-left corner.
     *        Persists until the next ShowSingleMarker() call (next target).
     * @param visible  Draw the overlay
     * @param touchPx  Touch position in touchscreen-local pixels
     * @param line1    First error readout line
     * @param line2    Second error readout line
     */
    void SetFittsOverlay(bool visible, cv::Point2i touchPx,
                         const std::string& line1, const std::string& line2);

    void updateGridConfig(int cols, int rows, float markerSizeMm, float paddingMm);

    /**
     * @brief Show or hide the dense calibration grid on the touchscreen.
     *        Layout is auto-calculated from ArucoCalibrationGridConfig:
     *        markers are packed with fixed markerPadMm spacing inside the
     *        markerExclusionMm boundary. Uses DICT_4X4_250 (IDs start at 0).
     *        Idempotent — safe to call repeatedly with the same value.
     */
    void SetCalibrationGridVisible(bool visible);

    /**
     * @brief Switch the detection dictionary and valid ID range.
     *        true  → DICT_4X4_250, IDs 0–249 (Cal3 calibration grid)
     *        false → DICT_4X4_50,  IDs from aruco_marker config (default)
     *        Thread-safe — takes effect on the next detection cycle.
     */
    void SetCalibrationDetection(bool calibration);

    /**
     * @brief Return the auto-calculated grid dimensions for the calibration grid.
     *        Useful for building the 3D point array in Cal3Handler solvePnP.
     */
    cv::Size GetCalibrationGridSize() const { return calGridSize_; }

private:
    // ---- Detection thread ---------------------------------------------------
    void DetectLoop();
    std::vector<DetectedMarker> RunDetection(const cv::Mat& grayFrame);

    // ---- Display helpers ----------------------------------------------------
    void        initDetector();
    void        renderGridImage();
    cv::Point2i gridCellOrigin(int col, int row) const;
    void        renderCalibrationGridImage();

    // ---- Configuration ------------------------------------------------------
    ArucoMarkerConfig          detectCfg_;
    ArucoDetectorConfig        detectorCfg_;
    ArucoDisplayConfig         displayCfg_;
    ArucoCalibrationGridConfig calGridCfg_;
    TouchscreenConfig          touchCfg_;
    cv::Mat                    camMatrix_;
    cv::Mat                    distCoeffs_;

    // ---- OpenCV detector ----------------------------------------------------
    cv::aruco::Dictionary         dictionary_;       // DICT_4X4_50  — Fitts / default
    cv::aruco::DetectorParameters detectorParams_;
    cv::aruco::ArucoDetector      detector_;         // built from dictionary_
    cv::aruco::ArucoDetector      calDetector_;      // built from calGridDictionary_ (DICT_4X4_250)

    // Active detection mode — written from main thread, read from detect thread.
    // Atomics avoid the need for a mutex in the RunDetection() hot path.
    std::atomic<bool> useCalDetector_{ false };
    std::atomic<int>  activeValidIdMin_{ 0 };
    std::atomic<int>  activeValidIdMax_{ 45 };

    // ---- Display ------------------------------------------------------------
    cv::Mat markerGridImage_;
    bool    gridVisible_    = false;   // Fitts grid open/closed state
    cv::Mat calGridImage_;
    bool    calGridVisible_ = false;   // Calibration grid open/closed state
    cv::Size calGridSize_   = {};      // Auto-calculated cols/rows for the calibration grid

    // Fitts touch-sample overlay — drawn on top of singleMarkerImage_
    cv::Mat     singleMarkerImage_;        // BGR base image for the current Fitts target
    bool        fittsOverlayVisible_ = false;
    cv::Point2i fittsTouchPx_        = {};
    std::string fittsLine1_;
    std::string fittsLine2_;

    // Calibration grid uses a larger dictionary (DICT_4X4_250) so it can
    // accommodate more markers than the Fitts grid (DICT_4X4_50).
    cv::aruco::Dictionary calGridDictionary_;

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
