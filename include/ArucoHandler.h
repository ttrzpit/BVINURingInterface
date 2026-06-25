#pragma once

// =============================================================================
// ArucoHandler.h - ArUco marker detection (threaded) and touchscreen grid
//
// Detection runs on a dedicated background thread so the main loop never
// blocks waiting for the OpenCV detector. Detection can take longer than one
// camera frame period, so the result may be a few frames stale - main.cpp
// pairs the displayed frame with GetLatestDetectionTimestamp() so the
// overlay never drifts relative to the image underneath.
//
// Usage:
//   aruco.Start();                              // launch detection thread once
//   // each main loop iteration:
//   aruco.SubmitFrame(frame.gray, frame.timestamp); // non-blocking, hands off frame
//   auto markers = aruco.GetLatestDetection();  // non-blocking, reads last result
//   aruco.Stop();                               // join thread on shutdown
//
// Display functions (showMarkerGrid, updateGridConfig) remain on the main
// thread - they must not be called from the detection thread.
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
#include "FittsBoardLayout.h"


// ---- Output type ------------------------------------------------------------

/**
 * @brief Data for a single detected ArUco marker in a camera frame.
 */
struct DetectedMarker {
    int                        id = 0;          ///< Marker ID from the dictionary
    cv::Point2i                centerPx;        ///< Pixel-space centroid
    std::array<cv::Point2f, 4> cornersPx;       ///< Four corners, clockwise from top-left
    cv::Point3f                positionMm;       ///< 3D position relative to camera [mm] (only set for the active target)
    float                      rotationDeg = 0.0f;  ///< Rotation about the Y-axis [degrees] (only set for the active target)
    float                      rollRad = 0.0f;      ///< In-plane roll from the marker's top edge [rad]
};


// ---- Handler ----------------------------------------------------------------

class ArucoHandler {
public:
    ArucoHandler(const ArucoMarkerConfig&          detectCfg,
                 const ArucoDetectorConfig&        detectorCfg,
                 const ArucoDisplayConfig&         displayCfg,
                 const ArucoCalibrationGridConfig& calGridCfg,
                 const FittsBoardConfig&           fittsBoardCfg,
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
     *        If a previous frame is still being processed, it is replaced -
     *        the detector always works on the most recent frame.
     * @param grayFrame  Grayscale frame to detect markers in
     * @param timestamp  Capture timestamp of this frame (CameraFrame::timestamp) -
     *                    echoed back by GetLatestDetectionTimestamp() for lag diagnostics.
     */
    void SubmitFrame(const cv::Mat& grayFrame, double timestamp);

    /**
     * @brief Return the most recently completed detection result. Non-blocking.
     *        Returns an empty vector if no detection has completed yet.
     */
    std::vector<DetectedMarker> GetLatestDetection();

    /** @brief Capture timestamp (CameraFrame::timestamp) of the frame that
     *         produced the latest detection result - used to pair the
     *         displayed frame with the marker overlay (see main.cpp). */
    double GetLatestDetectionTimestamp() const { return resultTimestamp_.load(); }

    // ---- Touchscreen display (main thread only) -----------------------------

    /**
     * @brief Show or hide the ArUco marker grid on the touchscreen.
     *        Showing opens and fullscreens the window; hiding destroys it.
     *        Idempotent - safe to call repeatedly with the same value.
     */
    void SetGridVisible(bool visible);

    /**
     * @brief Open the touchscreen window and show a blank white screen.
     *        Used for the Fitts task start state - participant sees a clean
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
     * @brief Show or hide the multi-scale Fitts board on the touchscreen
     *        (coarse perimeter markers + dense fine grid, DICT_4X4_1000).
     *        The board is persistent - the active target is indicated by the
     *        target-offset circle, not by hiding markers. Idempotent.
     */
    void SetFittsBoardVisible(bool visible);

    /**
     * @brief Switch detection for the multi-scale Fitts board.
     *        true  → DICT_4X4_1000, IDs 0–board max, per-marker physical size
     *                resolved from the board layout (coarse vs fine).
     *        false → restore DICT_4X4_50 and the default ID range / marker size.
     *        Thread-safe - takes effect on the next detection cycle.
     */
    void SetFittsBoardDetection(bool fitts);

    /** @brief Set the active guidance target ID. The detection thread solves
     *         full 3D pose only for this marker; pass 0 when no target is active.
     *         Thread-safe - takes effect on the next detection cycle. */
    void SetActiveTagId(int id) { activeTagId_.store(id); }

    /** @brief Number of fine (pointing-target) markers on the Fitts board.
     *         Target IDs span [GetFittsTargetIdMin(), that min + count - 1]. */
    int GetFittsTargetCount() const { return fittsLayout_.FineCount(); }
    int GetFittsTargetIdMin() const { return fittsLayout_.FineIdMin(); }
    int GetFittsTargetIdMax() const { return fittsLayout_.FineIdMax(); }
    /** @brief Largest marker ID on the board (includes coarse markers). Used
     *         to allow manual target entry of coarse IDs for testing. */
    int GetFittsBoardMaxId()  const { return fittsLayout_.MaxId(); }

    /** @brief Fine marker IDs eligible as random targets (interior of the grid,
     *         border rows/cols excluded). */
    const std::vector<int>& GetFittsSelectableTargetIds() const {
        return fittsLayout_.SelectableTargetIds();
    }

    /**
     * @brief Draw (or clear) the Fitts touch-sample overlay on top of the
     *        current Fitts target image: a dot at the touch position, an error
     *        triangle from the touch point to the target marker centre (black
     *        hypotenuse, green vertical leg, red horizontal leg), and two lines
     *        of endpoint-error readout text along the bottom of the screen.
     *        Persists until the next ShowSingleMarker() call (next target).
     * @param visible  Draw the overlay
     * @param touchPx  Touch position in touchscreen-local pixels
     * @param targetId Fitts board marker id of the active target (for its centre)
     * @param line1    First error readout line ("Endpoint error [PX] = ...")
     * @param line2    Second error readout line ("Endpoint error [mm] = ...")
     */
    void SetFittsOverlay(bool visible, cv::Point2i touchPx, int targetId,
                         const std::string& line1, const std::string& line2);

    /**
     * @brief Draw (or clear) the Fitts target-offset circle: a hollow circle
     *        marking the calibrated touch target location (marker center +
     *        Cal3 camera-to-fingertip offset), or the default "under the tag"
     *        offset before Cal3 completes. Persists on top of the current
     *        Fitts target image until cleared or a new target is shown
     *        (ShowSingleMarker()/ShowBlankTouchscreen()).
     * @param visible  Draw the circle
     * @param centerPx Circle center in touchscreen-local pixels
     * @param radiusPx Circle radius in touchscreen-local pixels
     * @param color    Circle outline color (e.g. Colors::RedMd once Cal3 is
     *                  complete, Colors::GraMd for the uncalibrated default)
     */
    void SetTargetOffsetCircle(bool visible, cv::Point2i centerPx, int radiusPx, cv::Scalar color);

    /** @brief Show/hide a magenta reference outline around the target marker
     *         (by Fitts layout id) on the touchscreen, drawn once the trial-ending
     *         touch is registered and kept until the next target is selected. */
    void SetTargetOutline(bool visible, int targetId);

    /**
     * @brief Pixel-space center of the Fitts board marker `id`, in
     *        touchscreen-local pixels. Resolved from the board layout (fine
     *        target IDs and coarse IDs both work). Returns {0,0} if `id` is not
     *        on the board.
     */
    cv::Point2i GetGridMarkerCenterPx(int id) const;

    void updateGridConfig(int cols, int rows, float markerSizeMm, float paddingMm);

    /**
     * @brief Show or hide the dense calibration grid on the touchscreen.
     *        Layout is auto-calculated from ArucoCalibrationGridConfig:
     *        markers are packed with fixed markerPadMm spacing inside the
     *        markerExclusionMm boundary. Uses DICT_4X4_1000 (IDs start at 0).
     *        Idempotent - safe to call repeatedly with the same value.
     */
    void SetCalibrationGridVisible(bool visible);

    /**
     * @brief Switch the detection dictionary and valid ID range.
     *        true  → DICT_4X4_1000, IDs 0–999 (Cal3 calibration grid)
     *        false → DICT_4X4_50,  IDs from aruco_marker config (default)
     *        Thread-safe - takes effect on the next detection cycle.
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
    void        renderFittsBoardImage();

    /** @brief Redraw singleMarkerImage_ with the target-offset circle (if
     *         visible) and the Fitts touch overlay (if visible), then
     *         cv::imshow(). Called by SetTargetOffsetCircle()/SetFittsOverlay(). */
    void        RedrawTouchscreenOverlay();

    // ---- Configuration ------------------------------------------------------
    ArucoMarkerConfig          detectCfg_;
    ArucoDetectorConfig        detectorCfg_;
    ArucoDisplayConfig         displayCfg_;
    ArucoCalibrationGridConfig calGridCfg_;
    TouchscreenConfig          touchCfg_;
    FittsBoardLayout           fittsLayout_;
    cv::Mat                    camMatrix_;
    cv::Mat                    distCoeffs_;

    // ---- OpenCV detector ----------------------------------------------------
    cv::aruco::Dictionary         dictionary_;       // DICT_4X4_50  - Fitts / default
    cv::aruco::DetectorParameters detectorParams_;
    cv::aruco::ArucoDetector      detector_;         // built from dictionary_
    cv::aruco::ArucoDetector      calDetector_;      // built from calGridDictionary_ (DICT_4X4_1000)

    // Active detection mode - written from main thread, read from detect thread.
    // Atomics avoid the need for a mutex in the RunDetection() hot path.
    std::atomic<bool> useCalDetector_{ false };
    std::atomic<int>  activeValidIdMin_{ 0 };
    std::atomic<int>  activeValidIdMax_{ 45 };

    // When true, per-marker physical size is resolved from fittsLayout_ (coarse
    // vs fine) during pose estimation instead of the single detectCfg_ size.
    // Read on the detection thread; fittsLayout_ is immutable after construction.
    std::atomic<bool> useFittsBoardSizes_{ false };

    // ID of the active guidance target. The detection thread solves full 3D
    // pose only for this marker (the only one whose pose is consumed), which
    // keeps detection cheap on the dense Fitts board. 0 = none.
    std::atomic<int> activeTagId_{ 0 };

    // ---- Display ------------------------------------------------------------
    cv::Mat markerGridImage_;
    bool    gridVisible_    = false;   // Fitts grid open/closed state
    cv::Mat calGridImage_;
    bool    calGridVisible_ = false;   // Calibration grid open/closed state
    cv::Mat fittsBoardImage_;
    bool    fittsBoardVisible_ = false;  // Multi-scale Fitts board open/closed state
    cv::Size calGridSize_   = {};      // Auto-calculated cols/rows for the calibration grid

    // Fitts touch-sample overlay - drawn on top of singleMarkerImage_
    cv::Mat     singleMarkerImage_;        // BGR base image for the current Fitts target
    bool        fittsOverlayVisible_ = false;
    cv::Point2i fittsTouchPx_        = {};
    int         fittsOverlayTargetId_ = 0;    // Active target id - for the error-triangle target centre
    std::string fittsLine1_;
    std::string fittsLine2_;

    // Fitts target-offset circle (hollow) - drawn on top of singleMarkerImage_
    bool        targetCircleVisible_   = false;
    cv::Point2i targetCirclePx_        = {};
    int         targetCircleRadiusPx_  = 0;
    cv::Scalar  targetCircleColor_     = { 0, 0, 255 };  // BGR, default red

    // Magenta reference outline around the just-touched target marker, drawn on
    // the touchscreen from the trial-ending touch until the next target loads.
    bool        targetOutlineVisible_  = false;
    int         targetOutlineId_       = 0;

    // Calibration grid uses a larger dictionary (DICT_4X4_1000) so it can
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
    double                  pendingTimestamp_ = 0.0;
    bool                    frameReady_ = false;

    // Output slot: detection thread writes here, main loop reads
    std::mutex                  resultMutex_;
    std::vector<DetectedMarker> latestResult_;

    // Timestamp of the frame behind latestResult_ - written by the detection
    // thread, read from the main thread, so a plain atomic (no mutex) suffices.
    std::atomic<double> resultTimestamp_{0.0};
};
