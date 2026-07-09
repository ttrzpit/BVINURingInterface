#pragma once

// =============================================================================
// CameraHandler.h - USB camera capture with GPU-accelerated preprocessing
//
// Runs capture on a dedicated background thread so the main loop never blocks
// waiting for a new frame. Each iteration the camera thread:
//   1. grab()   - blocks until the camera delivers a new frame at its native rate
//                 (the capture timestamp is taken here, before any processing)
//   2. retrieve() - decodes the MJPEG frame
//   3. GPU: undistort (remap) + convert to grayscale + CLAHE contrast enhancement
//   4. Publishes the result into a mutex-protected latestFrame slot and
//      notifies WaitForFrame() waiters
//
// The main loop calls WaitForFrame() to sleep until a new frame is published
// (bounded by a timeout so it still services keys/serial), then
// getLatestFrame() to copy the slot non-blockingly.
// =============================================================================

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "Config.h"


// ---- Output type ------------------------------------------------------------

/**
 * @brief One preprocessed camera frame, ready for display and ArUco detection.
 */
struct CameraFrame {
    cv::Mat  undistorted;        ///< Color BGR, GPU-undistorted - for DisplayHandler
    cv::Mat  gray;               ///< Grayscale, CLAHE-enhanced - for ArucoHandler
    bool     ready     = false;  ///< True when the frame contains valid data
    double   timestamp = 0.0;   ///< Capture time [seconds, from cv::getTickCount]
};


// ---- Handler ----------------------------------------------------------------

class CameraHandler {
public:
    /**
     * @param cfg  Camera configuration (device path, resolution, intrinsics, etc.)
     *             Must outlive this object.
     */
    explicit CameraHandler(const CameraConfig& cfg);
    ~CameraHandler();

    /** @brief Open the camera device and launch the capture thread. */
    void start();

    /** @brief Signal the capture thread to stop and wait for it to finish. */
    void stop();

    /**
     * @brief Return a copy of the most recently completed frame. Non-blocking.
     *        Returns frame.ready == false if no frame has been captured yet.
     */
    CameraFrame getLatestFrame();

    /**
     * @brief Block until a frame newer than lastTimestamp is published, or the
     *        timeout elapses. Returns true if a new frame is available. This is
     *        the main loop's pacing primitive - it replaces the former busy-spin
     *        (the loop previously re-polled getLatestFrame() at unbounded rate,
     *        burning a core and starving the detection thread).
     * @param lastTimestamp  Timestamp of the frame the caller has already seen
     * @param timeoutMs      Upper bound on the wait [ms]
     */
    bool WaitForFrame(double lastTimestamp, int timeoutMs);

private:
    void captureLoop();       // Runs on captureThread_
    void openCamera();        // Open VideoCapture and apply all hardware settings
    void buildUndistortMap(); // Precompute GPU remap tables from intrinsics

    const CameraConfig& cfg_;

    cv::VideoCapture camera_;

    // GPU pipeline resources (allocated once, reused every frame)
    cv::cuda::GpuMat gpuRaw_, gpuUndistorted_, gpuGray_, gpuGrayEq_;
    cv::cuda::GpuMat gpuRemap1_, gpuRemap2_;
    cv::Mat          cpuRemap1_, cpuRemap2_;   // Source for the GPU remap tables

    // Thread-shared frame slot - protected by frameMutex_. frameCv_ is
    // notified on every publish so WaitForFrame() wakes immediately.
    std::mutex              frameMutex_;
    std::condition_variable frameCv_;
    CameraFrame             latestFrame_;

    // Thread lifecycle
    std::thread       captureThread_;
    std::atomic<bool> running_{ false };
};
