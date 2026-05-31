#pragma once

// =============================================================================
// DisplayHandler.h — Operator display on the main monitor
//
// Shows the undistorted camera frame with overlays for detected markers and
// a telemetry bar at the bottom of the frame.
//
// Stubs (marked TODO) are provided for richer overlays that will be filled in
// once task logic is defined — the interface won't need to change.
//
// Important: pollKey() must be called once per main loop iteration because
// OpenCV's cv::pollKey() is what drives the GUI event system for all windows.
// Without it, windows freeze.
// =============================================================================

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ArucoHandler.h"  // DetectedMarker
#include "Config.h"
#include "TouchHandler.h"  // TouchState


class DisplayHandler {
public:
    /**
     * @param cfg  Display config (window dimensions).
     *             Must outlive this object.
     */
    /**
     * @param cfg             Display config (window dimensions)
     * @param principalPoint  Camera principal point (cx, cy) in pixels —
     *                        used to draw the optical-axis crosshair
     */
    DisplayHandler(const DisplayConfig& cfg, cv::Point2i principalPoint);

    /**
     * @brief Refresh the operator display with the latest frame and data.
     * @param frame    Undistorted color frame from CameraHandler (may be empty)
     * @param markers  Currently detected markers from ArucoHandler
     * @param touch    Current touchscreen state from TouchHandler
     */
    void Update(const cv::Mat&                     frame,
                const std::vector<DetectedMarker>& markers,
                const TouchState&                  touch);

    /**
     * @brief Process pending keyboard events. Returns the key code (0–255),
     *        or -1 if no key was pressed. Call once per main loop iteration.
     */
    int PollKey();

private:
    // Overlay helpers — extend these as task requirements are defined
    void DrawCameraElements(cv::Mat& frame);
    void DrawMarkerOverlays(cv::Mat& frame, const std::vector<DetectedMarker>& markers);
    void DrawTelemetryBar(cv::Mat& frame, const std::vector<DetectedMarker>& markers,
                          const TouchState& touch);

    const DisplayConfig& cfg_;
    cv::Point2i          principalPoint_;   // Camera principal point (cx, cy) [px]

    static constexpr const char* WIN_NAME = "NURing Operator";
};
