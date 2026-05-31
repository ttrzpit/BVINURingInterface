#include "DisplayHandler.h"

#include <iostream>

// =============================================================================
// DisplayHandler.cpp — Operator display on the main monitor
//
// The display window shows:
//   - The undistorted color camera frame
//   - Green dots + IDs at each detected marker's centroid  (stub — expand later)
//   - A status bar at the bottom with marker count and touch state (stub)
//
// OpenCV's imshow() queues frames for rendering; the actual update happens
// when pollKey() triggers the GUI event loop via cv::pollKey().
// =============================================================================


DisplayHandler::DisplayHandler(const DisplayConfig& cfg, cv::Point2i principalPoint)
    : cfg_(cfg)
    , principalPoint_(principalPoint)
{
    cv::namedWindow(WIN_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN_NAME, cfg_.width, cfg_.height);
    std::cout << "DisplayHandler: Window '" << WIN_NAME
              << "' created (" << cfg_.width << "x" << cfg_.height << ")\n";
}


// ---- Public -----------------------------------------------------------------

void DisplayHandler::Update(const cv::Mat&                     frame,
                             const std::vector<DetectedMarker>& markers,
                             const TouchState&                  touch) {
    if (frame.empty()) return;

    // Work on a copy so the original frame data is not modified
    cv::Mat canvas = frame.clone();

    DrawCameraElements(canvas);
    DrawMarkerOverlays(canvas, markers);
    DrawTelemetryBar(canvas, markers, touch);

    cv::imshow(WIN_NAME, canvas);
}

int DisplayHandler::PollKey() {
    // cv::pollKey() drives the GUI event system for ALL OpenCV windows.
    // It must be called every iteration — without it windows freeze.
    return cv::pollKey() & 0xFF;
}


// ---- Private ----------------------------------------------------------------

void DisplayHandler::DrawCameraElements(cv::Mat& frame) {
    // Horizontal line across the full frame width at the camera principal point Y
    cv::line(frame,
             cv::Point(0,               principalPoint_.y),
             cv::Point(frame.cols - 1,  principalPoint_.y),
             cv::Scalar(0, 200, 200), 1);

    // Vertical line across the full frame height at the camera principal point X
    cv::line(frame,
             cv::Point(principalPoint_.x, 0),
             cv::Point(principalPoint_.x, frame.rows - 1),
             cv::Scalar(0, 200, 200), 1);
}


void DisplayHandler::DrawMarkerOverlays(cv::Mat& frame,
                                         const std::vector<DetectedMarker>& markers) {
    // TODO: draw corner boxes, axis arrows (cv::drawFrameAxes), and richer
    //       telemetry once task requirements are finalized.
    //
    // Stub: a filled circle at the centroid and the marker ID as text.
    for (const auto& m : markers) {
        cv::circle(frame, m.centerPx, 6, cv::Scalar(0, 255, 0), cv::FILLED);
        cv::putText(frame,
                    "ID " + std::to_string(m.id),
                    m.centerPx + cv::Point2i(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 0), 2);
    }
}

void DisplayHandler::DrawTelemetryBar(cv::Mat&                            frame,
                                       const std::vector<DetectedMarker>& markers,
                                       const TouchState&                   touch) {
    // TODO: add structured panels (3D positions, task state, timing, etc.)
    //       once the task framework is defined.
    //
    // Stub: single status line at the bottom of the frame.
    std::string touchStr = touch.isTouched
        ? ("TOUCH (" + std::to_string(touch.position.x) + ", "
                     + std::to_string(touch.position.y) + ")")
        : "no touch";

    std::string status = "Markers: " + std::to_string(markers.size())
                       + "  |  " + touchStr;

    cv::putText(frame, status,
                cv::Point2i(10, frame.rows - 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(210, 210, 210), 1);
}
