#pragma once

// =============================================================================
// DisplayHandler.h — Operator display + Telemetry panel + Controller panel
//
// Three windows are managed here:
//
//   "NURing Operator"  — camera feed with marker overlays.
//   "System Information" — Telemetry grid panel (see AddHeadingCell etc.)
//   "Controller"         — Controller panel (see AddControllerHeadingCell etc.)
//
// Both grid panels use Excel-style cell references ("A1", "B3", "AB2").
// Cell functions follow the same signature pattern:
//   Add*Cell(text, cellRef, colSpan, rowSpan, align, fontSize [, fill, text])
//   Add*Border(cellRef, colSpan, rowSpan, color, thickness)
//
// PollKey() must be called once per main loop iteration — it drives the OpenCV
// window event system for ALL windows.
// =============================================================================

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ArucoHandler.h"    // DetectedMarker
#include "Colors.h"
#include "Config.h"
#include "KeyboardHandler.h" // KeyboardState
#include "PacketTypes.h"     // SerialState
#include "TouchHandler.h"    // TouchState


class DisplayHandler {
public:
    /**
     * @param cfg              Operator display config (size and window position)
     * @param principalPoint   Camera principal point (cx, cy) for the crosshair
     * @param telCfg           Telemetry panel config
     * @param controllerCfg    Controller panel config
     */
    DisplayHandler(const DisplayConfig &cfg,
                   cv::Point2i principalPoint,
                   const TelemetryConfig &telCfg,
                   const ControllerPanelConfig &controllerCfg);

    // ---- Operator display ---------------------------------------------------

    /** @brief Refresh all windows. Call once per main loop iteration. */
    void Update(const cv::Mat &frame, const std::vector<DetectedMarker> &markers,
                const TouchState &touch, const KeyboardState &kb,
                const SerialState &serial);

    /** @brief Poll keyboard events. Must be called every loop. Returns raw key or -1. */
    int PollKey();

    // ---- Telemetry panel ----------------------------------------------------

    void ClearTelemetry();

    void AddHeadingCell(const std::string &text, const std::string &cellRef,
                        int colSpan, int rowSpan, const std::string &align,
                        float fontSize,
                        const cv::Scalar &fillColor = Colors::GraDk,
                        const cv::Scalar &textColor = Colors::White);

    void AddSubheadingCell(const std::string &text, const std::string &cellRef,
                           int colSpan, int rowSpan, const std::string &align,
                           float fontSize,
                           const cv::Scalar &fillColor = Colors::GraBk,
                           const cv::Scalar &textColor = Colors::White);

    void AddBodyCell(const std::string &text, const std::string &cellRef,
                     int colSpan, int rowSpan, const std::string &align,
                     float fontSize,
                     const cv::Scalar &fillColor = Colors::Black,
                     const cv::Scalar &textColor = Colors::White);

    void AddBorder(const std::string &cellRef, int colSpan, int rowSpan,
                   const cv::Scalar &color, int thickness);

    // ---- Controller panel ---------------------------------------------------

    void ClearController();

    void AddControllerHeadingCell(const std::string &text, const std::string &cellRef,
                                  int colSpan, int rowSpan, const std::string &align,
                                  float fontSize,
                                  const cv::Scalar &fillColor = Colors::GraDk,
                                  const cv::Scalar &textColor = Colors::White);

    void AddControllerSubheadingCell(const std::string &text, const std::string &cellRef,
                                     int colSpan, int rowSpan, const std::string &align,
                                     float fontSize,
                                     const cv::Scalar &fillColor = Colors::GraBk,
                                     const cv::Scalar &textColor = Colors::White);

    void AddControllerBodyCell(const std::string &text, const std::string &cellRef,
                               int colSpan, int rowSpan, const std::string &align,
                               float fontSize,
                               const cv::Scalar &fillColor = Colors::Black,
                               const cv::Scalar &textColor = Colors::White);

    void AddControllerBorder(const std::string &cellRef, int colSpan, int rowSpan,
                             const cv::Scalar &color, int thickness);

private:
    // ---- Operator display helpers -------------------------------------------
    void DrawCameraElements(cv::Mat &frame);
    void DrawMarkerOverlays(cv::Mat &frame,
                            const std::vector<DetectedMarker> &markers,
                            int activeTagId);
    void DrawTelemetryBar(cv::Mat &frame,
                          const std::vector<DetectedMarker> &markers,
                          const TouchState &touch, const KeyboardState &kb);

    // ---- Shared cell infrastructure -----------------------------------------
    enum class CellStyle { HEADING, SUBHEADING, BODY };

    /** @brief Parse "A1", "AB3" etc. into a 0-indexed (col, row) grid coordinate. */
    cv::Point2i ParseCellRef(const std::string &ref) const;

    // ---- Telemetry grid helpers ---------------------------------------------
    void PopulateTelemetryPanel(const std::vector<DetectedMarker> &markers,
                                const TouchState &touch, const KeyboardState &kb,
                                const SerialState &serial);

    void DrawTelCell(const std::string &text, const std::string &cellRef,
                     int colSpan, int rowSpan, const std::string &align,
                     float fontSize, const cv::Scalar &textColor,
                     const cv::Scalar &fillColor, CellStyle style);

    cv::Rect CellRect(const std::string &ref, int colSpan, int rowSpan) const;

    // ---- Controller grid helpers --------------------------------------------
    void PopulateControllerPanel(const std::vector<DetectedMarker> &markers,
                                 const TouchState &touch, const KeyboardState &kb);

    void DrawControllerCell(const std::string &text, const std::string &cellRef,
                            int colSpan, int rowSpan, const std::string &align,
                            float fontSize, const cv::Scalar &textColor,
                            const cv::Scalar &fillColor, CellStyle style);

    cv::Rect ControllerCellRect(const std::string &ref, int colSpan, int rowSpan) const;

    // ---- Members ------------------------------------------------------------
    const DisplayConfig &cfg_;
    const TelemetryConfig &telCfg_;
    const ControllerPanelConfig &controllerCfg_;
    cv::Point2i principalPoint_;

    // Telemetry panel
    int     cellW_;          // width  / cols
    int     cellH_;          // height / rows
    cv::Mat matTelemetry_;

    // Controller panel
    int     controllerCellW_;  // width  / cols
    int     controllerCellH_;  // height / rows
    cv::Mat matController_;

    // Loop frequency measurement
    int    freqFrameCount_  = 0;
    double freqWindowStart_ = 0.0;
    float  measuredFreqHz_  = 0.0f;

    // Panel update throttle — telemetry and controller panels refresh at 10 Hz
    double lastPanelUpdateTime_ = 0.0;

    static constexpr const char *WIN_OPERATOR   = "NURing Operator";
    static constexpr const char *WIN_TELEMETRY  = "System Information";
    static constexpr const char *WIN_CONTROLLER = "Controller";
};
