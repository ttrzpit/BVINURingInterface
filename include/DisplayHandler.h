#pragma once

// =============================================================================
// DisplayHandler.h — Operator display + Telemetry panel
//
// Two windows are managed here:
//
//   "NURing Operator" (1600×1070) — camera feed with marker overlays.
//
//   "Telemetry" (1600×270) — a 50-column × 6-row grid panel positioned
//   directly below the operator window. Callers populate it each frame using
//   three functions:
//
//     ClearTelemetry()                                  — reset to black
//     AddHeadingCell(text, cell, cols, rows, align, sz) — bold/dark-bg cell
//     AddBodyCell   (text, cell, cols, rows, align, sz) — regular/black-bg cell
//     AddBorder     (cell, cols, rows, color, thickness) — draws a rectangle
//
//   Cell references use Excel-style notation: "A1" through "AX6"
//   (A–Z = cols 1–26, AA–AX = cols 27–50; rows 1–6).
//
//   Optional fill/text colour arguments can be passed after the required ones;
//   they default to the appropriate heading or body style colours.
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
#include "TouchHandler.h"    // TouchState


class DisplayHandler {
public:
    /**
     * @param cfg             Operator display config (size and window position)
     * @param principalPoint  Camera principal point (cx, cy) for the crosshair
     * @param telCfg          Telemetry panel config (size, grid, window position)
     */
    DisplayHandler(const DisplayConfig&    cfg,
                   cv::Point2i             principalPoint,
                   const TelemetryConfig&  telCfg);

    // ---- Operator display ---------------------------------------------------

    /**
     * @brief Refresh the operator display and show the telemetry panel.
     *        Call once per main loop iteration after all AddCell calls.
     */
    void Update(const cv::Mat&                     frame,
                const std::vector<DetectedMarker>& markers,
                const TouchState&                  touch,
                const KeyboardState&               kb);

    /** @brief Poll keyboard events. Must be called every loop. Returns raw key or -1. */
    int PollKey();

    // ---- Telemetry panel ----------------------------------------------------

    /** @brief Reset the telemetry canvas to black. Call at the start of each frame. */
    void ClearTelemetry();

    /**
     * @brief Draw a heading-style cell (DUPLEX font, dark-gray background by default).
     * @param text      Text to display
     * @param cellRef   Top-left cell reference, e.g. "A1", "AB3"
     * @param colSpan   Number of columns to span
     * @param rowSpan   Number of rows to span
     * @param align     "left", "center", or "right"
     * @param fontSize  Font scale factor
     * @param fillColor Background fill color (default: GraDk)
     * @param textColor Text color (default: White)
     */
    void AddHeadingCell(const std::string& text,
                        const std::string& cellRef,
                        int colSpan, int rowSpan,
                        const std::string& align,
                        float fontSize,
                        const cv::Scalar& fillColor = Colors::GraDk,
                        const cv::Scalar& textColor = Colors::White);

    /**
     * @brief Draw a body-style cell (SIMPLEX font, black background by default).
     */
    void AddBodyCell(const std::string& text,
                     const std::string& cellRef,
                     int colSpan, int rowSpan,
                     const std::string& align,
                     float fontSize,
                     const cv::Scalar& fillColor = Colors::GraBk,
                     const cv::Scalar& textColor = Colors::White);

    /**
     * @brief Draw a colored border rectangle around a span of cells.
     * @param cellRef    Top-left cell of the border
     * @param colSpan    Width in cells
     * @param rowSpan    Height in cells
     * @param color      Border color (use Colors::*)
     * @param thickness  Line width in pixels
     */
    void AddBorder(const std::string& cellRef,
                   int colSpan, int rowSpan,
                   const cv::Scalar& color,
                   int thickness);

private:
    // ---- Operator display helpers -------------------------------------------
    void DrawCameraElements(cv::Mat& frame);
    void DrawMarkerOverlays(cv::Mat& frame, const std::vector<DetectedMarker>& markers, int activeTagId);
    void DrawTelemetryBar(cv::Mat& frame, const std::vector<DetectedMarker>& markers,
                          const TouchState& touch, const KeyboardState& kb);

    // ---- Telemetry grid helpers ---------------------------------------------
    enum class CellStyle { HEADING, BODY };

    /** @brief Populate the telemetry grid each frame. Called automatically by Update(). */
    void PopulateTelemetryPanel(const std::vector<DetectedMarker>& markers,
                                const TouchState&                  touch,
                                const KeyboardState&               kb);

    /** @brief Shared implementation for AddHeadingCell / AddBodyCell. */
    void DrawTelCell(const std::string& text,
                     const std::string& cellRef,
                     int colSpan, int rowSpan,
                     const std::string& align,
                     float fontSize,
                     const cv::Scalar& textColor,
                     const cv::Scalar& fillColor,
                     CellStyle style);

    /** @brief Parse "A1", "AB3" etc. into a 0-indexed (col, row) grid coordinate. */
    cv::Point2i ParseCellRef(const std::string& ref) const;

    /** @brief Return the pixel rect for a cell span. */
    cv::Rect    CellRect(const std::string& ref, int colSpan, int rowSpan) const;

    // ---- Members ------------------------------------------------------------
    const DisplayConfig&   cfg_;
    const TelemetryConfig& telCfg_;
    cv::Point2i            principalPoint_;

    int cellW_;   // Telemetry cell width  in pixels (telCfg_.width  / telCfg_.cols)
    int cellH_;   // Telemetry cell height in pixels (telCfg_.height / telCfg_.rows)

    cv::Mat matTelemetry_;   // Canvas for the telemetry panel

    static constexpr const char* WIN_OPERATOR  = "NURing Operator";
    static constexpr const char* WIN_TELEMETRY = "Telemetry";
};
