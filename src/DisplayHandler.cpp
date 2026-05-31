#include "DisplayHandler.h"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>

// =============================================================================
// DisplayHandler.cpp
//
// Two windows:
//   WIN_OPERATOR  — camera feed + overlays
//   WIN_TELEMETRY — 50×6 cell grid panel
//
// Telemetry grid coordinate system:
//   Columns: A–Z (0–25), AA–AX (26–49)  (50 total)
//   Rows:    1–6 (0-indexed internally as 0–5)
//   Origin:  top-left corner of the telemetry canvas
// =============================================================================


// =============================================================================
// Construction
// =============================================================================

DisplayHandler::DisplayHandler(const DisplayConfig&   cfg,
                                cv::Point2i            principalPoint,
                                const TelemetryConfig& telCfg)
    : cfg_(cfg)
    , telCfg_(telCfg)
    , principalPoint_(principalPoint)
    , cellW_(telCfg.width  / telCfg.cols)
    , cellH_(telCfg.height / telCfg.rows)
    , matTelemetry_(telCfg.height, telCfg.width, CV_8UC3, Colors::Black)
{
    // ---- Operator display window --------------------------------------------
    cv::namedWindow(WIN_OPERATOR, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN_OPERATOR, cfg_.width, cfg_.height);
    cv::moveWindow(WIN_OPERATOR, cfg_.xPos, cfg_.yPos);

    // ---- Telemetry window ---------------------------------------------------
    cv::namedWindow(WIN_TELEMETRY, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN_TELEMETRY, telCfg_.width, telCfg_.height);
    cv::moveWindow(WIN_TELEMETRY, telCfg_.xPos, telCfg_.yPos);

    std::cout << "DisplayHandler: Operator window " << cfg_.width << "x" << cfg_.height
              << " at (" << cfg_.xPos << ", " << cfg_.yPos << ")\n";
    std::cout << "DisplayHandler: Telemetry window " << telCfg_.width << "x" << telCfg_.height
              << " — " << telCfg_.cols << " cols x " << telCfg_.rows << " rows"
              << " — cell " << cellW_ << "x" << cellH_ << " px"
              << " — at (" << telCfg_.xPos << ", " << telCfg_.yPos << ")\n";
}


// =============================================================================
// Operator display — public
// =============================================================================

void DisplayHandler::Update(const cv::Mat&                     frame,
                             const std::vector<DetectedMarker>& markers,
                             const TouchState&                  touch,
                             const KeyboardState&               kb) {

    // ---- Operator display ---------------------------------------------------
    if (!frame.empty()) {
        // Crop to configured dimensions before drawing overlays so that
        // overlay anchor points (e.g. bottom of frame) use the cropped size
        int cropW = std::min(frame.cols, cfg_.width);
        int cropH = std::min(frame.rows, cfg_.height);
        cv::Mat canvas = frame(cv::Rect(0, 0, cropW, cropH)).clone();

        DrawCameraElements(canvas);
        DrawMarkerOverlays(canvas, markers, kb.activeTagId);
        DrawTelemetryBar(canvas, markers, touch, kb);

        cv::imshow(WIN_OPERATOR, canvas);
    }

    // ---- Telemetry panel ----------------------------------------------------
    PopulateTelemetryPanel(markers, touch, kb);
    cv::imshow(WIN_TELEMETRY, matTelemetry_);
}

int DisplayHandler::PollKey() {
    // Must be called every iteration — drives the event loop for ALL windows
    return cv::pollKey();
}


// =============================================================================
// Telemetry panel — public
// =============================================================================

void DisplayHandler::ClearTelemetry() {
    matTelemetry_.setTo(Colors::Black);
}

void DisplayHandler::AddHeadingCell(const std::string& text,
                                     const std::string& cellRef,
                                     int colSpan, int rowSpan,
                                     const std::string& align,
                                     float fontSize,
                                     const cv::Scalar& fillColor,
                                     const cv::Scalar& textColor) {
    DrawTelCell(text, cellRef, colSpan, rowSpan, align, fontSize,
                textColor, fillColor, CellStyle::HEADING);
}

void DisplayHandler::AddBodyCell(const std::string& text,
                                  const std::string& cellRef,
                                  int colSpan, int rowSpan,
                                  const std::string& align,
                                  float fontSize,
                                  const cv::Scalar& fillColor,
                                  const cv::Scalar& textColor) {
    DrawTelCell(text, cellRef, colSpan, rowSpan, align, fontSize,
                textColor, fillColor, CellStyle::BODY);
}

void DisplayHandler::AddBorder(const std::string& cellRef,
                                int colSpan, int rowSpan,
                                const cv::Scalar& color,
                                int thickness) {
    cv::Rect r = CellRect(cellRef, colSpan, rowSpan);
    cv::rectangle(matTelemetry_, r, color, thickness);
}


// =============================================================================
// Telemetry panel — private helpers
// =============================================================================

void DisplayHandler::PopulateTelemetryPanel(const std::vector<DetectedMarker>& markers,
                                              const TouchState&                  touch,
                                              const KeyboardState&               kb) {
    ClearTelemetry();

    // ---- Marker count -------------------------------------------------------
    AddHeadingCell("Markers",                         "A1", 9, 1, "center", 0.6f);
    AddBodyCell(std::to_string(markers.size()),        "A2", 1, 1, "center", 0.5f);
    AddBorder("A1", 3, 2, Colors::GraMd, 1);

    // // ---- Active tag ---------------------------------------------------------
    // AddHeadingCell("Active Tag",                       "D1", 3, 1, "center", 0.4f);
    // AddBodyCell(kb.activeTagId > 0
    //                 ? "ID " + std::to_string(kb.activeTagId)
    //                 : "--",                             "D2", 3, 1, "center", 0.45f);
    // AddBorder("D1", 3, 2, Colors::GraMd, 1);

    // // ---- Touch state --------------------------------------------------------
    // AddHeadingCell("Touch",                            "G1", 3, 1, "center", 0.4f);
    // AddBodyCell(touch.isTouched ? "YES" : "no",        "G2", 3, 1, "center", 0.45f);
    // AddBorder("G1", 3, 2, Colors::GraMd, 1);

    // Add more telemetry cells here as task requirements are defined
}

void DisplayHandler::DrawTelCell(const std::string& text,
                                  const std::string& cellRef,
                                  int colSpan, int rowSpan,
                                  const std::string& align,
                                  float fontSize,
                                  const cv::Scalar& textColor,
                                  const cv::Scalar& fillColor,
                                  CellStyle style) {
    cv::Rect r = CellRect(cellRef, colSpan, rowSpan);

    // Fill background
    cv::rectangle(matTelemetry_, r, fillColor, cv::FILLED);

    // Thin separator line — same as the reference's subtle white grid line
    cv::rectangle(matTelemetry_, r, Colors::GraDk, 1);

    if (text.empty()) return;

    // Choose font based on style
    int fontFace = (style == CellStyle::HEADING)
        ? cv::FONT_HERSHEY_DUPLEX
        : cv::FONT_HERSHEY_SIMPLEX;

    int baseLine = 0;
    cv::Size textSz = cv::getTextSize(text, fontFace, fontSize, 1, &baseLine);

    // Vertical centre of the cell
    int textY = r.y + (r.height + textSz.height) / 2;

    // Horizontal position based on alignment
    int textX = 0;
    if (align == "center") {
        textX = r.x + (r.width - textSz.width) / 2;
    } else if (align == "right") {
        textX = r.x + r.width - textSz.width - 4;
    } else {
        // left (default)
        textX = r.x + 4;
    }

    cv::putText(matTelemetry_, text, cv::Point(textX, textY),
                fontFace, fontSize, textColor, 1, cv::LINE_AA);
}

cv::Point2i DisplayHandler::ParseCellRef(const std::string& ref) const {
    int col      = 0;
    int numStart = 0;

    // Determine column index from leading letter(s)
    if (ref.size() > 1 && std::isalpha(static_cast<unsigned char>(ref[1]))) {
        // Two-letter column: AA = 26, AB = 27, ... AX = 49
        col      = 26 + (ref[1] - 'A');
        numStart = 2;
    } else {
        // Single-letter column: A = 0, B = 1, ... Z = 25
        col      = ref[0] - 'A';
        numStart = 1;
    }

    int row = std::stoi(ref.substr(numStart)) - 1;   // 1-based → 0-based
    return cv::Point2i(col, row);
}

cv::Rect DisplayHandler::CellRect(const std::string& ref, int colSpan, int rowSpan) const {
    cv::Point2i pos = ParseCellRef(ref);
    return cv::Rect(pos.x * cellW_,
                    pos.y * cellH_,
                    colSpan * cellW_,
                    rowSpan * cellH_);
}


// =============================================================================
// Operator display — private helpers
// =============================================================================

void DisplayHandler::DrawCameraElements(cv::Mat& frame) {
    // Horizontal line at the camera principal point Y
    cv::line(frame,
             cv::Point(0,               principalPoint_.y),
             cv::Point(frame.cols - 1,  principalPoint_.y),
             cv::Scalar(0, 200, 200), 1);

    // Vertical line at the camera principal point X
    cv::line(frame,
             cv::Point(principalPoint_.x, 0),
             cv::Point(principalPoint_.x, frame.rows - 1),
             cv::Scalar(0, 200, 200), 1);
}

void DisplayHandler::DrawMarkerOverlays(cv::Mat&                            frame,
                                         const std::vector<DetectedMarker>& markers,
                                         int                                activeTagId) {
    for (const auto& m : markers) {

        if (activeTagId > 0 && m.id == activeTagId) {
            // Active tag: green outline square + ID label
            std::vector<cv::Point> corners(4);
            for (int k = 0; k < 4; k++)
                corners[k] = cv::Point(static_cast<int>(m.cornersPx[k].x),
                                       static_cast<int>(m.cornersPx[k].y));
            cv::polylines(frame, corners, true, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame,
                        "ID " + std::to_string(m.id),
                        m.centerPx + cv::Point2i(10, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 0), 2);

        } else {
            // Non-active markers: small dot and ID text
            cv::circle(frame, m.centerPx, 6, cv::Scalar(0, 200, 0), cv::FILLED);
            cv::putText(frame,
                        "ID " + std::to_string(m.id),
                        m.centerPx + cv::Point2i(10, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 200, 0), 2);
        }
    }
}

void DisplayHandler::DrawTelemetryBar(cv::Mat&                            frame,
                                       const std::vector<DetectedMarker>& markers,
                                       const TouchState&                   touch,
                                       const KeyboardState&                kb) {
    const int   x    = 10;
    const float font = 0.5f;
    const auto  col  = cv::Scalar(210, 210, 210);

    // Bottom line — marker count and touch state
    std::string touchStr = touch.isTouched
        ? ("TOUCH (" + std::to_string(touch.position.x) + ", "
                     + std::to_string(touch.position.y) + ")")
        : "no touch";
    std::string activeStr = (kb.activeTagId > 0)
        ? ("  |  Active tag: ID " + std::to_string(kb.activeTagId))
        : "";

    cv::putText(frame,
                "Markers: " + std::to_string(markers.size()) + "  |  " + touchStr + activeStr,
                cv::Point2i(x, frame.rows - 12),
                cv::FONT_HERSHEY_SIMPLEX, font, col, 1);

    // Second line from bottom — shows the command currently being typed
    if (!kb.inputBuffer.empty()) {
        cv::putText(frame,
                    "Cmd> " + kb.inputBuffer,
                    cv::Point2i(x, frame.rows - 32),
                    cv::FONT_HERSHEY_SIMPLEX, font, cv::Scalar(100, 220, 255), 1);
    }
}
