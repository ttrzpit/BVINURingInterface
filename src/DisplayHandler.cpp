#include "DisplayHandler.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
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

DisplayHandler::DisplayHandler(const DisplayConfig &cfg,
                               cv::Point2i principalPoint,
                               const TelemetryConfig &telCfg,
                               const ControllerPanelConfig &controllerCfg)
    : cfg_(cfg), telCfg_(telCfg), controllerCfg_(controllerCfg), principalPoint_(principalPoint), cellW_(telCfg.width / telCfg.cols), cellH_(telCfg.height / telCfg.rows), matTelemetry_(telCfg.height, telCfg.width, CV_8UC3, Colors::Black), controllerCellW_(controllerCfg.width / controllerCfg.cols), controllerCellH_(controllerCfg.height / controllerCfg.rows), matController_(controllerCfg.height, controllerCfg.width, CV_8UC3, Colors::Black) {
    // ---- Operator display window --------------------------------------------
    cv::namedWindow(WIN_OPERATOR, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN_OPERATOR, cfg_.width, cfg_.height);
    cv::moveWindow(WIN_OPERATOR, cfg_.xPos, cfg_.yPos);

    // ---- Telemetry window ---------------------------------------------------
    cv::namedWindow(WIN_TELEMETRY, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN_TELEMETRY, telCfg_.width, telCfg_.height);
    cv::moveWindow(WIN_TELEMETRY, telCfg_.xPos, telCfg_.yPos);

    // ---- Controller window --------------------------------------------------
    // WINDOW_AUTOSIZE makes the window conform exactly to the image size, avoiding
    // the gray-bar pillarboxing that occurs when the WM enforces a minimum window
    // width larger than the controller panel's configured width.
    cv::namedWindow(WIN_CONTROLLER, cv::WINDOW_AUTOSIZE);
    cv::moveWindow(WIN_CONTROLLER, controllerCfg_.xPos, controllerCfg_.yPos);

    std::cout << "DisplayHandler: Operator window " << cfg_.width << "x" << cfg_.height
              << " at (" << cfg_.xPos << ", " << cfg_.yPos << ")\n";
    std::cout << "DisplayHandler: Telemetry window " << telCfg_.width << "x" << telCfg_.height
              << " — " << telCfg_.cols << "x" << telCfg_.rows << " cells ("
              << cellW_ << "x" << cellH_ << " px)"
              << " at (" << telCfg_.xPos << ", " << telCfg_.yPos << ")\n";
    std::cout << "DisplayHandler: Controller window " << controllerCfg_.width << "x" << controllerCfg_.height
              << " — " << controllerCfg_.cols << "x" << controllerCfg_.rows << " cells ("
              << controllerCellW_ << "x" << controllerCellH_ << " px)"
              << " at (" << controllerCfg_.xPos << ", " << controllerCfg_.yPos << ")\n";
}

// =============================================================================
// Operator display — public
// =============================================================================

void DisplayHandler::Update(const cv::Mat &frame,
                            const std::vector<DetectedMarker> &markers,
                            const TouchState &touch, const KeyboardState &kb,
                            const SerialState &serial) {
    // ---- Loop frequency measurement -----------------------------------------
    // Count every call; once a full second has elapsed, latch the Hz value and
    // reset. Using cv::getTickCount so there are no extra includes needed.
    freqFrameCount_++;
    double now = cv::getTickCount() / cv::getTickFrequency();
    double elapsed = now - freqWindowStart_;
    if (elapsed >= 1.0) {
        measuredFreqHz_ = static_cast<float>(freqFrameCount_ / elapsed);
        freqFrameCount_ = 0;
        freqWindowStart_ = now;
    }

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

    // ---- Telemetry + Controller panels (throttled to 10 Hz) -----------------
    // Populating these panels involves many draw calls per frame. Limiting to
    // 10 Hz saves ~3-4 ms per iteration without any visible lag for telemetry.
    if (now - lastPanelUpdateTime_ >= 0.1) {
        PopulateTelemetryPanel(markers, touch, kb, serial);
        cv::imshow(WIN_TELEMETRY, matTelemetry_);

        PopulateControllerPanel(markers, touch, kb);
        cv::imshow(WIN_CONTROLLER, matController_);

        lastPanelUpdateTime_ = now;
    }
}

int DisplayHandler::PollKey() {
    // Must be called every iteration — drives the event loop for ALL windows
    return cv::pollKey();
}

// =============================================================================
// Telemetry panel — public
// =============================================================================

void DisplayHandler::ClearTelemetry() { matTelemetry_.setTo(Colors::Black); }

void DisplayHandler::AddHeadingCell(const std::string &text,
                                    const std::string &cellRef, int colSpan,
                                    int rowSpan, const std::string &align,
                                    float fontSize, const cv::Scalar &fillColor,
                                    const cv::Scalar &textColor) {
    DrawTelCell(text, cellRef, colSpan, rowSpan, align, fontSize, textColor,
                fillColor, CellStyle::HEADING);
}

void DisplayHandler::AddSubheadingCell(const std::string &text,
                                       const std::string &cellRef, int colSpan,
                                       int rowSpan, const std::string &align,
                                       float fontSize,
                                       const cv::Scalar &fillColor,
                                       const cv::Scalar &textColor) {
    DrawTelCell(text, cellRef, colSpan, rowSpan, align, fontSize, textColor,
                fillColor, CellStyle::HEADING);
}

void DisplayHandler::AddBodyCell(const std::string &text,
                                 const std::string &cellRef, int colSpan,
                                 int rowSpan, const std::string &align,
                                 float fontSize, const cv::Scalar &fillColor,
                                 const cv::Scalar &textColor) {
    DrawTelCell(text, cellRef, colSpan, rowSpan, align, fontSize, textColor,
                fillColor, CellStyle::BODY);
}

void DisplayHandler::AddBorder(const std::string &cellRef, int colSpan,
                               int rowSpan, const cv::Scalar &color,
                               int thickness) {
    cv::Rect r = CellRect(cellRef, colSpan, rowSpan);
    cv::rectangle(matTelemetry_, r, color, thickness);
}

// =============================================================================
// Controller panel — public
// =============================================================================

void DisplayHandler::ClearController() { matController_.setTo(Colors::Black); }

void DisplayHandler::AddControllerHeadingCell(const std::string &text, const std::string &cellRef,
                                              int colSpan, int rowSpan, const std::string &align,
                                              float fontSize, const cv::Scalar &fillColor,
                                              const cv::Scalar &textColor) {
    DrawControllerCell(text, cellRef, colSpan, rowSpan, align, fontSize, textColor, fillColor, CellStyle::HEADING);
}

void DisplayHandler::AddControllerSubheadingCell(const std::string &text, const std::string &cellRef,
                                                 int colSpan, int rowSpan, const std::string &align,
                                                 float fontSize, const cv::Scalar &fillColor,
                                                 const cv::Scalar &textColor) {
    DrawControllerCell(text, cellRef, colSpan, rowSpan, align, fontSize, textColor, fillColor, CellStyle::SUBHEADING);
}

void DisplayHandler::AddControllerBodyCell(const std::string &text, const std::string &cellRef,
                                           int colSpan, int rowSpan, const std::string &align,
                                           float fontSize, const cv::Scalar &fillColor,
                                           const cv::Scalar &textColor) {
    DrawControllerCell(text, cellRef, colSpan, rowSpan, align, fontSize, textColor, fillColor, CellStyle::BODY);
}

void DisplayHandler::AddControllerBorder(const std::string &cellRef, int colSpan, int rowSpan,
                                         const cv::Scalar &color, int thickness) {
    cv::Rect r = ControllerCellRect(cellRef, colSpan, rowSpan);
    cv::rectangle(matController_, r, color, thickness);
}

// =============================================================================
// Telemetry panel — private helpers
// =============================================================================

void DisplayHandler::PopulateTelemetryPanel(
    const std::vector<DetectedMarker> &markers, const TouchState &touch,
    const KeyboardState &kb, const SerialState &serial) {
    ClearTelemetry();

    float headerFontSize = 0.55f;
    float bodyFontSize = 0.45f;

    // ---- System Status -------------------------------------------------------
    AddHeadingCell("System Status", "A1", 5, 1, "center", headerFontSize);
    AddSubheadingCell("Serial", "A2", 2, 1, "center", bodyFontSize);
    AddSubheadingCell("Camera", "A3", 2, 1, "center", bodyFontSize);
    AddSubheadingCell("State", "A4", 2, 1, "center", bodyFontSize);
    AddSubheadingCell("Teensy", "A5", 2, 1, "center", bodyFontSize);
    AddSubheadingCell("User ID", "A6", 2, 1, "center", bodyFontSize);

    AddBodyCell(std::to_string(static_cast<int>(serial.txFrequencyHz)) + " Hz",
                "C2", 3, 1, "center", bodyFontSize,
                serial.txFrequencyHz >= 150.0f ? Colors::GreBk : Colors::RedBk);
    AddBodyCell(std::to_string(static_cast<int>(measuredFreqHz_)) + " Hz", "C3", 3, 1, "center", bodyFontSize,
                measuredFreqHz_ >= 60.0f ? Colors::GreBk : Colors::RedBk);
    {
        std::string stateStr;
        cv::Scalar stateFill;
        switch (kb.systemState) {
            case SystemState::CALIBRATING:
                stateStr = "CALIBRATING";
                stateFill = Colors::YelBk;
                break;
            case SystemState::CAL3:
                stateStr = "CAL3";
                stateFill = Colors::OraBk;
                break;
            case SystemState::FITTS:
                stateStr = "FITTS";
                stateFill = Colors::GreBk;
                break;
            default:
                stateStr = "IDLE";
                stateFill = Colors::RedBk;
                break;
        }
        AddBodyCell(stateStr, "C4", 3, 1, "center", bodyFontSize, stateFill);
    }
    AddBodyCell("[TEENSY]", "C5", 3, 1, "center", bodyFontSize);
    AddBodyCell("[NURING]", "C6", 3, 1, "center", bodyFontSize);
    AddBorder("A1", 5, 8, Colors::GraMd, 2);

    // ---- Marker visibility -------------------------------------------------------
    AddHeadingCell("Marker Visibility", "F1", 8, 1, "center", headerFontSize);
    AddHeadingCell(std::to_string(markers.size()), "N1", 1, 1, "center",
                   headerFontSize);
    // One cell per marker ID (1–45), 9 per row across columns A–I, rows 2–6.
    // Colour rules:
    //   active marker   → dark green background, white text
    //   detected        → default background,    white text
    //   not detected    → default background,    gray text
    auto isDetected = [&markers](int id) {
        for (const auto &m : markers)
            if (m.id == id) return true;
        return false;
    };

    for (int id = 1; id <= 45; id++) {
        std::string cellRef = std::string(1, 'F' + (id - 1) % 9) + std::to_string(2 + (id - 1) / 9);

        bool detected = isDetected(id);
        bool active = (id == kb.activeTagId);

        cv::Scalar fill = active ? Colors::GreBk : Colors::GraBk;
        cv::Scalar text = detected ? Colors::White : Colors::GraMd;

        AddBodyCell(std::to_string(id), cellRef, 1, 1, "center", bodyFontSize, fill, text);
    }
    AddBorder("F1", 9, 6, Colors::GraMd, 2);

    // ---- Keyboard inputs -------------------
    AddSubheadingCell("Input", "F7", 2, 1, "center", bodyFontSize);
    AddSubheadingCell("Output", "F8", 2, 1, "center", bodyFontSize);
    AddBodyCell(kb.inputBuffer, "H7", 7, 1, "center", bodyFontSize);
    AddBodyCell(kb.outputBuffer, "H8", 7, 1, "center", bodyFontSize);
    AddBorder("F7", 9, 2, Colors::GraMd, 2);

    // ---- Active Marker Telemetry (pos_camera from solvePnP) -------------------
    // Find the active marker in this frame's detection results
    const DetectedMarker *activeMarker = nullptr;
    if (kb.activeTagId > 0) {
        for (const auto &m : markers) {
            if (m.id == kb.activeTagId) {
                activeMarker = &m;
                break;
            }
        }
    }

    // Format a float to one decimal place (e.g. "12.3")
    auto fmtMm = [](float v) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << v;
        return ss.str();
    };

    // Format a float to two decimal places (e.g., "12.34")
    auto fmt2Dec = [](float v) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << v;
        return ss.str();
    };

    AddHeadingCell("Telemetry", "O1", 4, 1, "center", headerFontSize);
    AddSubheadingCell("Active:", "O2", 3, 1, "center", bodyFontSize);
    AddSubheadingCell("x", "O3", 1, 1, "center", bodyFontSize);
    AddSubheadingCell("y", "O4", 1, 1, "center", bodyFontSize);
    AddSubheadingCell("z", "O5", 1, 1, "center", bodyFontSize);
    AddSubheadingCell("R2", "O6", 1, 1, "center", bodyFontSize);
    AddSubheadingCell("R3", "O7", 1, 1, "center", bodyFontSize);
    AddSubheadingCell("Roll", "O8", 1, 1, "center", bodyFontSize);

    // ID box: show the active tag ID whenever one is set, even if not currently
    // detected
    AddSubheadingCell(kb.activeTagId > 0 ? std::to_string(kb.activeTagId) : "--",
                      "R2", 1, 1, "center", bodyFontSize);

    if (activeMarker) {
        const float x = activeMarker->positionMm.x;
        const float y = activeMarker->positionMm.y;
        const float z = activeMarker->positionMm.z;

        // pos_camera = solvePnP translation vector [mm]
        AddBodyCell(fmtMm(x), "P3", 3, 1, "center", bodyFontSize);
        AddBodyCell(fmtMm(y), "P4", 3, 1, "center", bodyFontSize);
        AddBodyCell(fmtMm(z), "P5", 3, 1, "center", bodyFontSize);
        AddBodyCell(fmtMm(std::sqrt(x * x + y * y)), "P6", 3, 1, "center", bodyFontSize);          // R2 = 2D Euclidean distance in XY plane [mm]
        AddBodyCell(fmtMm(std::sqrt(x * x + y * y + z * z)), "P7", 3, 1, "center", bodyFontSize);  // R3 = 3D Euclidean distance [mm]
        AddBodyCell(fmtMm(activeMarker->rotationDeg), "P8", 3, 1, "center", bodyFontSize);         // roll_current = rvec[1] * RAD2DEG from solvePnP
    } else {
        // Active tag not in frame this iteration
        AddBodyCell("--", "P3", 3, 1, "center", bodyFontSize);
        AddBodyCell("--", "P4", 3, 1, "center", bodyFontSize);
        AddBodyCell("--", "P5", 3, 1, "center", bodyFontSize);
        AddBodyCell("--", "P6", 3, 1, "center", bodyFontSize);
        AddBodyCell("--", "P7", 3, 1, "center", bodyFontSize);
        AddBodyCell("--", "P8", 3, 1, "center", bodyFontSize);
    }
    AddBorder("O1", 4, 8, Colors::GraMd, 2);

    // ---- Serial connection information ----------------------------------------------
    AddHeadingCell("Serial Communication", "S1", 29, 1, "center", headerFontSize);
    AddSubheadingCell("Status", "S2", 3, 1, "center", bodyFontSize);
    AddSubheadingCell("Teensy State", "V2", 3, 1, "center", bodyFontSize);
    AddSubheadingCell("Out", "Y2", 1, 1, "center", bodyFontSize);

    AddSubheadingCell("PwmA", "Z2", 2, 1, "center", bodyFontSize);
    AddSubheadingCell("PwmB", "AB2", 2, 1, "center", bodyFontSize);
    AddSubheadingCell("PwmC", "AD2", 2, 1, "center", bodyFontSize);

    AddSubheadingCell("In", "AF2", 1, 1, "center", bodyFontSize);
    AddSubheadingCell("EncA", "AG2", 3, 1, "center", bodyFontSize);
    AddSubheadingCell("EncB", "AJ2", 3, 1, "center", bodyFontSize);
    AddSubheadingCell("EncC", "AM2", 3, 1, "center", bodyFontSize);
    AddSubheadingCell("CurA", "AP2", 2, 1, "center", bodyFontSize);
    AddSubheadingCell("CurB", "AR2", 2, 1, "center", bodyFontSize);
    AddSubheadingCell("CurC", "AT2", 2, 1, "center", bodyFontSize);

    // Connection status — coloured background makes it easy to spot at a glance
    AddBodyCell(serial.isConnected ? "Connected" : "No Teensy",
                "S3", 3, 1, "center", bodyFontSize,
                serial.isConnected ? Colors::GreBk : Colors::RedBk);

    // Teensy state byte echoed back (single ASCII char: 'I'=idle, 'D'=drive, etc.)
    AddBodyCell(serial.hasRx ? std::string(1, static_cast<char>(serial.lastRx.state)) : "--",
                "V3", 3, 1, "center", bodyFontSize);

    // Outgoing packet index (0–99 rolling)
    AddBodyCell(std::to_string(serial.lastTx.packet_index), "Y3", 1, 1, "center", bodyFontSize);

    // Commanded PWM values (0 = full power, 2047 = no power)
    AddBodyCell(std::to_string(serial.lastTx.pwm_A), "Z3", 2, 1, "center", bodyFontSize);
    AddBodyCell(std::to_string(serial.lastTx.pwm_B), "AB3", 2, 1, "center", bodyFontSize);
    AddBodyCell(std::to_string(serial.lastTx.pwm_C), "AD3", 2, 1, "center", bodyFontSize);

    // Incoming data — only shown once at least one valid RX packet has arrived
    const std::string na = "--";
    AddBodyCell(serial.hasRx ? std::to_string(serial.lastRx.packet_index) : na, "AF3", 1, 1, "center", bodyFontSize);
    AddBodyCell(serial.hasRx ? std::to_string(serial.lastRx.encoder_count_A) : na, "AG3", 3, 1, "center", bodyFontSize);
    AddBodyCell(serial.hasRx ? std::to_string(serial.lastRx.encoder_count_B) : na, "AJ3", 3, 1, "center", bodyFontSize);
    AddBodyCell(serial.hasRx ? std::to_string(serial.lastRx.encoder_count_C) : na, "AM3", 3, 1, "center", bodyFontSize);
    AddBodyCell(serial.hasRx ? fmt2Dec(serial.lastRx.current_raw_A * 0.01) : na, "AP3", 2, 1, "center", bodyFontSize);
    AddBodyCell(serial.hasRx ? fmt2Dec(serial.lastRx.current_raw_B * 0.01) : na, "AR3", 2, 1, "center", bodyFontSize);
    AddBodyCell(serial.hasRx ? fmt2Dec(serial.lastRx.current_raw_C * 0.01) : na, "AT3", 2, 1, "center", bodyFontSize);
    AddBorder("S1", 29, 3, Colors::GraMd, 2);

    // ---- Active tag ---------------------------------------------------------
    // AddHeadingCell("Active Tag", "P1", 3, 1, "center", 0.4f);
    // AddBodyCell(kb.activeTagId > 0 ? "ID " + std::to_string(kb.activeTagId) : "--",
    //             "D2", 3, 1, "center", 0.45f);
    // AddBorder("D1", 3, 2, Colors::GraMd, 1);

    // // ---- Touch state
    // --------------------------------------------------------
    // AddHeadingCell("Touch",                            "G1", 3, 1, "center",
    // 0.4f); AddBodyCell(touch.isTouched ? "YES" : "no",        "G2", 3, 1,
    // "center", 0.45f); AddBorder("G1", 3, 2, Colors::GraMd, 1);

    // Add more telemetry cells here as task requirements are defined
}

void DisplayHandler::DrawTelCell(const std::string &text,
                                 const std::string &cellRef, int colSpan,
                                 int rowSpan, const std::string &align,
                                 float fontSize, const cv::Scalar &textColor,
                                 const cv::Scalar &fillColor, CellStyle style) {
    cv::Rect r = CellRect(cellRef, colSpan, rowSpan);

    // Fill background
    cv::rectangle(matTelemetry_, r, fillColor, cv::FILLED);

    // Thin separator line — same as the reference's subtle white grid line
    cv::rectangle(matTelemetry_, r, Colors::GraDk, 1);

    if (text.empty())
        return;

    // Choose font based on style
    int fontFace = (style == CellStyle::HEADING) ? cv::FONT_HERSHEY_DUPLEX
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

    cv::putText(matTelemetry_, text, cv::Point(textX, textY), fontFace, fontSize,
                textColor, 1, cv::LINE_AA);
}

cv::Point2i DisplayHandler::ParseCellRef(const std::string &ref) const {
    int col = 0;
    int numStart = 0;

    // Determine column index from leading letter(s)
    if (ref.size() > 1 && std::isalpha(static_cast<unsigned char>(ref[1]))) {
        // Two-letter column: AA = 26, AB = 27, ... AX = 49
        col = 26 + (ref[1] - 'A');
        numStart = 2;
    } else {
        // Single-letter column: A = 0, B = 1, ... Z = 25
        col = ref[0] - 'A';
        numStart = 1;
    }

    int row = std::stoi(ref.substr(numStart)) - 1;  // 1-based → 0-based
    return cv::Point2i(col, row);
}

cv::Rect DisplayHandler::CellRect(const std::string &ref, int colSpan,
                                  int rowSpan) const {
    cv::Point2i pos = ParseCellRef(ref);
    return cv::Rect(pos.x * cellW_, pos.y * cellH_, colSpan * cellW_,
                    rowSpan * cellH_);
}

// =============================================================================
// Controller panel — private helpers
// =============================================================================

void DisplayHandler::PopulateControllerPanel(const std::vector<DetectedMarker> &markers,
                                             const TouchState &touch, const KeyboardState &kb) {
    ClearController();
    // Populate controller telemetry cells here as the controller is implemented

    float headerFontSize = 0.55f;
    float bodyFontSize = 0.45f;

    cv::Point2i center = cv::Point2i(149, 895);
    float radius = 135.0f;

    AddControllerHeadingCell("Controller Panel", "A1", 10, 1, "center", headerFontSize);

    AddControllerHeadingCell("Virtual Fingertip", "A23", 10, 1, "center", headerFontSize);
    AddControllerBorder("A23", 10, 11, Colors::GraMd, 2);

    cv::circle(matController_, center, radius, Colors::GraDk, 2);
    cv::circle(matController_, center, 4, Colors::GraDk, -1);
}

void DisplayHandler::DrawControllerCell(const std::string &text, const std::string &cellRef,
                                        int colSpan, int rowSpan, const std::string &align,
                                        float fontSize, const cv::Scalar &textColor,
                                        const cv::Scalar &fillColor, CellStyle style) {
    cv::Rect r = ControllerCellRect(cellRef, colSpan, rowSpan);

    cv::rectangle(matController_, r, fillColor, cv::FILLED);
    cv::rectangle(matController_, r, Colors::GraDk, 1);

    if (text.empty()) return;

    int fontFace = (style == CellStyle::BODY) ? cv::FONT_HERSHEY_SIMPLEX : cv::FONT_HERSHEY_DUPLEX;

    int baseLine = 0;
    cv::Size textSz = cv::getTextSize(text, fontFace, fontSize, 1, &baseLine);

    int textY = r.y + (r.height + textSz.height) / 2;
    int textX = 0;
    if (align == "center") {
        textX = r.x + (r.width - textSz.width) / 2;
    } else if (align == "right") {
        textX = r.x + r.width - textSz.width - 4;
    } else {
        textX = r.x + 4;
    }

    cv::putText(matController_, text, cv::Point(textX, textY), fontFace, fontSize,
                textColor, 1, cv::LINE_AA);
}

cv::Rect DisplayHandler::ControllerCellRect(const std::string &ref, int colSpan, int rowSpan) const {
    cv::Point2i pos = ParseCellRef(ref);
    return cv::Rect(pos.x * controllerCellW_, pos.y * controllerCellH_,
                    colSpan * controllerCellW_, rowSpan * controllerCellH_);
}

// =============================================================================
// Operator display — private helpers
// =============================================================================

void DisplayHandler::DrawCameraElements(cv::Mat &frame) {
    // Horizontal line at the camera principal point Y
    cv::line(frame, cv::Point(0, principalPoint_.y),
             cv::Point(frame.cols - 1, principalPoint_.y),
             cv::Scalar(0, 200, 200), 1);

    // Vertical line at the camera principal point X
    cv::line(frame, cv::Point(principalPoint_.x, 0),
             cv::Point(principalPoint_.x, frame.rows - 1),
             cv::Scalar(0, 200, 200), 1);
}

void DisplayHandler::DrawMarkerOverlays(
    cv::Mat &frame, const std::vector<DetectedMarker> &markers,
    int activeTagId) {
    for (const auto &m : markers) {
        if (activeTagId > 0 && m.id == activeTagId) {
            // Active tag: green outline square + ID label
            std::vector<cv::Point> corners(4);
            for (int k = 0; k < 4; k++)
                corners[k] = cv::Point(static_cast<int>(m.cornersPx[k].x),
                                       static_cast<int>(m.cornersPx[k].y));
            cv::polylines(frame, corners, true, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "ID " + std::to_string(m.id),
                        m.centerPx + cv::Point2i(10, -10), cv::FONT_HERSHEY_SIMPLEX,
                        0.6, cv::Scalar(0, 255, 0), 2);

        } else {
            // Non-active markers: small dot and ID text
            cv::circle(frame, m.centerPx, 6, cv::Scalar(0, 200, 0), cv::FILLED);
            cv::putText(frame, "ID " + std::to_string(m.id),
                        m.centerPx + cv::Point2i(10, -10), cv::FONT_HERSHEY_SIMPLEX,
                        0.6, cv::Scalar(0, 200, 0), 2);
        }
    }
}

void DisplayHandler::DrawTelemetryBar(
    cv::Mat &frame, const std::vector<DetectedMarker> &markers,
    const TouchState &touch, const KeyboardState &kb) {
    const int x = 10;
    const float font = 0.5f;
    const auto col = cv::Scalar(210, 210, 210);

    // Bottom line — marker count and touch state
    std::string touchStr = touch.isTouched
                               ? ("TOUCH (" + std::to_string(touch.position.x) +
                                  ", " + std::to_string(touch.position.y) + ")")
                               : "no touch";
    std::string activeStr =
        (kb.activeTagId > 0)
            ? ("  |  Active tag: ID " + std::to_string(kb.activeTagId))
            : "";

    // cv::putText(frame,
    //             "Markers: " + std::to_string(markers.size()) + "  |  " +
    //                 touchStr + activeStr,
    //             cv::Point2i(x, frame.rows - 12), cv::FONT_HERSHEY_SIMPLEX, font,
    //             col, 1);

    // // Second line from bottom — shows the command currently being typed
    // if (!kb.inputBuffer.empty()) {
    //     cv::putText(frame, "Cmd> " + kb.inputBuffer,
    //                 cv::Point2i(x, frame.rows - 32), cv::FONT_HERSHEY_SIMPLEX, font,
    //                 cv::Scalar(100, 220, 255), 1);
    // }
}
