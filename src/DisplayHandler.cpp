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
//   WIN_OPERATOR  - camera feed + overlays
//   WIN_TELEMETRY - 50×6 cell grid panel
//
// Telemetry grid coordinate system:
//   Columns: A–Z (0–25), AA–AX (26–49)  (50 total)
//   Rows:    1–6 (0-indexed internally as 0–5)
//   Origin:  top-left corner of the telemetry canvas
// =============================================================================

namespace {

/** @brief Format a raw key code for the [LAST_INPUT] telemetry cell. */
std::string FormatLastInputKey( int key ) {
    switch ( key ) {
        case -1:
            return "--";
        case 27:
            return "ESC";
        case 32:
            return "SPACE";
        case 96:
            return "`";
        case 13:
        case 10:
            return "ENTER";
        case 8:
        case 127:
            return "BKSP";
        case 174:
            return ".";    // numpad decimal
        case 171:
            return "+";    // numpad add
        case 173:
            return "-";    // numpad subtract
        default:
            if ( key >= 176 && key <= 185 ) return std::string( 1, static_cast<char>( '0' + ( key - 176 ) ) );    // numpad digits
            if ( key >= 33 && key <= 126 ) return std::string( 1, static_cast<char>( key ) );
            return std::to_string( key );
    }
}

}    // namespace

// =============================================================================
// Construction
// =============================================================================

DisplayHandler::DisplayHandler( const DisplayConfig &cfg,
                                cv::Point2i principalPoint,
                                const TelemetryConfig &telCfg,
                                const ControllerPanelConfig &controllerCfg )
    : cfg_( cfg ), telCfg_( telCfg ), controllerCfg_( controllerCfg ), principalPoint_( principalPoint ), cellW_( telCfg.width / telCfg.cols ), cellH_( telCfg.height / telCfg.rows ), matTelemetry_( telCfg.height, telCfg.width, CV_8UC3, Colors::Black ), controllerCellW_( controllerCfg.width / controllerCfg.cols ), controllerCellH_( controllerCfg.height / controllerCfg.rows ), matController_( controllerCfg.height, controllerCfg.width, CV_8UC3, Colors::Black ) {
    // ---- Operator display window --------------------------------------------
    cv::namedWindow( WIN_OPERATOR, cv::WINDOW_NORMAL );
    cv::resizeWindow( WIN_OPERATOR, cfg_.width, cfg_.height );
    cv::moveWindow( WIN_OPERATOR, cfg_.xPos, cfg_.yPos );

    // ---- Telemetry window ---------------------------------------------------
    cv::namedWindow( WIN_TELEMETRY, cv::WINDOW_NORMAL );
    cv::resizeWindow( WIN_TELEMETRY, telCfg_.width, telCfg_.height );
    cv::moveWindow( WIN_TELEMETRY, telCfg_.xPos, telCfg_.yPos );

    // ---- Controller window --------------------------------------------------
    // WINDOW_AUTOSIZE makes the window conform exactly to the image size, avoiding
    // the gray-bar pillarboxing that occurs when the WM enforces a minimum window
    // width larger than the controller panel's configured width.
    cv::namedWindow( WIN_CONTROLLER, cv::WINDOW_AUTOSIZE );
    cv::moveWindow( WIN_CONTROLLER, controllerCfg_.xPos, controllerCfg_.yPos );

    std::cout << "DisplayHandler: Operator window " << cfg_.width << "x" << cfg_.height
              << " at (" << cfg_.xPos << ", " << cfg_.yPos << ")\n";
    std::cout << "DisplayHandler: Telemetry window " << telCfg_.width << "x" << telCfg_.height
              << " - " << telCfg_.cols << "x" << telCfg_.rows << " cells ("
              << cellW_ << "x" << cellH_ << " px)"
              << " at (" << telCfg_.xPos << ", " << telCfg_.yPos << ")\n";
    std::cout << "DisplayHandler: Controller window " << controllerCfg_.width << "x" << controllerCfg_.height
              << " - " << controllerCfg_.cols << "x" << controllerCfg_.rows << " cells ("
              << controllerCellW_ << "x" << controllerCellH_ << " px)"
              << " at (" << controllerCfg_.xPos << ", " << controllerCfg_.yPos << ")\n";
}

// =============================================================================
// Operator display - public
// =============================================================================

void DisplayHandler::Update( const cv::Mat &frame,
                             const std::vector<DetectedMarker> &markers,
                             const TouchState &touch, const KeyboardState &kb,
                             const SerialState &serial ) {
    // ---- Loop frequency measurement -----------------------------------------
    // Count every call; once a full second has elapsed, latch the Hz value and
    // reset. Using cv::getTickCount so there are no extra includes needed.
    freqFrameCount_++;
    double now = cv::getTickCount() / cv::getTickFrequency();
    double elapsed = now - freqWindowStart_;
    if ( elapsed >= 1.0 ) {
        measuredFreqHz_ = static_cast<float>( freqFrameCount_ / elapsed );
        freqFrameCount_ = 0;
        freqWindowStart_ = now;
    }

    // ---- Operator display ---------------------------------------------------
    if ( !frame.empty() ) {
        // Crop to configured dimensions before drawing overlays so that
        // overlay anchor points (e.g. bottom of frame) use the cropped size
        int cropW = std::min( frame.cols, cfg_.width );
        int cropH = std::min( frame.rows, cfg_.height );
        cv::Mat canvas = frame( cv::Rect( 0, 0, cropW, cropH ) ).clone();

        DrawCameraElements( canvas );
        DrawMarkerOverlays( canvas, markers, kb.activeTagId );

        if ( targetCircleVisible_ ) {
            // FITTS target - active tag center offset "under" by the Cal3 Y
            // offset (or the default before Cal3 completes). Color is red
            // (cal3 complete) or gray (default position, not yet calibrated).
            cv::circle( canvas, targetCircleCenterPx_, targetCircleRadiusPx_, targetCircleColor_, 1 );
        }

        if ( virtualFingertipVisible_ ) {
            const cv::Point2i &p = virtualFingertipPx_;
            cv::circle( canvas, p, 14, cv::Scalar( 255, 0, 255 ), 2 );    // magenta ring
            cv::circle( canvas, p, 3, cv::Scalar( 255, 0, 255 ), -1 );    // filled centre
            // cv::putText(canvas, "FT", p + cv::Point2i(16, -12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
        }

        if ( touchFingertipVisible_ ) {
            // Frozen fingertip position at the moment of the last touchscreen
            // contact - stays until the next target is selected.
            cv::circle( canvas, touchFingertipPx_, 3, Colors::BluMd, -1 );
        }

        DrawTelemetryBar( canvas, markers, touch, kb );

        cv::imshow( WIN_OPERATOR, canvas );
    }

    // ---- Telemetry + Controller panels (throttled to 10 Hz) -----------------
    // Populating these panels involves many draw calls per frame. Limiting to
    // 10 Hz saves ~3-4 ms per iteration without any visible lag for telemetry.
    if ( now - lastPanelUpdateTime_ >= 0.1 ) {
        PopulateTelemetryPanel( markers, touch, kb, serial );
        cv::imshow( WIN_TELEMETRY, matTelemetry_ );

        PopulateControllerPanel( markers, touch, kb, serial );
        cv::imshow( WIN_CONTROLLER, matController_ );

        lastPanelUpdateTime_ = now;
    }
}

int DisplayHandler::PollKey() {
    // Must be called every iteration - drives the event loop for ALL windows
    return cv::pollKey();
}

// =============================================================================
// Telemetry panel - public
// =============================================================================

void DisplayHandler::ClearTelemetry() { matTelemetry_.setTo( Colors::Black ); }

void DisplayHandler::AddHeadingCell( const std::string &text,
                                     const std::string &cellRef, int colSpan,
                                     int rowSpan, const std::string &align,
                                     float fontSize, const cv::Scalar &fillColor,
                                     const cv::Scalar &textColor ) {
    DrawTelCell( text, cellRef, colSpan, rowSpan, align, fontSize, textColor,
                 fillColor, CellStyle::HEADING );
}

void DisplayHandler::AddSubheadingCell( const std::string &text,
                                        const std::string &cellRef, int colSpan,
                                        int rowSpan, const std::string &align,
                                        float fontSize,
                                        const cv::Scalar &fillColor,
                                        const cv::Scalar &textColor ) {
    DrawTelCell( text, cellRef, colSpan, rowSpan, align, fontSize, textColor,
                 fillColor, CellStyle::HEADING );
}

void DisplayHandler::AddBodyCell( const std::string &text,
                                  const std::string &cellRef, int colSpan,
                                  int rowSpan, const std::string &align,
                                  float fontSize, const cv::Scalar &fillColor,
                                  const cv::Scalar &textColor ) {
    DrawTelCell( text, cellRef, colSpan, rowSpan, align, fontSize, textColor,
                 fillColor, CellStyle::BODY );
}

void DisplayHandler::AddBorder( const std::string &cellRef, int colSpan,
                                int rowSpan, const cv::Scalar &color,
                                int thickness ) {
    cv::Rect r = CellRect( cellRef, colSpan, rowSpan );
    cv::rectangle( matTelemetry_, r, color, thickness );
}

// =============================================================================
// Controller panel - public
// =============================================================================

void DisplayHandler::ClearController() { matController_.setTo( Colors::Black ); }

void DisplayHandler::AddControllerHeadingCell( const std::string &text, const std::string &cellRef,
                                               int colSpan, int rowSpan, const std::string &align,
                                               float fontSize, const cv::Scalar &fillColor,
                                               const cv::Scalar &textColor ) {
    DrawControllerCell( text, cellRef, colSpan, rowSpan, align, fontSize, textColor, fillColor, CellStyle::HEADING );
}

void DisplayHandler::AddControllerSubheadingCell( const std::string &text, const std::string &cellRef,
                                                  int colSpan, int rowSpan, const std::string &align,
                                                  float fontSize, const cv::Scalar &fillColor,
                                                  const cv::Scalar &textColor ) {
    DrawControllerCell( text, cellRef, colSpan, rowSpan, align, fontSize, textColor, fillColor, CellStyle::SUBHEADING );
}

void DisplayHandler::AddControllerBodyCell( const std::string &text, const std::string &cellRef,
                                            int colSpan, int rowSpan, const std::string &align,
                                            float fontSize, const cv::Scalar &fillColor,
                                            const cv::Scalar &textColor ) {
    DrawControllerCell( text, cellRef, colSpan, rowSpan, align, fontSize, textColor, fillColor, CellStyle::BODY );
}

void DisplayHandler::AddControllerBorder( const std::string &cellRef, int colSpan, int rowSpan,
                                          const cv::Scalar &color, int thickness ) {
    cv::Rect r = ControllerCellRect( cellRef, colSpan, rowSpan );
    cv::rectangle( matController_, r, color, thickness );
}

// =============================================================================
// Telemetry panel - private helpers
// =============================================================================

void DisplayHandler::PopulateTelemetryPanel(
    const std::vector<DetectedMarker> &markers, const TouchState &touch,
    const KeyboardState &kb, const SerialState &serial ) {
    ClearTelemetry();

    float headerFontSize = 0.55f;
    float bodyFontSize = 0.45f;

    // ---- System Status -------------------------------------------------------
    // AddHeadingCell("System Status", "A1", 5, 1, "center", headerFontSize);
    // AddSubheadingCell("Serial", "A2", 2, 1, "center", bodyFontSize);
    // AddSubheadingCell("Camera", "A3", 2, 1, "center", bodyFontSize);
    // AddSubheadingCell("State", "A4", 2, 1, "center", bodyFontSize);
    // AddSubheadingCell("Teensy", "A5", 2, 1, "center", bodyFontSize);
    // AddSubheadingCell("User ID", "A6", 2, 1, "center", bodyFontSize);

    // AddBodyCell(std::to_string(static_cast<int>(serial.txFrequencyHz)) + " Hz",
    //             "C2", 3, 1, "center", bodyFontSize,
    //             serial.txFrequencyHz >= 150.0f ? Colors::GreBk : Colors::RedBk);
    // AddBodyCell(std::to_string(static_cast<int>(measuredFreqHz_)) + " Hz", "C3", 3, 1, "center", bodyFontSize,
    //             measuredFreqHz_ >= 60.0f ? Colors::GreBk : Colors::RedBk);
    // {
    //     std::string stateStr;
    //     cv::Scalar stateFill;
    //     switch (kb.systemState) {
    //         case SystemState::CALIBRATING:
    //             stateStr = "CALIBRATING";
    //             stateFill = Colors::YelBk;
    //             break;
    //         case SystemState::CAL3:
    //             stateStr = "CAL3";
    //             stateFill = Colors::OraBk;
    //             break;
    //         case SystemState::FITTS:
    //             stateStr = "FITTS";
    //             stateFill = Colors::GreBk;
    //             break;
    //         default:
    //             stateStr = "IDLE";
    //             stateFill = Colors::RedBk;
    //             break;
    //     }
    //     AddBodyCell(stateStr, "C4", 3, 1, "center", bodyFontSize, stateFill);
    // }
    // {
    //     std::string robotStr;
    //     cv::Scalar robotFill;
    //     switch (serial.robotState) {
    //         case RobotState::IDLE:
    //             robotStr = "IDLE";
    //             robotFill = Colors::GraBk;
    //             break;
    //         case RobotState::READY:
    //             robotStr = "READY";
    //             robotFill = Colors::GreBk;
    //             break;
    //         case RobotState::GUIDING:
    //             robotStr = "GUIDING";
    //             robotFill = Colors::OraBk;
    //             break;
    //         default:
    //             robotStr = "DISCONN";
    //             robotFill = Colors::RedBk;
    //             break;
    //     }
    //     AddBodyCell(robotStr, "C5", 3, 1, "center", bodyFontSize, robotFill);
    // }
    // AddBodyCell(kb.activeUserId > 0 ? std::to_string(kb.activeUserId) : "---", "C6", 3, 1, "center", bodyFontSize);

    // AddBorder("A1", 5, 6, Colors::GraMd, 2);

    // // Mouse
    // AddSubheadingCell("Mouse", "A7", 2, 1, "center", bodyFontSize);
    // AddBodyCell(touch.isTouched ? "CLICK" : "---", "A8", 2, 1, "center", bodyFontSize,
    //             touch.isTouched ? Colors::GreBk : Colors::GraBk);
    // AddBodyCell(touch.isTouched ? std::to_string(touch.position.x) : "--", "C7", 3, 1, "center", bodyFontSize);
    // AddBodyCell(touch.isTouched ? std::to_string(touch.position.y) : "--", "C8", 3, 1, "center", bodyFontSize);

    // AddBorder("A7", 5, 2, Colors::GraMd, 2);

    // kb.userId > 0 ? std::to_string(kb.userId) : "--",
    // ---- Marker visibility -------------------------------------------------------
    AddHeadingCell( "Marker Visibility", "F1", 8, 1, "center", headerFontSize );
    AddHeadingCell( std::to_string( markers.size() ), "N1", 1, 1, "center",
                    headerFontSize );
    // One cell per marker ID (1–45), 9 per row across columns A–I, rows 2–6.
    // Colour rules:
    //   active marker   → dark green background, white text
    //   detected        → default background,    white text
    //   not detected    → default background,    gray text
    auto isDetected = [&markers]( int id ) {
        for ( const auto &m : markers )
            if ( m.id == id ) return true;
        return false;
    };

    for ( int id = 1; id <= 45; id++ ) {
        std::string cellRef = std::string( 1, 'F' + ( id - 1 ) % 9 ) + std::to_string( 2 + ( id - 1 ) / 9 );

        bool detected = isDetected( id );
        bool active = ( id == kb.activeTagId );

        cv::Scalar fill = active ? Colors::GreBk : Colors::GraBk;
        cv::Scalar text = detected ? Colors::White : Colors::GraMd;

        AddBodyCell( std::to_string( id ), cellRef, 1, 1, "center", bodyFontSize, fill, text );
    }
    AddBorder( "F1", 9, 6, Colors::GraMd, 2 );

    // ---- Keyboard inputs -------------------
    // Row 8 spans 29 columns (S..AU, cols 18-46) to match the heading at S7.
    // Layout: "Input"(2) + inputBuffer(5) + outputBuffer(15) + "Last"(2) + lastInputKey(5) = 29
    AddHeadingCell( "Keyboard Input Monitor", "S7", 29, 1, "center", bodyFontSize );
    AddSubheadingCell( "Input", "S8", 2, 1, "center", bodyFontSize );
    AddBodyCell( kb.inputBuffer, "U8", 2, 1, "center", bodyFontSize );
    AddBodyCell( kb.outputBuffer, "W8", 21, 1, "center", bodyFontSize );
    AddSubheadingCell( "Last", "AR8", 2, 1, "center", bodyFontSize );
    AddBodyCell( FormatLastInputKey( kb.lastInputKey ), "AT8", 2, 1, "center", bodyFontSize );    // Last keyboard entry
    AddBorder( "S7", 29, 2, Colors::GraMd, 2 );

    // Format a float to two decimal places (e.g., "12.34")
    auto fmt2Dec = []( float v ) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision( 2 ) << v;
        return ss.str();
    };

    // ---- Serial connection information ----------------------------------------------
    AddHeadingCell( "Serial Communication", "S1", 29, 1, "center", headerFontSize );
    AddSubheadingCell( "Status", "S2", 3, 1, "center", bodyFontSize );
    AddSubheadingCell( "Teensy State", "V2", 3, 1, "center", bodyFontSize );
    AddSubheadingCell( "Out", "Y2", 1, 1, "center", bodyFontSize );

    AddSubheadingCell( "PwmA", "Z2", 2, 1, "center", bodyFontSize );
    AddSubheadingCell( "PwmB", "AB2", 2, 1, "center", bodyFontSize );
    AddSubheadingCell( "PwmC", "AD2", 2, 1, "center", bodyFontSize );

    AddSubheadingCell( "In", "AF2", 1, 1, "center", bodyFontSize );
    AddSubheadingCell( "EncA", "AG2", 3, 1, "center", bodyFontSize );
    AddSubheadingCell( "EncB", "AJ2", 3, 1, "center", bodyFontSize );
    AddSubheadingCell( "EncC", "AM2", 3, 1, "center", bodyFontSize );
    AddSubheadingCell( "CurA", "AP2", 2, 1, "center", bodyFontSize );
    AddSubheadingCell( "CurB", "AR2", 2, 1, "center", bodyFontSize );
    AddSubheadingCell( "CurC", "AT2", 2, 1, "center", bodyFontSize );

    // Connection status - coloured background makes it easy to spot at a glance
    AddBodyCell( serial.isConnected ? "Connected" : "No Teensy",
                 "S3", 3, 1, "center", bodyFontSize,
                 serial.isConnected ? Colors::GreBk : Colors::RedBk );

    // Teensy state byte echoed back (single ASCII char: 'I'=idle, 'D'=drive, etc.)
    AddBodyCell( serial.hasRx ? std::string( 1, static_cast<char>( serial.lastRx.state ) ) : "--",
                 "V3", 3, 1, "center", bodyFontSize );

    // Outgoing packet index (0–99 rolling)
    AddBodyCell( std::to_string( serial.lastTx.packet_index ), "Y3", 1, 1, "center", bodyFontSize );

    // Commanded PWM values (0 = full power, 2047 = no power)
    AddBodyCell( std::to_string( serial.lastTx.pwm_A ), "Z3", 2, 1, "center", bodyFontSize );
    AddBodyCell( std::to_string( serial.lastTx.pwm_B ), "AB3", 2, 1, "center", bodyFontSize );
    AddBodyCell( std::to_string( serial.lastTx.pwm_C ), "AD3", 2, 1, "center", bodyFontSize );

    // Incoming data - only shown once at least one valid RX packet has arrived
    const std::string na = "--";
    AddBodyCell( serial.hasRx ? std::to_string( serial.lastRx.packet_index ) : na, "AF3", 1, 1, "center", bodyFontSize );
    AddBodyCell( serial.hasRx ? std::to_string( serial.lastRx.encoder_count_A ) : na, "AG3", 3, 1, "center", bodyFontSize );
    AddBodyCell( serial.hasRx ? std::to_string( serial.lastRx.encoder_count_B ) : na, "AJ3", 3, 1, "center", bodyFontSize );
    AddBodyCell( serial.hasRx ? std::to_string( serial.lastRx.encoder_count_C ) : na, "AM3", 3, 1, "center", bodyFontSize );
    AddBodyCell( serial.hasRx ? fmt2Dec( serial.lastRx.current_raw_A * 0.01 ) : na, "AP3", 2, 1, "center", bodyFontSize );
    AddBodyCell( serial.hasRx ? fmt2Dec( serial.lastRx.current_raw_B * 0.01 ) : na, "AR3", 2, 1, "center", bodyFontSize );
    AddBodyCell( serial.hasRx ? fmt2Dec( serial.lastRx.current_raw_C * 0.01 ) : na, "AT3", 2, 1, "center", bodyFontSize );
    AddBorder( "S1", 29, 3, Colors::GraMd, 2 );

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

void DisplayHandler::DrawTelCell( const std::string &text,
                                  const std::string &cellRef, int colSpan,
                                  int rowSpan, const std::string &align,
                                  float fontSize, const cv::Scalar &textColor,
                                  const cv::Scalar &fillColor, CellStyle style ) {
    cv::Rect r = CellRect( cellRef, colSpan, rowSpan );

    // Fill background
    cv::rectangle( matTelemetry_, r, fillColor, cv::FILLED );

    // Thin separator line - same as the reference's subtle white grid line
    cv::rectangle( matTelemetry_, r, Colors::GraDk, 1 );

    if ( text.empty() )
        return;

    // Choose font based on style
    int fontFace = ( style == CellStyle::HEADING ) ? cv::FONT_HERSHEY_DUPLEX
                                                   : cv::FONT_HERSHEY_SIMPLEX;

    int baseLine = 0;
    cv::Size textSz = cv::getTextSize( text, fontFace, fontSize, 1, &baseLine );

    // Vertical centre of the cell
    int textY = r.y + ( r.height + textSz.height ) / 2;

    // Horizontal position based on alignment
    int textX = 0;
    if ( align == "center" ) {
        textX = r.x + ( r.width - textSz.width ) / 2;
    } else if ( align == "right" ) {
        textX = r.x + r.width - textSz.width - 4;
    } else {
        // left (default)
        textX = r.x + 4;
    }

    cv::putText( matTelemetry_, text, cv::Point( textX, textY ), fontFace, fontSize,
                 textColor, 1, cv::LINE_AA );
}

cv::Point2i DisplayHandler::ParseCellRef( const std::string &ref ) const {
    int col = 0;
    int numStart = 0;

    // Determine column index from leading letter(s)
    if ( ref.size() > 1 && std::isalpha( static_cast<unsigned char>( ref[1] ) ) ) {
        // Two-letter column: AA = 26, AB = 27, ... AX = 49
        col = 26 + ( ref[1] - 'A' );
        numStart = 2;
    } else {
        // Single-letter column: A = 0, B = 1, ... Z = 25
        col = ref[0] - 'A';
        numStart = 1;
    }

    int row = std::stoi( ref.substr( numStart ) ) - 1;    // 1-based → 0-based
    return cv::Point2i( col, row );
}

cv::Rect DisplayHandler::CellRect( const std::string &ref, int colSpan,
                                   int rowSpan ) const {
    cv::Point2i pos = ParseCellRef( ref );
    return cv::Rect( pos.x * cellW_, pos.y * cellH_, colSpan * cellW_ + 1,
                     rowSpan * cellH_ + 1 );
}

// =============================================================================
// Controller panel - private helpers
// =============================================================================

void DisplayHandler::SetCal3State( bool isComplete, cv::Point3f offset, float rollRefRad ) {
    cal3Complete_ = isComplete;
    cal3Offset_ = offset;
    cal3RollDeg_ = rollRefRad * RAD_TO_DEG;
}

void DisplayHandler::SetVirtualFingertip( bool visible, cv::Point2i px ) {
    virtualFingertipVisible_ = visible;
    virtualFingertipPx_ = px;
}

void DisplayHandler::SetTouchFingertip( bool visible, cv::Point2i px ) {
    touchFingertipVisible_ = visible;
    touchFingertipPx_ = px;
}

void DisplayHandler::SetControllerTelemetry( const ControllerTelemetry &tele ) {
    controllerTele_ = tele;
}

void DisplayHandler::SetCal1State( bool recording, const std::vector<cv::Point2f> &samples,
                                   const AromBoundary &boundary ) {
    cal1Recording_ = recording;
    cal1Samples_ = samples;
    cal1Boundary_ = boundary;
}

void DisplayHandler::SetCal2State( bool recording, int headingIdx ) {
    cal2Recording_ = recording;
    cal2HeadingIdx_ = headingIdx;
}

void DisplayHandler::SetTargetCircle( bool visible, cv::Point2i centerPx, int radiusPx, cv::Scalar color ) {
    targetCircleVisible_ = visible;
    targetCircleCenterPx_ = centerPx;
    targetCircleRadiusPx_ = radiusPx;
    targetCircleColor_ = color;
}

void DisplayHandler::SetGestureIndicator( bool active, GestureEvent event ) {
    gestureIndicatorActive_ = active;
    gestureEvent_ = event;
}

void DisplayHandler::PopulateControllerPanel( const std::vector<DetectedMarker> &markers,
                                              const TouchState &touch, const KeyboardState &kb,
                                              const SerialState &serial ) {
    ClearController();
    // Populate controller telemetry cells here as the controller is implemented

    float headerFontSize = 0.55f;
    float bodyFontSize = 0.45f;
    float tableFontSize = 0.4f;

    // --- Formatting Helpers ------------------------------------------------------------------------------------------
    // Format float (0.1f)
    auto fmtMm = []( float v ) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision( 1 ) << v;
        return ss.str();
    };
    // Format radians (0.2f)
    auto fmtRad = []( float v ) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision( 2 ) << v;
        return ss.str();
    };
    // Format float (0.3f)
    auto fmtMm3 = []( float v ) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision( 3 ) << v;
        return ss.str();
    };
    // Format float (0.2f)
    auto fmtNum2 = []( float v ) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision( 2 ) << v;
        return ss.str();
    };

    // --- System Readiness Panel ---------------------------------------------------------------------------
    AddControllerHeadingCell( "System Readiness", "A1", 15, 1, "center", headerFontSize );

    // Current program state (kb.systemState)
    std::string systemStateStr;
    switch ( kb.systemState ) {
        case SystemState::CALIBRATING:
            systemStateStr = "CALIBRATING";
            break;
        case SystemState::CAL3:
            systemStateStr = "CAL3";
            break;
        case SystemState::FITTS:
            systemStateStr = "FITTS";
            break;
        case SystemState::PRETENSION:
            systemStateStr = "PRETENSION";
            break;
        case SystemState::TENSION_ADJUST:
            systemStateStr = "TENSION ADJUST";
            break;
        default:
            systemStateStr = "IDLE";
            break;
    }

    // Teensy state byte echoed back (see T_PacketTypes.h TeensyState namespace:
    // 'W'=WAITING, 'I'=IDLE, 'D'=DRIVING, 'R'=READY)
    std::string teensyStateStr = "--";
    if ( serial.hasRx ) {
        switch ( static_cast<char>( serial.lastRx.state ) ) {
            case 'W':
                teensyStateStr = "WAITING";
                break;
            case 'I':
                teensyStateStr = "IDLE";
                break;
            case 'D':
                teensyStateStr = "DRIVING";
                break;
            case 'R':
                teensyStateStr = "READY";
                break;
            default:
                teensyStateStr = std::string( 1, static_cast<char>( serial.lastRx.state ) );
                break;
        }
    }

    // Amplifier state: INACTIVE (no PWM output), GUIDING (guidance force
    // contributing to PWM while a target is active), or ACTIVE (PWM output
    // for tensioning/other processes, no guidance contribution).
    std::string amplifierStateStr;
    cv::Scalar amplifierColor;
    if ( !controllerTele_.outputEnabled ) {
        amplifierStateStr = "INACTIVE";
        amplifierColor = Colors::GraBk;
    } else if ( controllerTele_.isTargetActive && controllerTele_.guidanceOutputEnabled ) {
        amplifierStateStr = "GUIDING";
        amplifierColor = Colors::BluBk;
    } else {
        amplifierStateStr = "ACTIVE";
        amplifierColor = Colors::GreBk;
    }

    // States
    AddControllerSubheadingCell( "PC State", "A2", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Camera", "A3", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Serial", "A4", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Teensy State", "A5", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Amplifier State", "A6", 4, 1, "center", bodyFontSize );
    AddControllerBodyCell( systemStateStr, "E2", 4, 1, "center", bodyFontSize,
                           kb.systemState == SystemState::IDLE ? Colors::RedBk : Colors::GreBk );    // PC program state
    AddControllerBodyCell( std::to_string( static_cast<int>( measuredFreqHz_ ) ) + " Hz", "E3", 4, 1, "center", bodyFontSize,
                           measuredFreqHz_ >= 60.0f ? Colors::GreBk : Colors::RedBk );    // Camera detection loop frequency
    AddControllerBodyCell( std::to_string( static_cast<int>( serial.txFrequencyHz ) ) + " Hz", "E4", 4, 1, "center", bodyFontSize,
                           ( serial.isConnected && serial.txFrequencyHz >= 150.0f ) ? Colors::GreBk : Colors::RedBk );    // Serial TX frequency
    AddControllerBodyCell( teensyStateStr, "E5", 4, 1, "center", bodyFontSize,
                           serial.hasRx ? Colors::GreBk : Colors::RedBk );                             // Teensy state
    AddControllerBodyCell( amplifierStateStr, "E6", 4, 1, "center", bodyFontSize, amplifierColor );    // Amplifier state

    // Calibration
    AddControllerSubheadingCell( "Tension", "I2", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Home Position", "I3", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "C1: ARoM", "I4", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "C2: Stiffness", "I5", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "C3: Offset", "I6", 4, 1, "center", bodyFontSize );
    AddControllerBodyCell( controllerTele_.outputEnabled ? "SET" : "Not Set", "M2", 3, 1, "center", bodyFontSize, controllerTele_.outputEnabled ? Colors::GreBk : Colors::RedBk );      // Preload tension held
    AddControllerBodyCell( controllerTele_.homeSet ? "SET" : "Not Set", "M3", 3, 1, "center", bodyFontSize, controllerTele_.homeSet ? Colors::GreBk : Colors::RedBk );                  // Home position recorded
    AddControllerBodyCell( cal1Boundary_.valid ? "SET" : "Not Set", "M4", 3, 1, "center", bodyFontSize, cal1Boundary_.valid ? Colors::GreBk : Colors::RedBk );                          // AROM boundary computed
    AddControllerBodyCell( controllerTele_.stiffnessValid ? "SET" : "Not Set", "M5", 3, 1, "center", bodyFontSize, controllerTele_.stiffnessValid ? Colors::GreBk : Colors::RedBk );    // Stiffness profile K(theta) computed
    AddControllerBodyCell( cal3Complete_ ? "SET" : "Not Set", "M6", 3, 1, "center", bodyFontSize, cal3Complete_ ? Colors::GreBk : Colors::RedBk );                                      // Fingertip-to-camera offset computed
    AddControllerBorder( "A1", 15, 6, Colors::GraMd, 2 );

    // --- Target Telemetry Panel ----------------------------------------------------------------------------
    AddControllerHeadingCell( "Target Telemetry / Error Vector", "A7", 15, 1, "center", headerFontSize );
    AddControllerSubheadingCell( "", "A8", 5, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "x", "F8", 2, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "y", "H8", 2, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "z", "J8", 2, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Rxy", "L8", 2, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Rxyz", "N8", 2, 1, "center", bodyFontSize );
    // Active marker position (pos_camera from solvePnP) - moved here from System Information
    const DetectedMarker *activeMarker = nullptr;
    if ( kb.activeTagId > 0 ) {
        for ( const auto &m : markers ) {
            if ( m.id == kb.activeTagId ) {
                activeMarker = &m;
                break;
            }
        }
    }

    // Position
    AddControllerSubheadingCell( "Position [mm]", "A9", 5, 1, "center", bodyFontSize );
    if ( activeMarker ) {
        const float x = activeMarker->positionMm.x;
        const float y = activeMarker->positionMm.y;
        const float z = activeMarker->positionMm.z;
        AddControllerBodyCell( fmtMm( x ), "F9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( fmtMm( y ), "H9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( fmtMm( z ), "J9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( fmtMm( std::sqrt( x * x + y * y ) ), "L9", 2, 1, "center", bodyFontSize );            // Rxy
        AddControllerBodyCell( fmtMm( std::sqrt( x * x + y * y + z * z ) ), "N9", 2, 1, "center", bodyFontSize );    // Rxyz
    } else {
        AddControllerBodyCell( "--", "F9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "H9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "J9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "L9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "N9", 2, 1, "center", bodyFontSize );
    }

    // Velocity - virtual fingertip velocity (low-pass filtered), no z-component
    AddControllerSubheadingCell( "Velocity [mm/s]", "A10", 5, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.vel_virtual.x ), "F10", 2, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.vel_virtual.y ), "H10", 2, 1, "center", bodyFontSize );
    AddControllerBodyCell( "--", "J10", 2, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( std::sqrt( controllerTele_.vel_virtual.x * controllerTele_.vel_virtual.x + controllerTele_.vel_virtual.y * controllerTele_.vel_virtual.y ) ), "L10", 2, 1, "center", bodyFontSize );
    AddControllerBodyCell( "--", "N10", 2, 1, "center", bodyFontSize );

    // Accumulated Error - PID integral term (accumulated position error), no z-component
    AddControllerSubheadingCell( "Accumulated [mm]", "A11", 5, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.posErrorIntegral.x ), "F11", 2, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.posErrorIntegral.y ), "H11", 2, 1, "center", bodyFontSize );
    AddControllerBodyCell( "--", "J11", 2, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( std::sqrt( controllerTele_.posErrorIntegral.x * controllerTele_.posErrorIntegral.x + controllerTele_.posErrorIntegral.y * controllerTele_.posErrorIntegral.y ) ), "L11", 2, 1, "center", bodyFontSize );
    AddControllerBodyCell( "--", "N11", 2, 1, "center", bodyFontSize );
    AddControllerBorder( "A7", 15, 5, Colors::GraMd, 2 );

    // --- Controller Output Panel ----------------------------------------------------------------------------
    // Header
    AddControllerHeadingCell( "Controller Output", "A12", 15, 1, "center", headerFontSize );
    AddControllerSubheadingCell( "", "A13", 6, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "A | AdEx", "G13", 3, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "B | AbEx", "J13", 3, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "C | Flex", "M13", 3, 1, "center", bodyFontSize );

    // Tension - preload tensions set during the pretensioning process. While the
    // operator is actively adjusting tension (pretensioning step 3/4, before the
    // value is confirmed with Enter / SetPreloadTensions()), show the live
    // tension/PWM being adjusted instead of the stale preload from last time.
    cv::Point3f tensionRowN = controllerTele_.manualTensionMode ? controllerTele_.tension : controllerTele_.preloadTension;
    cv::Point3f tensionRowPwm = controllerTele_.manualTensionMode ? controllerTele_.pwm : controllerTele_.preloadPwm;
    AddControllerSubheadingCell( "Tension", "A14", 3, 2, "center", bodyFontSize );
    AddControllerSubheadingCell( "N", "D14", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( fmtNum2( tensionRowN.x ), "G14", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( fmtNum2( tensionRowN.y ), "J14", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( fmtNum2( tensionRowN.z ), "M14", 3, 1, "center", tableFontSize );
    AddControllerSubheadingCell( "PWM", "D15", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( 2048 - tensionRowPwm.x ) ), "G15", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( 2048 - tensionRowPwm.y ) ), "J15", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( 2048 - tensionRowPwm.z ) ), "M15", 3, 1, "center", tableFontSize );

    // Force - per-motor tension/PWM contribution from the guidance force command
    AddControllerSubheadingCell( "Force", "A16", 3, 2, "center", bodyFontSize );
    AddControllerSubheadingCell( "N", "D16", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( fmtNum2( controllerTele_.deflectionForce.x ), "G16", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( fmtNum2( controllerTele_.deflectionForce.y ), "J16", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( fmtNum2( controllerTele_.deflectionForce.z ), "M16", 3, 1, "center", tableFontSize );
    AddControllerSubheadingCell( "PWM", "D17", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( std::lround( controllerTele_.deflectionForcePwm.x ) ) ), "G17", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( std::lround( controllerTele_.deflectionForcePwm.y ) ) ), "J17", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( std::lround( controllerTele_.deflectionForcePwm.z ) ) ), "M17", 3, 1, "center", tableFontSize );

    // Stiffness - K(theta), the stiffness-calibration profile interpolated at
    // the current error heading. Independent of "Gain kP" (gainTune) below -
    // only reflects the Cal2 stiffness calibration result. A single scalar
    // (not per-motor), so it spans the A/B/C columns.
    AddControllerSubheadingCell( "Stiffness", "A18", 3, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "N/mm", "D18", 3, 1, "center", tableFontSize );
    // Cell color: green when K(theta) is measured and contributing to
    // kP_effective, orange when measured but not contributing (disabled via
    // 'k' toggle), dark red when Cal2 hasn't produced a profile yet.
    AddControllerBodyCell( fmtNum2( controllerTele_.stiffnessGain ), "G18", 9, 1, "center", tableFontSize,
                           !controllerTele_.stiffnessValid        ? Colors::RedBk
                           : controllerTele_.stiffnessGainEnabled ? Colors::GreBk
                                                                  : Colors::OraBk );

    // Gain kP - custom-tuned proportional gain per motor, seeded from
    // cfg_.gain_kP and adjustable via 'G' (the other term in kP_effective).
    AddControllerSubheadingCell( "Gain kP", "A19", 3, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "N/mm", "D19", 3, 1, "center", tableFontSize );
    // Cell color: green when this motor's gain is contributing to
    // kP_effective, black (default) when it's zero.
    AddControllerBodyCell( fmtNum2( controllerTele_.gainTune.x ), "G19", 3, 1, "center", tableFontSize,
                           controllerTele_.gainTune.x > 0.0f ? Colors::GreBk : Colors::Black );
    AddControllerBodyCell( fmtNum2( controllerTele_.gainTune.y ), "J19", 3, 1, "center", tableFontSize,
                           controllerTele_.gainTune.y > 0.0f ? Colors::GreBk : Colors::Black );
    AddControllerBodyCell( fmtNum2( controllerTele_.gainTune.z ), "M19", 3, 1, "center", tableFontSize,
                           controllerTele_.gainTune.z > 0.0f ? Colors::GreBk : Colors::Black );

    // Integral Gain
    AddControllerSubheadingCell( "Gain kI", "A20", 3, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "N/mm*s", "D20", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( "--", "G20", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( "--", "J20", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( "--", "M20", 3, 1, "center", tableFontSize );
    
    // Output - T_output = T_preload + T_deflection, clamped to [T_preload, tension_output_max]
    AddControllerSubheadingCell( "Output", "A21", 3, 3, "center", bodyFontSize );
    AddControllerSubheadingCell( "N", "D21", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( fmtNum2( controllerTele_.outputTension.x ), "G21", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( fmtNum2( controllerTele_.outputTension.y ), "J21", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( fmtNum2( controllerTele_.outputTension.z ), "M21", 3, 1, "center", tableFontSize );
    AddControllerSubheadingCell( "I", "D22", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( "--", "G22", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( "--", "J22", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( "--", "M22", 3, 1, "center", tableFontSize );
    AddControllerSubheadingCell( "PWM", "D23", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( controllerTele_.outputPwm.x ) ), "G23", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( controllerTele_.outputPwm.y ) ), "J23", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( controllerTele_.outputPwm.z ) ), "M23", 3, 1, "center", tableFontSize );

    AddControllerBorder( "A12", 15, 12, Colors::GraMd, 2 );

    // AddControllerHeadingCell("Calibration", "A1", 15, 1, "center", headerFontSize);
    // AddControllerSubheadingCell("A-RoM", "A2", 3, 1, "center", bodyFontSize);
    // AddControllerSubheadingCell("Stiffness", "D2", 4, 1, "center", bodyFontSize, (controllerTele_.stiffnessValid ? Colors::GreBk : Colors::GraBk));
    // AddControllerSubheadingCell("Fingertip Offset", "H2", 8, 1, "center", bodyFontSize, (cal3Complete_ ? Colors::GreBk : Colors::GraBk));

    // AddControllerBodyCell("[NOT SET]", "A3", 3, 1, "center", bodyFontSize);
    // AddControllerBodyCell((controllerTele_.stiffnessValid ? "SET" : "[NOT SET]"), "D3", 4, 1, "center", bodyFontSize);

    if ( cal3Complete_ ) {
        //     std::ostringstream rollSS;
        //     rollSS << std::fixed << std::setprecision(1) << cal3RollDeg_ << "\xc2\xb0";
        //     AddControllerBodyCell(fmtMm(cal3Offset_.x), "H3", 2, 1, "center", bodyFontSize);
        //     AddControllerBodyCell(fmtMm(cal3Offset_.y), "J3", 2, 1, "center", bodyFontSize);
        //     AddControllerBodyCell(fmtMm(cal3Offset_.z), "L3", 2, 1, "center", bodyFontSize);
        //     AddControllerBodyCell(rollSS.str(), "N3", 2, 1, "center", bodyFontSize);
        // } else {
        //     AddControllerBodyCell("[XX]", "H3", 2, 1, "center", bodyFontSize);
        //     AddControllerBodyCell("[YY]", "J3", 2, 1, "center", bodyFontSize);
        //     AddControllerBodyCell("[ZZ]", "L3", 2, 1, "center", bodyFontSize);
        //     AddControllerBodyCell("[TH]", "N3", 2, 1, "center", bodyFontSize);
    }
    // AddControllerBorder("A1", 15, 3, Colors::GraMd, 2);

    // // --- Tendon / Motor State --------------------------------------------------------------------

    // float tableFontSize = 0.4f;

    // AddControllerHeadingCell("Tendon / Motor State", "A4", 15, 1, "center", headerFontSize);
    // AddControllerSubheadingCell("Motor",   "A5", 1, 1, "center", tableFontSize);
    // AddControllerSubheadingCell("q_abs",   "B5", 2, 1, "center", tableFontSize);
    // AddControllerSubheadingCell("q_home",  "D5", 2, 1, "center", tableFontSize);
    // AddControllerSubheadingCell("r_eff",   "F5", 2, 1, "center", tableFontSize);
    // AddControllerSubheadingCell("dL",      "H5", 2, 1, "center", tableFontSize);
    // AddControllerSubheadingCell("Tension", "J5", 2, 1, "center", tableFontSize);
    // AddControllerSubheadingCell("Current", "L5", 2, 1, "center", tableFontSize);
    // AddControllerSubheadingCell("PWM",     "N5", 2, 1, "center", tableFontSize);

    // const char *motorRows[3]  = { "A", "B", "C" };
    // const float q_abs[3]  = { controllerTele_.q_abs.x,  controllerTele_.q_abs.y,  controllerTele_.q_abs.z };
    // const float q_home[3] = { controllerTele_.q_home.x, controllerTele_.q_home.y, controllerTele_.q_home.z };
    // const float r_eff[3]  = { controllerTele_.r_eff.x,  controllerTele_.r_eff.y,  controllerTele_.r_eff.z };
    // const float dL[3]     = { controllerTele_.dL.x,     controllerTele_.dL.y,     controllerTele_.dL.z };
    // const float tension[3] = { controllerTele_.tension.x, controllerTele_.tension.y, controllerTele_.tension.z };
    // const float current[3] = { controllerTele_.current.x, controllerTele_.current.y, controllerTele_.current.z };
    // const float pwm[3]     = { controllerTele_.pwm.x,     controllerTele_.pwm.y,     controllerTele_.pwm.z };

    // for (int i = 0; i < 3; i++) {
    //     std::string row = std::to_string(6 + i);
    //     AddControllerBodyCell(motorRows[i],              "A" + row, 1, 1, "center", tableFontSize);
    //     AddControllerBodyCell(fmtRad(q_abs[i]),          "B" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(fmtRad(q_home[i]),         "D" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(fmtMm3(r_eff[i] * 1000.0f), "F" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(fmtNum2(dL[i] * 1000.0f),  "H" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(fmtNum2(tension[i]),       "J" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(fmtNum2(current[i]),       "L" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(std::to_string(static_cast<int>(pwm[i])), "N" + row, 2, 1, "center", tableFontSize);
    // }

    // AddControllerSubheadingCell("Home Position Set", "A9", 7, 1, "center", bodyFontSize,
    //                             (controllerTele_.homeSet ? Colors::GreBk : Colors::GraBk));
    // AddControllerSubheadingCell("Output Enabled", "H9", 8, 1, "center", bodyFontSize,
    //                             (controllerTele_.outputEnabled ? Colors::GreBk : Colors::GraBk));

    // AddControllerBorder("A4", 15, 6, Colors::GraMd, 2);

    // // --- Stiffness profile K(theta) --------------------------------------------------------------
    // AddControllerHeadingCell("Stiffness Profile (N/mm)", "A10", 15, 1, "center", headerFontSize);

    // for (int i = 0; i < CONSTANT_CALIBRATION_ANGLE_COUNT; i++) {
    //     int  labelRow = (i < 5) ? 11 : 13;
    //     int  valueRow = labelRow + 1;
    //     char col      = static_cast<char>('A' + 3 * (i % 5));
    //     std::string colStr(1, col);

    //     std::ostringstream angleSS;
    //     angleSS << static_cast<int>(CONSTANT_CALIBRATION_ANGLES_DEG[i]) << "\xc2\xb0";
    //     AddControllerSubheadingCell(angleSS.str(), colStr + std::to_string(labelRow), 3, 1, "center", tableFontSize);

    //     std::string kStr = controllerTele_.stiffnessValid ? fmtNum2(controllerTele_.stiffnessProfile[i]) : "--";
    //     AddControllerBodyCell(kStr, colStr + std::to_string(valueRow), 3, 1, "center", tableFontSize);
    // }

    // AddControllerSubheadingCell(controllerTele_.stiffnessGainEnabled ? "Stiffness Gain: ON" : "Stiffness Gain: OFF",
    //                             "A15", 15, 1, "center", bodyFontSize,
    //                             (controllerTele_.stiffnessGainEnabled ? Colors::GreBk : Colors::GraBk));

    // AddControllerBorder("A10", 15, 6, Colors::GraMd, 2);

    // --- Virtual XY fingertip panel ------------------------------------------------------------------
    AddControllerHeadingCell( "Virtual Fingertip Mapping", "A24", 15, 1, "center", headerFontSize );
    AddControllerSubheadingCell( "Tendons", "A25", 6, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "A | AdEx", "G25", 3, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "B | AbEx", "J25", 3, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "C | Flex", "M25", 3, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Unspooled Angle", "A26", 6, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Home Angle", "A27", 6, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Effective Pulley Radius", "A28", 6, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Length Change", "A29", 6, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtRad( controllerTele_.q_abs.x ), "G26", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtRad( controllerTele_.q_abs.y ), "J26", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtRad( controllerTele_.q_abs.z ), "M26", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtRad( controllerTele_.q_home.x ), "G27", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtRad( controllerTele_.q_home.y ), "J27", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtRad( controllerTele_.q_home.z ), "M27", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.r_eff.x * 1000.0f ), "G28", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.r_eff.y * 1000.0f ), "J28", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.r_eff.z * 1000.0f ), "M28", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.dL.x * 1000.0f ), "G29", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.dL.y * 1000.0f ), "J29", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.dL.z * 1000.0f ), "M29", 3, 1, "center", bodyFontSize );

    AddControllerSubheadingCell( "Virtual Position", "A45", 5, 1, "center", headerFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.pos_virtual.x ), "F45", 5, 1, "center", headerFontSize );
    AddControllerBodyCell( fmtMm( controllerTele_.pos_virtual.y ), "K45", 5, 1, "center", headerFontSize );

    // const float tension[3] = { controllerTele_.tension.x, controllerTele_.tension.y, controllerTele_.tension.z };
    // const float current[3] = { controllerTele_.current.x, controllerTele_.current.y, controllerTele_.current.z };
    // const float pwm[3]     = { controllerTele_.pwm.x,     controllerTele_.pwm.y,     controllerTele_.pwm.z };

    // for (int i = 0; i < 3; i++) {
    //     AddControllerBodyCell(fmtNum2(tension[i]),       "J" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(fmtNum2(current[i]),       "L" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(std::to_string(static_cast<int>(pwm[i])), "N" + row, 2, 1, "center", tableFontSize);

    // Virtual Mapping Parameters
    cv::Point2i center = cv::Point2i( 225, 1090 );
    int radius = 200;

    // const float pwm[3]     = { controllerTele_.pwm.x,     controllerTele_.pwm.y,     controllerTele_.pwm.z };

    // Motor direction components
    int dAx = static_cast<int>( CONSTANT_UNIT_VECTOR_A_X * radius );
    int dAy = static_cast<int>( -CONSTANT_UNIT_VECTOR_A_Y * radius );    // screen y is flipped
    int dBx = static_cast<int>( CONSTANT_UNIT_VECTOR_B_X * radius );
    int dBy = static_cast<int>( -CONSTANT_UNIT_VECTOR_B_Y * radius );
    int dCx = static_cast<int>( CONSTANT_UNIT_VECTOR_C_X * radius );
    int dCy = static_cast<int>( -CONSTANT_UNIT_VECTOR_C_Y * radius );

    // Motor output lines
    int rA = static_cast<int>( ( 2047.0f - controllerTele_.pwm.x ) / 2047.0f * radius );
    int rB = static_cast<int>( ( 2047.0f - controllerTele_.pwm.y ) / 2047.0f * radius );
    int rC = static_cast<int>( ( 2047.0f - controllerTele_.pwm.z ) / 2047.0f * radius );
    cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_A_X * rA, -CONSTANT_UNIT_VECTOR_A_Y * rA ), Colors::MagLt, 6 );    // Motor A (35°)
    cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_B_X * rB, -CONSTANT_UNIT_VECTOR_B_Y * rB ), Colors::MagLt, 6 );    // Motor A (35°)
    cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_C_X * rC, -CONSTANT_UNIT_VECTOR_C_Y * rC ), Colors::MagLt, 6 );    // Motor A (35°)

    // Motor direction lines - use Globals.h unit vectors scaled to radius
    cv::line( matController_, center, center + cv::Point2i( dAx, dAy ), Colors::GraBk, 2 );    // Motor A (35°)
    cv::line( matController_, center, center + cv::Point2i( dBx, dBy ), Colors::GraBk, 2 );    // Motor B (145°)
    cv::line( matController_, center, center + cv::Point2i( dCx, dCy ), Colors::GraBk, 2 );    // Motor C (270°)

    // Fingertip Map Circle elements
    cv::circle( matController_, center, radius, Colors::GraDk, 2 );
    cv::circle( matController_, center, 4, Colors::GraDk, -1 );

    // Motor elements
    cv::circle( matController_, center + cv::Point2i( dAx, dAy ), 12, Colors::GraDk, -1 );
    cv::circle( matController_, center + cv::Point2i( dBx, dBy ), 12, Colors::GraDk, -1 );
    cv::circle( matController_, center + cv::Point2i( dCx, dCy ), 12, Colors::GraDk, -1 );
    cv::putText( matController_, "A", center + cv::Point( dAx - 6, dAy + 5 ), cv::FONT_HERSHEY_DUPLEX, 0.6f, Colors::GraWt, 1 );
    cv::putText( matController_, "B", center + cv::Point( dBx - 6, dBy + 5 ), cv::FONT_HERSHEY_DUPLEX, 0.6f, Colors::GraWt, 1 );
    cv::putText( matController_, "C", center + cv::Point( dCx - 6, dCy + 7 ), cv::FONT_HERSHEY_DUPLEX, 0.6f, Colors::GraWt, 1 );

    // Virtual fingertip position - pos_virtual is in mm, screen y is flipped
    constexpr float kVirtualPlotPxPerMm = 10.0f;
    cv::Point2i posPx = center + cv::Point2i(
                                     static_cast<int>( controllerTele_.pos_virtual.x * kVirtualPlotPxPerMm ),
                                     static_cast<int>( -controllerTele_.pos_virtual.y * kVirtualPlotPxPerMm ) );
    cv::circle( matController_, posPx, 6, Colors::CyaMd, -1 );

    // AROM samples - drawn live while Cal1Handler is recording
    if ( cal1Recording_ ) {
        for ( const cv::Point2f &s : cal1Samples_ ) {
            cv::Point2i samplePx = center + cv::Point2i(
                                                static_cast<int>( s.x * kVirtualPlotPxPerMm ),
                                                static_cast<int>( -s.y * kVirtualPlotPxPerMm ) );
            cv::circle( matController_, samplePx, 2, Colors::YelMd, -1 );
        }
        // Calibration angles
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_000_X * radius, -CONSTANT_UNIT_VECTOR_CAL_000_Y * radius ), Colors::MagDk, 1 );    // Cal 000°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_035_X * radius, -CONSTANT_UNIT_VECTOR_CAL_035_Y * radius ), Colors::MagDk, 1 );    // Cal 035°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_060_X * radius, -CONSTANT_UNIT_VECTOR_CAL_060_Y * radius ), Colors::MagDk, 1 );    // Cal 060°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_090_X * radius, -CONSTANT_UNIT_VECTOR_CAL_090_Y * radius ), Colors::MagDk, 1 );    // Cal 090°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_120_X * radius, -CONSTANT_UNIT_VECTOR_CAL_120_Y * radius ), Colors::MagDk, 1 );    // Cal 120°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_145_X * radius, -CONSTANT_UNIT_VECTOR_CAL_145_Y * radius ), Colors::MagDk, 1 );    // Cal 145°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_180_X * radius, -CONSTANT_UNIT_VECTOR_CAL_180_Y * radius ), Colors::MagDk, 1 );    // Cal 180°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_210_X * radius, -CONSTANT_UNIT_VECTOR_CAL_210_Y * radius ), Colors::MagDk, 1 );    // Cal 210°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_240_X * radius, -CONSTANT_UNIT_VECTOR_CAL_240_Y * radius ), Colors::MagDk, 1 );    // Cal 240°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_270_X * radius, -CONSTANT_UNIT_VECTOR_CAL_270_Y * radius ), Colors::MagDk, 1 );    // Cal 270°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_300_X * radius, -CONSTANT_UNIT_VECTOR_CAL_300_Y * radius ), Colors::MagDk, 1 );    // Cal 300°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_330_X * radius, -CONSTANT_UNIT_VECTOR_CAL_330_Y * radius ), Colors::MagDk, 1 );    // Cal 330°
    }

    // AROM boundary polygon - drawn once Cal1Handler has computed it
    if ( cal1Boundary_.valid ) {
        constexpr int kBoundaryPlotPoints = 72;
        std::vector<cv::Point> boundaryPts;
        boundaryPts.reserve( kBoundaryPlotPoints );
        for ( int i = 0; i < kBoundaryPlotPoints; i++ ) {
            float theta = ( CONSTANT_TWO_PI * i ) / kBoundaryPlotPoints;
            float r = cal1Boundary_.RadiusAtAngle( theta );
            boundaryPts.emplace_back( center + cv::Point2i(
                                                   static_cast<int>( r * std::cos( theta ) * kVirtualPlotPxPerMm ),
                                                   static_cast<int>( -r * std::sin( theta ) * kVirtualPlotPxPerMm ) ) );
        }
        cv::polylines( matController_, boundaryPts, true, Colors::OraMd, 2 );
    }

    // Calibration angles during Calibration 2 - the heading currently being
    // measured (cal2HeadingIdx_, index into CONSTANT_CALIBRATION_ANGLES_DEG)
    // is drawn in MagMd; all other headings stay MagDk.
    if ( cal2Recording_ ) {
        auto calAngleColor = [this]( int idx ) { return idx == cal2HeadingIdx_ ? Colors::MagMd : Colors::MagDk; };
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_000_X * radius, -CONSTANT_UNIT_VECTOR_CAL_000_Y * radius ), calAngleColor( 0 ), 1 );     // Cal 000°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_035_X * radius, -CONSTANT_UNIT_VECTOR_CAL_035_Y * radius ), calAngleColor( 1 ), 1 );     // Cal 035°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_060_X * radius, -CONSTANT_UNIT_VECTOR_CAL_060_Y * radius ), calAngleColor( 2 ), 1 );     // Cal 060°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_090_X * radius, -CONSTANT_UNIT_VECTOR_CAL_090_Y * radius ), calAngleColor( 3 ), 1 );     // Cal 090°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_120_X * radius, -CONSTANT_UNIT_VECTOR_CAL_120_Y * radius ), calAngleColor( 4 ), 1 );     // Cal 120°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_145_X * radius, -CONSTANT_UNIT_VECTOR_CAL_145_Y * radius ), calAngleColor( 5 ), 1 );     // Cal 145°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_180_X * radius, -CONSTANT_UNIT_VECTOR_CAL_180_Y * radius ), calAngleColor( 6 ), 1 );     // Cal 180°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_210_X * radius, -CONSTANT_UNIT_VECTOR_CAL_210_Y * radius ), calAngleColor( 7 ), 1 );     // Cal 210°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_240_X * radius, -CONSTANT_UNIT_VECTOR_CAL_240_Y * radius ), calAngleColor( 8 ), 1 );     // Cal 240°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_270_X * radius, -CONSTANT_UNIT_VECTOR_CAL_270_Y * radius ), calAngleColor( 9 ), 1 );     // Cal 270°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_300_X * radius, -CONSTANT_UNIT_VECTOR_CAL_300_Y * radius ), calAngleColor( 10 ), 1 );    // Cal 300°
        cv::line( matController_, center, center + cv::Point2i( CONSTANT_UNIT_VECTOR_CAL_330_X * radius, -CONSTANT_UNIT_VECTOR_CAL_330_Y * radius ), calAngleColor( 11 ), 1 );    // Cal 330°
    }

    // Stiffness profile polygon - drawn once Cal2Handler has produced K(theta).
    // Normalized so the stiffest heading reaches `radius` px; the rest scale
    // proportionally, giving a spider-chart view of K(theta).
    if ( controllerTele_.stiffnessValid ) {
        float kMax = *std::max_element( controllerTele_.stiffnessProfile.begin(),
                                        controllerTele_.stiffnessProfile.end() );
        if ( kMax > 0.0f ) {
            std::vector<cv::Point> stiffnessPts;
            stiffnessPts.reserve( CONSTANT_CALIBRATION_ANGLE_COUNT );
            for ( int i = 0; i < CONSTANT_CALIBRATION_ANGLE_COUNT; i++ ) {
                float theta = CONSTANT_CALIBRATION_ANGLES_DEG[i] * DEG_TO_RAD;
                float r = ( controllerTele_.stiffnessProfile[i] / kMax ) * radius;
                stiffnessPts.emplace_back( center + cv::Point2i(
                                                        static_cast<int>( r * std::cos( theta ) ),
                                                        static_cast<int>( -r * std::sin( theta ) ) ) );
            }
            cv::polylines( matController_, stiffnessPts, true, Colors::GreMd, 2 );
        }
    }

    // Gesture indicator - green arrow (flick) or ring (confirm circle),
    // disappears after cooldownSecs / circleCooldownSecs
    if ( gestureIndicatorActive_ ) {
        constexpr int kArrowHalfLen = 80;
        cv::Point2i top = center + cv::Point2i( 0, -kArrowHalfLen );
        cv::Point2i bottom = center + cv::Point2i( 0, kArrowHalfLen );
        if ( gestureEvent_ == GestureEvent::FLICK_UP ) {
            cv::arrowedLine( matController_, bottom, top, Colors::GreMd, 4, cv::LINE_AA, 0, 0.3 );
        } else if ( gestureEvent_ == GestureEvent::FLICK_DOWN ) {
            cv::arrowedLine( matController_, top, bottom, Colors::GreMd, 4, cv::LINE_AA, 0, 0.3 );
        } else if ( gestureEvent_ == GestureEvent::CONFIRM ) {
            cv::circle( matController_, center, 60, Colors::GreMd, 4, cv::LINE_AA );
        }
    }

    // Border
    AddControllerBorder( "A24", 15, 22, Colors::GraMd, 2 );
}

void DisplayHandler::DrawControllerCell( const std::string &text, const std::string &cellRef,
                                         int colSpan, int rowSpan, const std::string &align,
                                         float fontSize, const cv::Scalar &textColor,
                                         const cv::Scalar &fillColor, CellStyle style ) {
    cv::Rect r = ControllerCellRect( cellRef, colSpan, rowSpan );

    cv::rectangle( matController_, r, fillColor, cv::FILLED );
    cv::rectangle( matController_, r, Colors::GraDk, 1 );

    if ( text.empty() ) return;

    int fontFace = 0;
    if ( style == CellStyle::BODY || fontSize <= 0.4f ) {
        fontFace = cv::FONT_HERSHEY_SIMPLEX;
    } else {
        fontFace = cv::FONT_HERSHEY_DUPLEX;
    }

    int baseLine = 0;
    cv::Size textSz = cv::getTextSize( text, fontFace, fontSize, 1, &baseLine );

    int textY = r.y + ( r.height + textSz.height ) / 2;
    int textX = 0;
    if ( align == "center" ) {
        textX = r.x + ( r.width - textSz.width ) / 2;
    } else if ( align == "right" ) {
        textX = r.x + r.width - textSz.width - 4;
    } else {
        textX = r.x + 4;
    }

    cv::putText( matController_, text, cv::Point( textX, textY ), fontFace, fontSize,
                 textColor, 1, cv::LINE_AA );
}

cv::Rect DisplayHandler::ControllerCellRect( const std::string &ref, int colSpan, int rowSpan ) const {
    cv::Point2i pos = ParseCellRef( ref );
    return cv::Rect( pos.x * controllerCellW_, pos.y * controllerCellH_,
                     colSpan * controllerCellW_, rowSpan * controllerCellH_ + 1 );
}

// =============================================================================
// Operator display - private helpers
// =============================================================================

void DisplayHandler::DrawCameraElements( cv::Mat &frame ) {
    // Horizontal line at the camera principal point Y
    cv::line( frame, cv::Point( 0, principalPoint_.y ),
              cv::Point( frame.cols - 1, principalPoint_.y ),
              cv::Scalar( 0, 200, 200 ), 1 );

    // Vertical line at the camera principal point X
    cv::line( frame, cv::Point( principalPoint_.x, 0 ),
              cv::Point( principalPoint_.x, frame.rows - 1 ),
              cv::Scalar( 0, 200, 200 ), 1 );
}

void DisplayHandler::DrawMarkerOverlays(
    cv::Mat &frame, const std::vector<DetectedMarker> &markers,
    int activeTagId ) {
    for ( const auto &m : markers ) {
        if ( activeTagId > 0 && m.id == activeTagId ) {
            // Active tag: green outline square + ID label
            std::vector<cv::Point> corners( 4 );
            for ( int k = 0; k < 4; k++ )
                corners[k] = cv::Point( static_cast<int>( m.cornersPx[k].x ),
                                        static_cast<int>( m.cornersPx[k].y ) );
            cv::polylines( frame, corners, true, cv::Scalar( 0, 255, 0 ), 2 );
            cv::putText( frame, "ID " + std::to_string( m.id ),
                         m.centerPx + cv::Point2i( 10, -10 ), cv::FONT_HERSHEY_SIMPLEX,
                         0.6, cv::Scalar( 0, 255, 0 ), 2 );

        } else {
            // Non-active markers: small dot and ID text
            cv::circle( frame, m.centerPx, 6, cv::Scalar( 0, 200, 0 ), cv::FILLED );
            cv::putText( frame, "ID " + std::to_string( m.id ),
                         m.centerPx + cv::Point2i( 10, -10 ), cv::FONT_HERSHEY_SIMPLEX,
                         0.6, cv::Scalar( 0, 200, 0 ), 2 );
        }
    }
}

void DisplayHandler::DrawTelemetryBar(
    cv::Mat &frame, const std::vector<DetectedMarker> &markers,
    const TouchState &touch, const KeyboardState &kb ) {
    const int x = 10;
    const float font = 0.5f;
    const auto col = cv::Scalar( 210, 210, 210 );

    // Bottom line - marker count and touch state
    std::string touchStr = touch.isTouched
                               ? ( "TOUCH (" + std::to_string( touch.position.x ) +
                                   ", " + std::to_string( touch.position.y ) + ")" )
                               : "no touch";
    std::string activeStr =
        ( kb.activeTagId > 0 )
            ? ( "  |  Active tag: ID " + std::to_string( kb.activeTagId ) )
            : "";

    // cv::putText(frame,
    //             "Markers: " + std::to_string(markers.size()) + "  |  " +
    //                 touchStr + activeStr,
    //             cv::Point2i(x, frame.rows - 12), cv::FONT_HERSHEY_SIMPLEX, font,
    //             col, 1);

    // // Second line from bottom - shows the command currently being typed
    // if (!kb.inputBuffer.empty()) {
    //     cv::putText(frame, "Cmd> " + kb.inputBuffer,
    //                 cv::Point2i(x, frame.rows - 32), cv::FONT_HERSHEY_SIMPLEX, font,
    //                 cv::Scalar(100, 220, 255), 1);
    // }
}
