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
        case 255:
            return "DEL";
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

DisplayHandler::DisplayHandler( const DisplayConfig         &cfg,
                                cv::Point2i                  principalPoint,
                                const TelemetryConfig       &telCfg,
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

void DisplayHandler::Update( const cv::Mat                     &frame,
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
        int     cropW = std::min( frame.cols, cfg_.width );
        int     cropH = std::min( frame.rows, cfg_.height );
        cv::Mat canvas = frame( cv::Rect( 0, 0, cropW, cropH ) ).clone();

        DrawCameraElements( canvas );
        DrawMarkerOverlays( canvas, markers, kb.activeTagId );
        DrawObjectOverlays( canvas );

        if ( targetCircleVisible_ ) {
            // FITTS target - active tag center offset "under" by the Cal3 Y
            // offset (or the default before Cal3 completes). Color is red
            // (cal3 complete) or gray (default position, not yet calibrated).
            // cv::circle( canvas, targetCircleCenterPx_, targetCircleRadiusPx_, targetCircleColor_, 1 );
        }

        if ( virtualTargetVisible_ ) {
            // Green dot = guiding position ("virtual marker"): where the marker
            // centre must be steered so the fingertip lands on the target. It sits
            // at the marker centre plus the roll-induced offset (d - R*d).
            cv::circle( canvas, virtualTargetPx_, 6, Colors::GreMd, -1 );
            cv::circle( canvas, virtualTargetPx_, 6, Colors::GreLt, 1 );
        }

        // if ( touchFingertipVisible_ ) {
        //     // Frozen fingertip position at the moment of the last touchscreen
        //     // contact - stays until the next target is selected.
        //     cv::circle( canvas, touchFingertipPx_, 3, Colors::BluMd, -1 );
        // }

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
                                        float             fontSize,
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
    AddHeadingCell( "Marker Visibility", "A1", 12, 1, "center", headerFontSize );
    // AddHeadingCell( std::to_string( markers.size() ), "L1", 1, 1, "center", headerFontSize );
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

    int pad = 13;
    int xStart = 16;
    int yStart = 55;
    int r = 4;

    // Corner cells overlap coarse marker guard zones and have no fine marker.
    // The guard boundary at row=3 is a pixel-exact touch (area=0), so row 3 is NOT skipped.
    auto isFineSkipped = []( int col, int row ) -> bool {
        return ( col <= 2 || col >= 27 ) && ( row <= 2 || row >= 12 );
    };
    // Returns the fine marker ID (1-based, row-major) for (col, row), or -1 if skipped.
    auto fineMarkerId = [&isFineSkipped]( int col, int row ) -> int {
        if ( isFineSkipped( col, row ) ) return -1;
        int id = 1;
        for ( int rr = 0; rr < 15; rr++ ) {
            for ( int cc = 0; cc < 30; cc++ ) {
                if ( rr == row && cc == col ) return id;
                if ( !isFineSkipped( cc, rr ) ) id++;
            }
        }
        return -1;
    };

    auto markerColor = [&]( int id ) -> cv::Scalar {
        if ( id == kb.activeTagId ) return isDetected( id ) ? Colors::GreMd : Colors::GraWt;
        return isDetected( id ) ? Colors::GreDk : Colors::GraMd;
    };

    // Top 3 fine rows
    for ( int i = 3; i < 27; i++ ) {
        for ( int j = 0; j < 3; j++ ) {
            cv::circle( matTelemetry_, cv::Point( xStart + ( i * pad ), yStart + ( j * pad ) ), r, markerColor( fineMarkerId( i, j ) ), -1 );
        }
    }
    // Middle 9 fine rows
    for ( int i = 0; i < 30; i++ ) {
        for ( int j = 0; j < 9; j++ ) {
            cv::circle( matTelemetry_, cv::Point( xStart + ( i * pad ), ( yStart + ( pad * 3 ) + ( j * pad ) ) ), r, markerColor( fineMarkerId( i, j + 3 ) ), -1 );
        }
    }
    // Bottom 3 fine rows
    for ( int i = 3; i < 27; i++ ) {
        for ( int j = 12; j < 15; j++ ) {
            cv::circle( matTelemetry_, cv::Point( xStart + ( i * pad ), yStart + ( j * pad ) ), r, markerColor( fineMarkerId( i, j ) ), -1 );
        }
    }
    // Corner coarse markers — IDs follow the fine band: 1 + fineRows*fineCols + (0..3)
    const int coarseIdBase = 1 + 15 * 30;    // = 451; matches FittsBoardLayout coarse ID assignment
    cv::circle( matTelemetry_, cv::Point( 30, 68 ), 10, isDetected( coarseIdBase + 0 ) ? Colors::GreDk : Colors::GraMd, -1 );
    cv::circle( matTelemetry_, cv::Point( 379, 68 ), 10, isDetected( coarseIdBase + 1 ) ? Colors::GreDk : Colors::GraMd, -1 );
    cv::circle( matTelemetry_, cv::Point( 30, 225 ), 10, isDetected( coarseIdBase + 2 ) ? Colors::GreDk : Colors::GraMd, -1 );
    cv::circle( matTelemetry_, cv::Point( 379, 225 ), 10, isDetected( coarseIdBase + 3 ) ? Colors::GreDk : Colors::GraMd, -1 );

    // for ( int j = 0; j < 15; j++ ) {
    //     cv::circle( matTelemetry_, cv::Point( xStart, yStart + ( j * pad ) ), 3, Colors::GraMd, -1 );
    // }
    // for ( int id = 1; id <= 45; id++ ) {
    //     std::string cellRef = std::string( 1, 'A' + ( id - 1 ) % 9 ) + std::to_string( 2 + ( id - 1 ) / 9 );

    //     bool detected = isDetected( id );1
    //     bool active = ( id == kb.activeTagId );

    //     cv::Scalar fill = active ? Colors::GreBk : Colors::GraBk;
    //     cv::Scalar text = detected ? Colors::White : Colors::GraMd;

    //     AddBodyCell( std::to_string( id ), cellRef, 1, 1, "center", bodyFontSize, fill, text );
    // }
    AddBorder( "A1", 12, 8, Colors::GraMd, 2 );

    // ---- Trial logging status -----------------------------------------------
    // REC (red) = capturing; PRIMED (green) = armed, next 'r' starts capture;
    // OFF (gray) = idle.
    AddHeadingCell( "Trial Logging", "M1", 6, 1, "center", headerFontSize );
    std::string logStr = loggingActive_ ? "REC" : ( loggingPrimed_ ? "PRIMED" : "OFF" );
    cv::Scalar  logFill = loggingActive_ ? Colors::GreDk
                                         : ( loggingPrimed_ ? Colors::GreBk : Colors::GraBk );
    AddBodyCell( logStr, "M2", 6, 1, "center", bodyFontSize, logFill );
    AddBorder( "M1", 6, 2, Colors::GraMd, 2 );

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

    int      baseLine = 0;
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
                 textColor, 1, cv::LINE_4 );
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

void DisplayHandler::SetVirtualTarget( bool visible, cv::Point2i px ) {
    virtualTargetVisible_ = visible;
    virtualTargetPx_ = px;
}

void DisplayHandler::SetTouchFingertip( bool visible, cv::Point2i px ) {
    touchFingertipVisible_ = visible;
    touchFingertipPx_ = px;
}

void DisplayHandler::SetControllerTelemetry( const ControllerTelemetry &tele ) {
    controllerTele_ = tele;
}

void DisplayHandler::SetActiveTargetPosition( bool valid, cv::Point3f posMm ) {
    targetPosValid_ = valid;
    targetPosMm_ = posMm;
}

void DisplayHandler::SetLoggingStatus( bool primed, bool active ) {
    loggingPrimed_ = primed;
    loggingActive_ = active;
}

void DisplayHandler::SetArucoStats( float detectionHz, float lagMs ) {
    arucoDetectionHz_ = detectionHz;
    arucoLagMs_ = lagMs;
}

void DisplayHandler::SetEstimatedActiveTarget( bool visible, int tagId,
                                               const std::array<cv::Point2f, 4> &corners ) {
    estTargetVisible_ = visible;
    estTargetTagId_ = tagId;
    estTargetCorners_ = corners;
}

void DisplayHandler::SetTouchedTargetBox( bool visible, const std::array<cv::Point2f, 4> &corners ) {
    touchedBoxVisible_ = visible;
    touchedBoxCorners_ = corners;
}

void DisplayHandler::SetObjectOverlays( bool visible, const std::vector<ObjectOverlay> &overlays ) {
    objectOverlaysVisible_ = visible;
    objectOverlays_        = overlays;
}

void DisplayHandler::SetObjectStatusLine( bool visible, const std::string &text ) {
    objectStatusVisible_ = visible;
    objectStatusLine_    = text;
}

void DisplayHandler::SetWorldMarkerOutlines( bool visible,
                                             const std::vector<std::array<cv::Point2f, 4>> &outlines ) {
    worldOutlinesVisible_ = visible;
    worldOutlines_        = outlines;
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

    // Format float helper
    auto FmtFloat = []( float v, uint8_t p ) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision( p ) << v;
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
    cv::Scalar  amplifierColor;
    if ( !controllerTele_.outputEnabled ) {
        amplifierStateStr = "INACTIVE";
        amplifierColor = Colors::GraBk;
    } else if ( controllerTele_.isTargetActive && controllerTele_.guidanceOutputEnabled ) {
        amplifierStateStr = "GUIDING";
        amplifierColor = Colors::GreMd;
    } else {
        amplifierStateStr = "PAUSED";
        amplifierColor = Colors::GreBk;
    }

    // States
    AddControllerSubheadingCell( "PC State", "A2", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Cam/ArU/Lag", "A3", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Serial", "A4", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Teensy State", "A5", 4, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Amplifier State", "A6", 4, 1, "center", bodyFontSize );
    AddControllerBodyCell( systemStateStr, "E2", 4, 1, "center", bodyFontSize,
                           kb.systemState == SystemState::IDLE ? Colors::RedBk : Colors::GreBk );    // PC program state

    AddControllerBodyCell( std::to_string( static_cast<int>( measuredFreqHz_ ) ), "E3", 1, 1, "center", tableFontSize,
                           measuredFreqHz_ >= 60.0f ? Colors::GreBk : Colors::RedBk );    // Camera capture Hz
    AddControllerBodyCell( std::to_string( static_cast<int>( arucoDetectionHz_ ) ), "F3", 1, 1, "center", tableFontSize,
                           arucoDetectionHz_ >= 20.0f ? Colors::GreBk : Colors::YelBk );    // ArUco detect Hz
    AddControllerBodyCell( FmtFloat( arucoLagMs_, 0 ) + " ms", "G3", 2, 1, "center", tableFontSize,
                           arucoLagMs_ < 20.0f ? Colors::GreBk : Colors::RedBk );    // Detection lag

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
                                                                                                                                                                                        // AddControllerBodyCell( cal3Complete_ ? "SET" : "Not Set", "M6", 3, 1, "center", bodyFontSize, cal3Complete_ ? Colors::GreBk : Colors::RedBk );                                      // Fingertip-to-camera offset computed
    AddControllerBodyCell( cal3Complete_ ? FmtFloat( cal3Offset_.x, 1 ) : "-", "M6", 1, 1, "center", 0.3f, cal3Complete_ ? Colors::GreBk : Colors::RedBk );                             // Fingertip-to-camera offset computed
    AddControllerBodyCell( cal3Complete_ ? FmtFloat( cal3Offset_.y, 1 ) : "-", "N6", 1, 1, "center", 0.3f, cal3Complete_ ? Colors::GreBk : Colors::RedBk );                             // Fingertip-to-camera offset computed
    AddControllerBodyCell( cal3Complete_ ? FmtFloat( cal3Offset_.z, 1 ) : "-", "O6", 1, 1, "center", 0.3f, cal3Complete_ ? Colors::GreBk : Colors::RedBk );                             // Fingertip-to-camera offset computed

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

    AddControllerBorder( "A1", 15, 6, Colors::GraMd, 2 );

    // --- Target Telemetry Panel ----------------------------------------------------------------------------
    AddControllerHeadingCell( "Target Telemetry / Error Vector", "A7", 15, 1, "center", headerFontSize );
    AddControllerSubheadingCell( "", "A8", 5, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "x", "F8", 2, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "y", "H8", 2, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "z", "J8", 2, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Rxy", "L8", 2, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "Rxyz", "N8", 2, 1, "center", bodyFontSize );
    // Active-target position (pos_camera). Resolved in main.cpp: the directly
    // detected marker when visible, otherwise the board-pose estimate from the
    // coarse markers (far) or neighbouring fine markers (near), so the readout
    // tracks the target even when its own marker is dropped out.
    const bool haveTarget = targetPosValid_;

    // Position
    AddControllerSubheadingCell( "Position [mm]", "A9", 5, 1, "center", bodyFontSize );
    if ( haveTarget ) {
        const float x = targetPosMm_.x;
        const float y = targetPosMm_.y;
        const float z = targetPosMm_.z;
        AddControllerBodyCell( FmtFloat( x, 1 ), "F9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( FmtFloat( y, 1 ), "H9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( FmtFloat( z, 1 ), "J9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( FmtFloat( std::sqrt( x * x + y * y ), 1 ), "L9", 2, 1, "center", bodyFontSize );            // Rxy
        AddControllerBodyCell( FmtFloat( std::sqrt( x * x + y * y + z * z ), 1 ), "N9", 2, 1, "center", bodyFontSize );    // Rxyz
    } else {
        AddControllerBodyCell( "--", "F9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "H9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "J9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "L9", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "N9", 2, 1, "center", bodyFontSize );
    }

    // Guiding Position - the displacement Δp = pos_target_3d − pos_fingertip [mm]
    // computed by ControllerHandler's fingertip-to-target transform. This is the
    // move that lands the fingertip on the target (the error to the actual guided
    // target, offsets/roll compensation included). Δp.z is the depth from the
    // FINGERTIP to the target (target depth minus the Cal3 standoff), so it is
    // smaller than the camera-to-target depth shown in Position above.
    AddControllerSubheadingCell( "Guiding Pos [mm]", "A10", 5, 1, "center", bodyFontSize );
    if ( haveTarget ) {
        const cv::Point3f dp = controllerTele_.displacement;
        AddControllerBodyCell( FmtFloat( dp.x, 1 ), "F10", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( FmtFloat( dp.y, 1 ), "H10", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( FmtFloat( dp.z, 1 ), "J10", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( FmtFloat( std::sqrt( dp.x * dp.x + dp.y * dp.y ), 1 ), "L10", 2, 1, "center", bodyFontSize );                  // Rxy
        AddControllerBodyCell( FmtFloat( std::sqrt( dp.x * dp.x + dp.y * dp.y + dp.z * dp.z ), 1 ), "N10", 2, 1, "center", bodyFontSize );    // Rxyz
    } else {
        AddControllerBodyCell( "--", "F10", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "H10", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "J10", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "L10", 2, 1, "center", bodyFontSize );
        AddControllerBodyCell( "--", "N10", 2, 1, "center", bodyFontSize );
    }

    // // Velocity - virtual fingertip velocity (low-pass filtered), no z-component
    // AddControllerSubheadingCell( "Velocity [mm/s]", "A10", 5, 1, "center", bodyFontSize );
    // AddControllerBodyCell( fmtMm( controllerTele_.vel_virtual.x ), "F10", 2, 1, "center", bodyFontSize );
    // AddControllerBodyCell( fmtMm( controllerTele_.vel_virtual.y ), "H10", 2, 1, "center", bodyFontSize );
    // AddControllerBodyCell( "--", "J10", 2, 1, "center", bodyFontSize );
    // AddControllerBodyCell( fmtMm( std::sqrt( controllerTele_.vel_virtual.x * controllerTele_.vel_virtual.x + controllerTele_.vel_virtual.y * controllerTele_.vel_virtual.y ) ), "L10", 2, 1, "center", bodyFontSize );
    // AddControllerBodyCell( "--", "N10", 2, 1, "center", bodyFontSize );

    // Accumulated Error - PID integral term (accumulated position error), no z-component
    AddControllerSubheadingCell( "Accumulated [mm]", "A11", 5, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.posErrorIntegral.x, 1 ), "F11", 2, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.posErrorIntegral.y, 1 ), "H11", 2, 1, "center", bodyFontSize );
    AddControllerBodyCell( "--", "J11", 2, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( std::sqrt( controllerTele_.posErrorIntegral.x * controllerTele_.posErrorIntegral.x + controllerTele_.posErrorIntegral.y * controllerTele_.posErrorIntegral.y ), 1 ), "L11", 2, 1, "center", bodyFontSize );
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
    AddControllerBodyCell( FmtFloat( tensionRowN.x, 2 ), "G14", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( FmtFloat( tensionRowN.y, 2 ), "J14", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( FmtFloat( tensionRowN.z, 2 ), "M14", 3, 1, "center", tableFontSize );
    AddControllerSubheadingCell( "PWM", "D15", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( 2048 - tensionRowPwm.x ) ), "G15", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( 2048 - tensionRowPwm.y ) ), "J15", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( 2048 - tensionRowPwm.z ) ), "M15", 3, 1, "center", tableFontSize );

    // Force - per-motor tension/PWM contribution from the guidance force command
    AddControllerSubheadingCell( "Force", "A16", 3, 2, "center", bodyFontSize );
    AddControllerSubheadingCell( "N", "D16", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.deflectionForce.x, 2 ), "G16", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.deflectionForce.y, 2 ), "J16", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.deflectionForce.z, 2 ), "M16", 3, 1, "center", tableFontSize );
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
    AddControllerBodyCell( FmtFloat( controllerTele_.stiffnessGain, 2 ), "G18", 9, 1, "center", tableFontSize,
                           !controllerTele_.stiffnessValid        ? Colors::RedBk
                           : controllerTele_.stiffnessGainEnabled ? Colors::GreBk
                                                                  : Colors::OraBk );

    // Gain kP - custom-tuned proportional gain per motor, seeded from
    // cfg_.gain_kP and adjustable via 'G' (the other term in kP_effective).
    AddControllerSubheadingCell( "Gain kP", "A19", 3, 1, "center", bodyFontSize );
    AddControllerSubheadingCell( "N/mm", "D19", 3, 1, "center", tableFontSize );
    // Cell color: green when this motor's gain is contributing to
    // kP_effective, black (default) when it's zero.
    AddControllerBodyCell( FmtFloat( controllerTele_.gainTune.x, 2 ), "G19", 3, 1, "center", tableFontSize,
                           controllerTele_.gainTune.x > 0.0f ? Colors::GreBk : Colors::Black );
    AddControllerBodyCell( FmtFloat( controllerTele_.gainTune.y, 2 ), "J19", 3, 1, "center", tableFontSize,
                           controllerTele_.gainTune.y > 0.0f ? Colors::GreBk : Colors::Black );
    AddControllerBodyCell( FmtFloat( controllerTele_.gainTune.z, 2 ), "M19", 3, 1, "center", tableFontSize,
                           controllerTele_.gainTune.z > 0.0f ? Colors::GreBk : Colors::Black );

    // Gain kI - custom-tuned integral gain per motor, seeded from cfg_.gain_kI
    // and adjustable via 'I'. Forms kI_effective for the Stage 1 endgame
    // integrator. Cell color: green when this motor's integral gain is active
    // (> 0), black (default) when zero.
    AddControllerSubheadingCell( "Gain kI", "A20", 3, 2, "center", bodyFontSize );
    AddControllerSubheadingCell( "N/mm*s", "D20", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.iGainTune.x, 2 ), "G20", 3, 1, "center", tableFontSize,
                           controllerTele_.iGainTune.x > 0.0f ? Colors::GreBk : Colors::Black );
    AddControllerBodyCell( FmtFloat( controllerTele_.iGainTune.y, 2 ), "J20", 3, 1, "center", tableFontSize,
                           controllerTele_.iGainTune.y > 0.0f ? Colors::GreBk : Colors::Black );
    AddControllerBodyCell( FmtFloat( controllerTele_.iGainTune.z, 2 ), "M20", 3, 1, "center", tableFontSize,
                           controllerTele_.iGainTune.z > 0.0f ? Colors::GreBk : Colors::Black );

    // PWM equivalent of the integral component's per-motor tension contribution.
    AddControllerSubheadingCell( "PWM", "D21", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( controllerTele_.integralPwm.x ) ), "G21", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( controllerTele_.integralPwm.y ) ), "J21", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( std::to_string( static_cast<int>( controllerTele_.integralPwm.z ) ), "M21", 3, 1, "center", tableFontSize );

    // Output - T_output = T_preload + T_deflection, clamped to [T_preload, tension_output_max]
    AddControllerSubheadingCell( "Output", "A22", 3, 2, "center", bodyFontSize );
    AddControllerSubheadingCell( "N", "D22", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.outputTension.x, 2 ), "G22", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.outputTension.y, 2 ), "J22", 3, 1, "center", tableFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.outputTension.z, 2 ), "M22", 3, 1, "center", tableFontSize );

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
    //     AddControllerBodyCell(FmtFloat(q_abs[i]),          "B" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(FmtFloat(q_home[i]),         "D" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(fmtMm3(r_eff[i] * 1000.0f), "F" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(FmtFloat(dL[i] * 1000.0f),  "H" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(FmtFloat(tension[i]),       "J" + row, 2, 1, "center", tableFontSize);
    //     AddControllerBodyCell(FmtFloat(current[i]),       "L" + row, 2, 1, "center", tableFontSize);
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

    //     std::string kStr = controllerTele_.stiffnessValid ? FmtFloat(controllerTele_.stiffnessProfile[i]) : "--";
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
    AddControllerBodyCell( FmtFloat( controllerTele_.q_abs.x, 2 ), "G26", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.q_abs.y, 2 ), "J26", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.q_abs.z, 2 ), "M26", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.q_home.x, 2 ), "G27", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.q_home.y, 2 ), "J27", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.q_home.z, 2 ), "M27", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.r_eff.x * 1000.0f, 1 ), "G28", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.r_eff.y * 1000.0f, 1 ), "J28", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.r_eff.z * 1000.0f, 1 ), "M28", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.dL.x * 1000.0f, 1 ), "G29", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.dL.y * 1000.0f, 1 ), "J29", 3, 1, "center", bodyFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.dL.z * 1000.0f, 1 ), "M29", 3, 1, "center", bodyFontSize );

    AddControllerSubheadingCell( "Virtual Position", "A44", 5, 1, "center", headerFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.pos_virtual.x, 1 ), "F44", 5, 1, "center", headerFontSize );
    AddControllerBodyCell( FmtFloat( controllerTele_.pos_virtual.y, 1 ), "K44", 5, 1, "center", headerFontSize );

    // Virtual Mapping Parameters
    cv::Point2i center = cv::Point2i( 225, 1080 );
    int         radius = 180;

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
    cv::Point2i     posPx = center + cv::Point2i(
                                     static_cast<int>( controllerTele_.pos_virtual.x * kVirtualPlotPxPerMm ),
                                     static_cast<int>( -controllerTele_.pos_virtual.y * kVirtualPlotPxPerMm ) );
    cv::circle( matController_, posPx, 6, Colors::CyaMd, -1 );

    // Integral accumulation - the PID integral term (posErrorIntegral, mm*s)
    // drawn as a growing horizontal (X, red) and vertical (Y, blue) line pair
    // centered on the circle center, so the operator can watch the integrator
    // wind up during the endgame. Screen y is flipped; length is clamped to the
    // circle radius.
    {
        constexpr float kIntegralPlotPxPerMmS = 3.6f;
        int             ix = std::clamp( static_cast<int>( controllerTele_.posErrorIntegral.x * kIntegralPlotPxPerMmS ), -radius, radius );
        int             iy = std::clamp( static_cast<int>( -controllerTele_.posErrorIntegral.y * kIntegralPlotPxPerMmS ), -radius, radius );
        cv::line( matController_, center, center + cv::Point2i( ix, 0 ), Colors::YelDk, 3 );
        cv::line( matController_, center, center + cv::Point2i( 0, iy ), Colors::YelDk, 3 );
        // cv::circle( matController_, center + cv::Point2i( ix, 0 ), 4, Colors::RedMd, -1 );
        // cv::circle( matController_, center + cv::Point2i( 0, iy ), 4, Colors::BluMd, -1 );
    }

    // AROM samples - drawn live while Cal1Handler is recording
    if ( cal1Recording_ ) {
        for ( const cv::Point2f &s : cal1Samples_ ) {
            cv::Point2i samplePx = center + cv::Point2i(
                                                static_cast<int>( s.x * kVirtualPlotPxPerMm ),
                                                static_cast<int>( -s.y * kVirtualPlotPxPerMm ) );
            cv::circle( matController_, samplePx, 2, Colors::YelMd, -1 );
        }
        // Calibration angles
        for ( int i = 0; i < CONSTANT_CALIBRATION_ANGLES_COUNT; i++ ) {
            float rad = CONSTANT_CALIBRATION_ANGLES_DEG[i] * DEG_TO_RAD;
            cv::line( matController_, center,
                      center + cv::Point2i( static_cast<int>( std::cos( rad ) * radius ),
                                            static_cast<int>( -std::sin( rad ) * radius ) ),
                      Colors::MagDk, 1 );
        }
    }

    // AROM boundary polygon - drawn once Cal1Handler has computed it
    if ( cal1Boundary_.valid ) {
        constexpr int          kBoundaryPlotPoints = 72;
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
        for ( int i = 0; i < CONSTANT_CALIBRATION_ANGLES_COUNT; i++ ) {
            float      rad = CONSTANT_CALIBRATION_ANGLES_DEG[i] * DEG_TO_RAD;
            cv::Scalar color = ( i == cal2HeadingIdx_ ) ? Colors::MagMd : Colors::MagDk;
            cv::line( matController_, center,
                      center + cv::Point2i( static_cast<int>( std::cos( rad ) * radius ),
                                            static_cast<int>( -std::sin( rad ) * radius ) ),
                      color, 1 );
        }
    }

    // Stiffness profile polygon - drawn once Cal2Handler has produced K(theta).
    // Normalized so the stiffest heading reaches `radius` px; the rest scale
    // proportionally, giving a spider-chart view of K(theta).
    if ( controllerTele_.stiffnessValid ) {
        float kMax = *std::max_element( controllerTele_.stiffnessProfile.begin(),
                                        controllerTele_.stiffnessProfile.end() );
        if ( kMax > 0.0f ) {
            std::vector<cv::Point> stiffnessPts;
            stiffnessPts.reserve( CONSTANT_CALIBRATION_ANGLES_COUNT );
            for ( int i = 0; i < CONSTANT_CALIBRATION_ANGLES_COUNT; i++ ) {
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
        if ( gestureEvent_ == GestureEvent::FLICK_UP ) {
            cv::arrowedLine( matController_, center, center + cv::Point2i( 0, -80 ), Colors::GreMd, 4, cv::LINE_4, 0, 0.3 );
        } else if ( gestureEvent_ == GestureEvent::FLICK_DOWN ) {
            cv::arrowedLine( matController_, center, center + cv::Point2i( 0, 80 ), Colors::GreMd, 4, cv::LINE_4, 0, 0.3 );
        } else if ( gestureEvent_ == GestureEvent::CONFIRM ) {
            cv::circle( matController_, center, 60, Colors::GreMd, 4, cv::LINE_4 );
        }
    }

    // Border
    AddControllerBorder( "A24", 15, 21, Colors::GraMd, 2 );
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

    int      baseLine = 0;
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
                 textColor, 1, cv::LINE_4 );
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
    // Top-left label: what the PID error is currently driving toward.
    // Before Cal3: the offset terms cancel, so error = marker position only.
    // After Cal3:  roll-compensated offset shifts the error target.
    std::string targetLabel;
    if ( activeTagId <= 0 ) {
        targetLabel = "Target: None";
    } else if ( cal3Complete_ ) {
        targetLabel = "Target: Marker Center + Cal3 Offset";
    } else {
        targetLabel = "Target: Marker Center";
    }
    // Black shadow then white text for readability on any camera background.
    cv::putText( frame, targetLabel, cv::Point( 11, 23 ), cv::FONT_HERSHEY_SIMPLEX,
                 0.55, cv::Scalar( 0, 0, 0 ), 3 );
    cv::putText( frame, targetLabel, cv::Point( 10, 22 ), cv::FONT_HERSHEY_SIMPLEX,
                 0.55, Colors::White, 1 );

    bool activeDrawn = false;
    for ( const auto &m : markers ) {
        // Every detected tag: just a small green dot at its center. With the
        // dense multi-scale Fitts board this keeps the view readable.
        // cv::circle( frame, m.centerPx, 2, cv::Scalar( 0, 200, 0 ), cv::FILLED );

        if ( activeTagId > 0 && m.id == activeTagId ) {
            // Active tag: add the green outline square + ID label on top.
            std::vector<cv::Point> corners( 4 );
            for ( int k = 0; k < 4; k++ )
                corners[k] = cv::Point( static_cast<int>( m.cornersPx[k].x ),
                                        static_cast<int>( m.cornersPx[k].y ) );
            cv::polylines( frame, corners, true, cv::Scalar( 0, 255, 0 ), 2 );
            cv::putText( frame, "ID " + std::to_string( m.id ),
                         m.centerPx + cv::Point2i( 10, -10 ), cv::FONT_HERSHEY_SIMPLEX,
                         0.6, cv::Scalar( 0, 255, 0 ), 2 );

            // Green line from the camera centre (principal point) to the guiding
            // position (green dot = where the camera should be steered so the
            // fingertip lands on the target). Before Cal3 / when no virtual target
            // is set, fall back to the marker centre.
            cv::Point2i lineTo = ( cal3Complete_ && virtualTargetVisible_ )
                                     ? virtualTargetPx_
                                     : m.centerPx;
            cv::line( frame, principalPoint_, lineTo, Colors::GreMd, 2 );
            activeDrawn = true;
        }
    }

    // Target marker not directly detected but estimated from the board pose
    // (coarse markers far away / neighbours up close): draw the same green box /
    // ID / guidance line at the estimated outline so the operator keeps the cue.
    if ( !activeDrawn && estTargetVisible_ && estTargetTagId_ > 0 ) {
        std::vector<cv::Point> corners( 4 );
        cv::Point2f            centerF( 0.f, 0.f );
        for ( int k = 0; k < 4; k++ ) {
            corners[k] = cv::Point( static_cast<int>( estTargetCorners_[k].x ),
                                    static_cast<int>( estTargetCorners_[k].y ) );
            centerF += estTargetCorners_[k];
        }
        cv::Point2i center( static_cast<int>( centerF.x / 4.f ),
                            static_cast<int>( centerF.y / 4.f ) );

        cv::polylines( frame, corners, true, cv::Scalar( 0, 255, 0 ), 2 );
        cv::putText( frame, "ID " + std::to_string( estTargetTagId_ ),
                     center + cv::Point2i( 10, -10 ), cv::FONT_HERSHEY_SIMPLEX,
                     0.6, cv::Scalar( 0, 255, 0 ), 2 );

        cv::Point2i lineTo = ( cal3Complete_ && virtualTargetVisible_ )
                                 ? virtualTargetPx_
                                 : center;
        cv::line( frame, principalPoint_, lineTo, Colors::GreMd, 2 );
    }

    // Magenta reference box: the target marker that was active at the moment of
    // the trial-ending touch, frozen until the next target loads.
    if ( touchedBoxVisible_ ) {
        std::vector<cv::Point> corners( 4 );
        for ( int k = 0; k < 4; k++ )
            corners[k] = cv::Point( static_cast<int>( touchedBoxCorners_[k].x ),
                                    static_cast<int>( touchedBoxCorners_[k].y ) );
        cv::polylines( frame, corners, true, Colors::MagMd, 2 );
    }
}

void DisplayHandler::DrawObjectOverlays( cv::Mat &frame ) {
    // OBJECTS mode only - every other state passes visible=false so the operator
    // view is unchanged. All points are already projected to operator-view pixels
    // by WorldObjectHandler; here we only draw. Colours mirror the ArUcoTest
    // prototype: green while the object marker is directly detected, yellow when
    // the object is drawn from its world anchor (its own marker occluded).
    if ( !objectOverlaysVisible_ ) return;

    // Rig-diagnostic status line (world markers / pose / active-object state),
    // top-left under the "Target:" label. Black shadow then white for contrast.
    if ( objectStatusVisible_ && !objectStatusLine_.empty() ) {
        cv::putText( frame, objectStatusLine_, cv::Point( 11, 47 ), cv::FONT_HERSHEY_SIMPLEX,
                     0.55, cv::Scalar( 0, 0, 0 ), 3, cv::LINE_4 );
        cv::putText( frame, objectStatusLine_, cv::Point( 10, 46 ), cv::FONT_HERSHEY_SIMPLEX,
                     0.55, Colors::White, 1, cv::LINE_4 );
    }

    const int W = frame.cols, H = frame.rows;
    // A projected point may land far outside the frame (e.g. a marker seen at a
    // grazing angle); OpenCV clips lines fine, but reject wildly out-of-range /
    // non-finite coords so a single bad pose can never scribble across the HUD.
    auto sane = [&]( const cv::Point2f &p ) {
        return std::isfinite( p.x ) && std::isfinite( p.y ) &&
               p.x > -4 * W && p.x < 5 * W && p.y > -4 * H && p.y < 5 * H;
    };
    auto ipt = []( const cv::Point2f &p ) {
        return cv::Point( static_cast<int>( std::lround( p.x ) ),
                          static_cast<int>( std::lround( p.y ) ) );
    };

    // Faint blue outline around each detected world-board marker, so the operator
    // can see which markers are anchoring the scene. Drawn first (under the object
    // overlays). Muted blue, thin.
    if ( worldOutlinesVisible_ ) {
        
        for ( const auto &q : worldOutlines_ ) {
            bool ok = true;
            std::vector<cv::Point> poly( 4 );
            for ( int k = 0; k < 4; k++ ) {
                if ( !sane( q[k] ) ) { ok = false; break; }
                poly[k] = ipt( q[k] );
            }
            if ( ok ) cv::polylines( frame, poly, true, Colors::BluMd, 2, cv::LINE_4 );
        }
    }

    for ( const auto &o : objectOverlays_ ) {
        const cv::Scalar col = o.visible ? Colors::GreMd : Colors::YelMd;

        // Wireframe (each edge pre-culled to endpoints in front of the camera).
        for ( const auto &[a, b] : o.edges )
            if ( sane( a ) && sane( b ) )
                cv::line( frame, ipt( a ), ipt( b ), col, 1, cv::LINE_4 );

        // Marker outline when anchored (a live marker is outlined by the
        // detection overlay / active-tag box already).
        if ( o.hasOutline ) {
            bool ok = true;
            std::vector<cv::Point> poly( 4 );
            for ( int k = 0; k < 4; k++ ) {
                if ( !sane( o.outline[k] ) ) { ok = false; break; }
                poly[k] = ipt( o.outline[k] );
            }
            if ( ok ) cv::polylines( frame, poly, true, col, 1, cv::LINE_4 );
        }

        // Base-origin XYZ gizmo (general cylinders): +X red, +Y green, +Z blue.
        if ( o.hasGizmo && sane( o.gizmoO ) ) {
            if ( sane( o.gizmoX ) ) cv::line( frame, ipt( o.gizmoO ), ipt( o.gizmoX ), Colors::RedMd, 2, cv::LINE_4 );
            if ( sane( o.gizmoY ) ) cv::line( frame, ipt( o.gizmoO ), ipt( o.gizmoY ), Colors::GreMd, 2, cv::LINE_4 );
            if ( sane( o.gizmoZ ) ) cv::line( frame, ipt( o.gizmoO ), ipt( o.gizmoZ ), Colors::BluMd, 2, cv::LINE_4 );
        }

        // Guidance-target dot (magenta). For the ACTIVE object also draw the
        // guidance line from the camera principal point to the target, matching
        // the FITTS "where guidance is pulling" cue.
        if ( o.hasTargetDot && sane( o.targetDot ) ) {
            if ( o.active )
                cv::line( frame, principalPoint_, ipt( o.targetDot ), Colors::GreMd, 2, cv::LINE_4 );
            cv::circle( frame, ipt( o.targetDot ), 5, Colors::MagMd, cv::FILLED, cv::LINE_4 );
        }

        // Name label at the marker origin (black shadow + magenta text). The
        // active object is marked; anchored objects note the anchored fallback.
        if ( o.hasLabel && sane( o.labelPos ) ) {
            std::string label = o.name;
            if ( o.anchored ) label += " (memory)";
            if ( o.active )   label += " *";
            const cv::Point p = ipt( o.labelPos );
            cv::putText( frame, label, p, cv::FONT_HERSHEY_SIMPLEX, 0.75, Colors::White, 5, cv::LINE_4 );
            cv::putText( frame, label, p, cv::FONT_HERSHEY_SIMPLEX, 0.75, Colors::MagDk, 2, cv::LINE_4 );
        }
    }
}

void DisplayHandler::DrawTelemetryBar(
    cv::Mat &frame, const std::vector<DetectedMarker> &markers,
    const TouchState &touch, const KeyboardState &kb ) {
    const int   x = 10;
    const float font = 0.5f;
    const auto  col = cv::Scalar( 210, 210, 210 );

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
