#pragma once

// =============================================================================
// TouchHandler.h - XInput2 touchscreen event reader
//
// Registers for XInput2 touch events on the X11 root window and drains the
// event queue each time getLatestTouch() is called. The call is non-blocking:
// it processes whatever events are pending and returns the current touch state.
//
// Touch coordinates are adjusted by the configured x/y offset so that the
// returned position is in touchscreen-local space (0,0 = top-left of the
// touchscreen, regardless of the monitor layout).
// =============================================================================

#include <mutex>

#include <X11/Xlib.h>
#include <X11/extensions/XInput2.h>

#include <opencv2/core.hpp>    // cv::Point2i

#include "Config.h"


// ---- Output type ------------------------------------------------------------

/** @brief Current state of the touchscreen. */
struct TouchState {
    cv::Point2i position  = { 0, 0 };  ///< Touch position in touchscreen-local coordinates
    bool        isTouched = false;      ///< True while a finger is in contact with the screen
};


// ---- Handler ----------------------------------------------------------------

class TouchHandler {
public:
    /**
     * @param cfg  Touchscreen config - specifically xOffset and yOffset are
     *             used to convert desktop-space coordinates to screen-local ones.
     *             Must outlive this object.
     */
    explicit TouchHandler(const TouchscreenConfig& cfg);

    /** @brief Closes the X11 display handle. */
    ~TouchHandler();

    /**
     * @brief Drain pending X11 touch events and return the current touch state.
     *        Non-blocking. Should be called once per main loop iteration.
     */
    TouchState getLatestTouch();

private:
    void initXInput();  // Register for XI_TouchBegin / Update / End events

    const TouchscreenConfig& cfg_;

    Display* xDisplay_     = nullptr;
    int      xinputOpcode_ = -1;

    TouchState currentTouch_;  // Updated in getLatestTouch()
    bool       prevIsTouched_ = false;  // For debug transition logging
};
