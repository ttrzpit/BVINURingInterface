#include "TouchHandler.h"

#include <cstdlib>    // calloc, free
#include <cstring>    // memset
#include <iostream>

// =============================================================================
// TouchHandler.cpp — XInput2 touchscreen event reader
//
// Uses the X11 XInput2 extension to receive raw touch events from the
// touchscreen, independent of which application has focus. This is the same
// approach as the reference code but restructured to return a plain TouchState
// rather than writing directly into a shared data structure.
//
// The raw X event coordinates are in desktop space. Subtracting cfg_.xOffset
// and cfg_.yOffset converts them to touchscreen-local space.
// =============================================================================


TouchHandler::TouchHandler(const TouchscreenConfig& cfg) : cfg_(cfg) {

    xDisplay_ = XOpenDisplay(nullptr);
    if (!xDisplay_) {
        std::cerr << "TouchHandler: Could not open X11 display.\n";
        return;
    }

    initXInput();
    std::cout << "TouchHandler: Initialized.\n";
}

TouchHandler::~TouchHandler() {
    if (xDisplay_) {
        XCloseDisplay(xDisplay_);
        std::cout << "TouchHandler: X11 display closed.\n";
    }
}


// ---- Public -----------------------------------------------------------------

TouchState TouchHandler::getLatestTouch() {

    if (!xDisplay_) return currentTouch_;

    // Drain all pending events in one call — we want the most current state
    // without blocking waiting for future events.
    while (XPending(xDisplay_)) {

        XEvent event;
        XNextEvent(xDisplay_, &event);

        // Only handle XInput2 generic events
        if (event.type != GenericEvent)                          continue;
        if (event.xcookie.extension != xinputOpcode_)           continue;
        if (!XGetEventData(xDisplay_, &event.xcookie))          continue;

        int evtype = event.xcookie.evtype;
        bool isTouchEvent = (evtype == XI_TouchBegin  ||
                             evtype == XI_TouchUpdate ||
                             evtype == XI_TouchEnd);

        if (isTouchEvent) {
            auto* xie = static_cast<XIDeviceEvent*>(event.xcookie.data);

            // Convert from desktop coordinates to touchscreen-local coordinates
            currentTouch_.position.x = static_cast<int>(xie->event_x) - cfg_.xOffset;
            currentTouch_.position.y = static_cast<int>(xie->event_y) - cfg_.yOffset;

            // isTouched is true for Begin and Update, false when finger lifts
            currentTouch_.isTouched = (evtype != XI_TouchEnd);
        }

        XFreeEventData(xDisplay_, &event.xcookie);
    }

    return currentTouch_;
}


// ---- Private ----------------------------------------------------------------

void TouchHandler::initXInput() {

    // Verify that the XInput extension is available
    int event, error;
    if (!XQueryExtension(xDisplay_, "XInputExtension", &xinputOpcode_, &event, &error)) {
        std::cerr << "TouchHandler: XInput extension not available.\n";
        return;
    }

    // We require XInput 2.2+ for multi-touch support
    int major = 2, minor = 2;
    if (XIQueryVersion(xDisplay_, &major, &minor) != Success ||
        (major * 1000 + minor) < 2002)
    {
        std::cerr << "TouchHandler: XInput 2.2 or higher required.\n";
        return;
    }

    Window rootWindow = DefaultRootWindow(xDisplay_);

    // Build an event mask requesting touch begin, update, and end events
    int            maskLen = XIMaskLen(XI_TouchEnd);
    unsigned char* mask    = static_cast<unsigned char*>(calloc(maskLen, sizeof(char)));

    XIEventMask evmask;
    evmask.deviceid = XIAllDevices;   // Receive events from all input devices
    evmask.mask_len = maskLen;
    evmask.mask     = mask;

    XISetMask(mask, XI_TouchBegin);
    XISetMask(mask, XI_TouchUpdate);
    XISetMask(mask, XI_TouchEnd);

    if (XISelectEvents(xDisplay_, rootWindow, &evmask, 1) != Success) {
        std::cerr << "TouchHandler: Failed to register XInput2 touch events.\n";
    } else {
        std::cout << "TouchHandler: XInput2 touch events registered.\n";
    }

    XFlush(xDisplay_);
    free(mask);

    // Map the touchscreen input device to the correct monitor so that touch
    // coordinates align with the display. Equivalent to running manually:
    //   xinput map-to-output <device_id> <output>
    // Device ID and output name are set in config.yaml [touchscreen].
    std::string cmd = "xinput map-to-output "
                    + std::to_string(cfg_.xinputDeviceId)
                    + " " + cfg_.xinputOutput;
    int result = system(cmd.c_str());
    if (result != 0)
        std::cerr << "TouchHandler: xinput map-to-output failed (exit " << result
                  << ") — check xinput_device_id and xinput_output in config.yaml\n";
    else
        std::cout << "TouchHandler: Touchscreen mapped to " << cfg_.xinputOutput << "\n";
}
