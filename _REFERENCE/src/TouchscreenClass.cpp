// Call to class header
#include "TouchscreenClass.h"
#include <cstring>	  // for strcmp

// System data manager
#include "SystemDataManager.h"


// Constructor
TouchscreenClass::TouchscreenClass( SystemDataManager& ctx )
	: dataHandle( ctx )
	, shared( ctx.getData() ) {

	std::cout << "TouchClass:   Touchscreen initialized.\n";

	if ( displayHandle ) {
		InitializeTouchEvents();	// 🔶 NEW
	}


	// Old version (working)
	// if ( displayHandle ) {
	// 	// Select input events for button press, release, and pointer motion on the root window
	// 	XSelectInput( displayHandle, DefaultRootWindow( displayHandle ), ButtonPressMask | ButtonReleaseMask | PointerMotionMask );

	// 	// [NEW CODE] Grab the pointer to ensure our application receives all pointer events.
	// 	int grabStatus = XGrabPointer( displayHandle, DefaultRootWindow( displayHandle ), True, ButtonPressMask | ButtonReleaseMask | PointerMotionMask, GrabModeAsync, GrabModeAsync, None, None, CurrentTime );
	// 	if ( grabStatus != GrabSuccess ) {
	// 		std::cerr << "TouchClass: XGrabPointer failed with status " << grabStatus << "\n";
	// 	}
	// }
}

/**
 * @brief Deconstructor
 */
TouchscreenClass::~TouchscreenClass() {
	XUngrabPointer( displayHandle, CurrentTime );	 // [NEW CODE] Release the pointer grab.
	XCloseDisplay( displayHandle );
	std::cout << "TouchClass:   X11 shutdown complete.\n";
}

/**
 * @brief Get cursor position and click
 */
void TouchscreenClass::GetCursorPosition() {
	// Call event-processing loop
	ProcessEvents();
}

/**
 * @brief Close the touchscreen class (to end X11 polling)
 */
void TouchscreenClass::Close() {
	// Close X11 handle
	XCloseDisplay( displayHandle );
}

void TouchscreenClass::ParseClick() {
	if ( buttonPressedNew == true ) {
		// Switch-case for active task
		switch ( shared->Task.repetitionNumber ) {
			case 0:
				// Nothing
				break;
			case 1:
				// Discrimination
				break;
			case 2:	   // Fitts
				if ( shared->Touchscreen.isTouched == 1 ) {
					shared->Task.command = 'c';
				} else {
					shared->Task.command = 0;
				}
				break;
			case 3:
				//
				break;
			case 4:
				//
				break;
			default:
				//
				break;
		}
		buttonPressedNew = false;
	}
}

void TouchscreenClass::ProcessEvents() {

	while ( XPending( displayHandle ) ) {
		XEvent event;
		XNextEvent( displayHandle, &event );

		if ( event.type == GenericEvent && event.xcookie.extension == xinputOpcode ) {
			XGetEventData( displayHandle, &event.xcookie );
			// 🔶 NEW: Process XInput2 touch events
			if ( event.xcookie.evtype == XI_TouchBegin || event.xcookie.evtype == XI_TouchUpdate || event.xcookie.evtype == XI_TouchEnd ) {
				XIDeviceEvent* xie = ( XIDeviceEvent* )event.xcookie.data;
				// std::cout << "Touch ID " << xie->detail << " @ (" << xie->event_x << ", " << xie->event_y << ")\n";

				shared->Touchscreen.positionTouched.x = xie->event_x - 3440;
				shared->Touchscreen.positionTouched.y = xie->event_y - 32;

				// Optional: update Z state
				if ( event.xcookie.evtype == XI_TouchBegin ) {	  // Touch begin
					// shared->touchPosition.z = 1;
					shared->Touchscreen.isTouched = true;
				} else if ( event.xcookie.evtype == XI_TouchEnd ) {	   // Touch end
					// shared->touchPosition.z = 0;
					shared->Touchscreen.isTouched = false;
				}
				// shared->touchPosition.z = ( event.xcookie.evtype == XI_TouchEnd ) ? 0 : 1;
			}
			XFreeEventData( displayHandle, &event.xcookie );
		}
	}


	ParseClick();
}


void TouchscreenClass::SelectInputFromDevice( Display* display, int deviceId, Window window ) {

	XIEventMask	  eventmask;
	unsigned char mask[( XI_LASTEVENT + 7 ) / 8] = { 0 };

	eventmask.deviceid = deviceId;
	eventmask.mask_len = sizeof( mask );
	eventmask.mask	   = mask;

	XISetMask( mask, XI_ButtonPress );
	XISetMask( mask, XI_ButtonRelease );
	XISetMask( mask, XI_Motion );

	int result = XISelectEvents( display, window, &eventmask, 1 );
	if ( result != Success ) {
		std::cerr << "TouchClass:   Failed to select events from device ID " << deviceId << "\n";
	}
}


int TouchscreenClass::GetTouchscreenDeviceId( Display* display, const std::string& deviceName ) {
	int			  nDevices;
	XIDeviceInfo* devices = XIQueryDevice( display, XIAllDevices, &nDevices );

	int foundId = -1;
	for ( int i = 0; i < nDevices; ++i ) {
		const char* name = devices[i].name;
		if ( name && deviceName == name ) {
			foundId = devices[i].deviceid;
			break;
		}
	}

	XIFreeDeviceInfo( devices );
	return foundId;
}
// 🔶 NEW FUNCTION TO INITIALIZE TOUCH EVENTS
void TouchscreenClass::InitializeTouchEvents() {
	int opcode, event, error;
	if ( !XQueryExtension( displayHandle, "XInputExtension", &opcode, &event, &error ) ) {
		std::cerr << "TouchClass:   XInput extension not available\n";
		return;
	}
	xinputOpcode = opcode;	  // 🔶 NEW: store for event checking


	int major = 2, minor = 2;
	if ( XIQueryVersion( displayHandle, &major, &minor ) != Success || ( major * 1000 + minor ) < 2002 ) {
		std::cerr << "TouchClass:   XInput2.2 not supported\n";
		return;
	}

	Window rootWindow = DefaultRootWindow( displayHandle );

	XIEventMask evmask;
	memset( &evmask, 0, sizeof( evmask ) );
	evmask.deviceid = XIAllDevices;	   // or use your touchscreen device ID

	int			   maskLen = XIMaskLen( XI_TouchEnd );
	unsigned char* mask	   = ( unsigned char* )calloc( maskLen, sizeof( char ) );
	evmask.mask_len		   = maskLen;
	evmask.mask			   = mask;

	XISetMask( mask, XI_TouchBegin );
	XISetMask( mask, XI_TouchUpdate );
	XISetMask( mask, XI_TouchEnd );

	if ( XISelectEvents( displayHandle, rootWindow, &evmask, 1 ) != Success ) {
		std::cerr << "TouchClass:   Failed to select XI2 touch events\n";
	} else {
		std::cout << "TouchClass:   XInput2 touch events registered\n";
	}

	XFlush( displayHandle );
	free( mask );
}
