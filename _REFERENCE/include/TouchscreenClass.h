#pragma once

// Memory for shared data
#include <memory>

// Linux headers for reading mouse position / touch
#include <X11/Xlib.h>
#include <X11/extensions/XInput2.h>


// Forward declarations
class SystemDataManager;
struct ManagedData;



/** 
 * @brief Display class definition
 */
class TouchscreenClass {

public:
	// Constructor
	TouchscreenClass( SystemDataManager& dataHandle );

	// De-constructor
	~TouchscreenClass();

	// Public functions
	void GetCursorPosition();
	void Close();


private:
	// Private functions
	void ParseClick();
	void ProcessEvents();
	void SelectInputFromDevice( Display* display, int deviceId, Window window );
	int	 GetTouchscreenDeviceId( Display* display, const std::string& deviceName );
	void InitializeTouchEvents();

	// Private variables
	bool buttonPressedNew = false;
	bool buttonPressedOld = false;

	// Variables for pointer position
	int			 rootX, rootY, winX, winY;
	unsigned int buttonMask;
	Window		 childReturn, rootReturn, root;
	int			 xinputOpcode = -1;	   // 🔶 NEW: stores XInput2 extension opcode

	// Display handle for X11
	Display* displayHandle = XOpenDisplay( nullptr );

	// Data manager handle
	SystemDataManager&			 dataHandle;
	std::shared_ptr<ManagedData> shared;
};