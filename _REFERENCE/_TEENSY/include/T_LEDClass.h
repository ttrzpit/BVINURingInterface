/** LED Class **/

#pragma once

// Memory for shared data
#include "T_Config.h"
#include <Arduino.h>
#include <memory>

// System data manager
#include "T_SharedDataManagerClass.h"
class SharedDataManager;
struct ManagedData;



/** 
 * @brief Amplifier  class definition
 */
class T_LEDClass {

public:
	// Default constructor
	T_LEDClass( SharedDataManager& dataHandle );

	// Public functions
	void Begin();
	void Update();


private:
	// Data manager handle
	SharedDataManager&			 dataHandle;
	std::shared_ptr<ManagedData> shared;
};