#pragma once

// Memory for shared data
#include <memory>

// For key mapping
#include <functional>
#include <unordered_map>

// OpenCV core functions
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Forward declarations
class SystemDataManager;
struct ManagedData;



/** 
 * @brief Capture class definition
 */
class InputClass {

public:
	// Data manager handle
	InputClass( SystemDataManager& dataHandle );

	// Public functions
	void ParseInput( int key );

	// Register key bindings
	void RegisterKeyBindings();


private:
	// Data manager handle
	SystemDataManager&			 dataHandle;
	std::shared_ptr<ManagedData> shared;

	// Key map
	std::unordered_map<int, std::function<void()>> keyBindings;

	// Increment / decrement
	void AdjustValue( float& target, float step, float min, float max );
	void Increment( float& target, float step, float max );
	void Decrement( float& target, float step, float min );

	void AdjustGain( float dir );
	void AdjustGainP( float step, float min, float max );
	void AdjustGainI( float step, float min, float max );
	void AdjustGainD( float step, float min, float max );
	void AdjustTension( float dir );
	void AdjustLimits( float dir );
	void PrintStates();


	// Functions
	void IncrementValueF( std::string name, float& target, float increment );


	// Keymapped functions
	void K_Exit();
	void K_SetActive1();
	void K_SetActive2();
	void K_SetActive3();
	void K_SetActive4();
	void K_SetActive5();
	void K_SetActiveNone();
	void K_AmplifierToggle();
	void K_AmplifierTensionToggle();
	void K_SerialToggle();
	void K_DirSelect_Abduction();
	void K_DirSelect_Adduction();
	void K_DirSelect_Flexion();
	void K_DirSelect_Extension();
	void K_GainSelect_Proportional();
	void K_GainSelect_Integral();
	void K_GainSelect_Derivative();
	void K_GainsZero();
	void K_TenSelect_A();
	void K_TenSelect_B();
	void K_TenSelect_C();
	void K_Increment();
	void K_Decrement();
	void K_EncoderZero();
	void K_EncoderMeasureLimit();
	void K_EncoderSetLimit();
	void K_FittsStart();
	void K_FittsStop();
	void K_TaskCalibrationStart();
	void K_LimitsSelectAll();
	void K_LimitsSelectA();
	void K_LimitsSelectB();
	void K_LimitsSelectC();
	void K_LimitsReset();
	void K_DeselectAdjustment();
	void K_SetReverseType();
	void K_RotateCamera();
};
