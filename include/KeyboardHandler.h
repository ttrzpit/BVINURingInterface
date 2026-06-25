#pragma once

// =============================================================================
// KeyboardHandler.h - Single-keypress, table-driven input state machine
//
// Every keystroke from cv::pollKey() is processed immediately against the
// current InputState (see KeyCommandTable.h for the full key -> transition
// table, mirroring keyboard_reference.md). Numeric entries (nnn/nnnn/nn/n.n)
// are buffered and require Enter to confirm; everything else is instant.
//
// The current state and any executed actions are exposed in KeyboardState,
// which main.cpp reads each loop and passes to whichever handlers need it.
//
// Adding a new command:
//   1. Add a row to kKeyCommandTable (or kNumericEntryTable) in
//      KeyCommandTable.h.
//   2. If it needs a side effect, add a KeyAction value and handle it in
//      KeyboardHandler::ExecuteAction().
// =============================================================================

#include <cstdint>
#include <string>

// ---- Input state --------------------------------------------------------------
// Flat state machine driving single-keypress dispatch. ANY/SAME/QUIT are
// sentinels used only in KeyCommandTable rows - never assigned to
// KeyboardState::inputState.

enum class InputState {
    IDLE,
    CAL_SEL,
    CAL_ROM,
    CAL_STI,
    CAL_OFF,
    TEN_MENU,
    PRE_TENSION,
    LOG,
    LOG_UID,
    MOT_PWM,
    MOT_PWM_A,
    MOT_PWM_B,
    MOT_PWM_C,
    MOT_PWM_ALL,
    FIT_WARN,    // Calibration-incomplete confirmation prompt before FITTS
    FIT_SEL,
    FIT_RUN,
    FIT_ACT,
    TEN_SEL_ALL,
    TEN_SEL_A,
    TEN_SEL_B,
    TEN_SEL_C,
    TEN_ADJ_ALL,
    TEN_ADJ_A,
    TEN_ADJ_B,
    TEN_ADJ_C,
    GAIN_ALL,
    GAIN_A,
    GAIN_B,
    GAIN_C,
    IGAIN_ALL,
    IGAIN_A,
    IGAIN_B,
    IGAIN_C,
    // Sentinels - only valid in KeyCommand::requiredState / newState
    ANY,
    SAME,
    QUIT
};

// ---- System state -----------------------------------------------------------
// Coarse view of InputState, derived via DeriveSystemState(). Used by
// main.cpp and DisplayHandler to decide which grid/handler is active.

enum class SystemState {
    IDLE,             ///< Default - no active task, ArUco grid hidden
    CALIBRATING,      ///< General calibration - ArUco grid shown on touchscreen
    CAL3,             ///< Calibration Stage 3: camera-to-fingertip offset collection
    FITTS,            ///< Fitts task running
    PRETENSION,       ///< Guided pretensioning / encoder-zeroing / home-recording sequence
    TENSION_ADJUST    ///< Standalone tension adjustment (manual tension mode, no guided sequence)
};

/** @brief Map an InputState to its coarse SystemState for grid/handler dispatch. */
SystemState DeriveSystemState( InputState state );

/** @brief True for the gain-tuning overlay states - proportional (GAIN_*, 'P')
 *         and integral (IGAIN_*, 'I'). These states don't represent a task of
 *         their own, so SystemState and any task entry/exit logic must ignore
 *         them, letting the underlying task (FITTS, CAL_STI, etc.) keep running
 *         while gains are tuned. Switching between the P and I overlays
 *         preserves the task to return to on exit. */
bool IsGainTuneInputState( InputState state );

/** @brief True only for the integral-gain overlay states (IGAIN_*, 'I'). */
bool IsIGainTuneInputState( InputState state );

// ---- Serial connection action -----------------------------------------------
// Set by the 'S' (toggle) key; cleared by main.cpp after acting.

enum class SerialAction { NONE,
                          TOGGLE };

// ---- Key action --------------------------------------------------------------
// One-shot side effects triggered by KeyCommandTable rows / numeric entries.
// See KeyboardHandler::ExecuteAction().

enum class KeyAction {
    NONE,
    TOGGLE_SERIAL,
    SET_USER_ID,
    SET_MOTOR_PWM,
    RANDOM_FITTS_TARGET,
    SET_FITTS_TARGET,
    PRETENSION_ADVANCE,
    ADJUST_TENSION_INC,
    ADJUST_TENSION_DEC,
    SET_TENSION,
    ADJUST_GAIN_INC,
    ADJUST_GAIN_DEC,
    ADJUST_IGAIN_INC,
    ADJUST_IGAIN_DEC,
    EXIT_GAIN_MODE,
    SET_HOME_POSITION,
    SET_ROBOT_IDLE,
    SET_ROBOT_READY,
    TOGGLE_STIFFNESS_GAIN,
    TOGGLE_LOGGING,
    TOGGLE_ESTOP,
};

// ---- Motor test request -----------------------------------------------------

struct MotorTestRequest {
    bool     active = false;
    char     motor = 'A';    ///< 'A', 'B', 'C', or 'D' (all motors)
    uint16_t pwm = 2047;     ///< PWM value to apply (0=full, 2047=off)
};

// ---- Tension adjustment request (pretensioning step 3/4) --------------------

struct TensionAdjustRequest {
    bool  active = false;
    char  motor = 'A';           ///< 'A', 'B', 'C', or 'D' (all motors)
    bool  isAbsolute = false;    ///< true: set to valueN; false: nudge by deltaN
    float deltaN = 0.0f;         ///< +/- step [N], used when !isAbsolute
    float valueN = 0.0f;         ///< Absolute setpoint [N], used when isAbsolute
};

// ---- Gain tuning request (direction-dependent kP boost, 'G' key) -------------

struct GainAdjustRequest {
    bool  active = false;
    char  motor = 'A';          ///< 'A', 'B', 'C', or 'D' (all motors)
    float deltaGain = 0.0f;     ///< +/- step applied to the selected gain
    bool  isIntegral = false;   ///< true: adjust integral gain (iGainTune); false: proportional (gainTune)
};

// ---- RobotState ladder request -----------------------------------------------
// Set by the global 'e'/'E' keys; cleared by main.cpp after acting.

enum class RobotStateRequest { NONE,
                               GO_IDLE,
                               GO_READY };

// ---- Output type ------------------------------------------------------------

struct KeyboardState {
    int                  activeTagId = 0;      ///< ArUco ID to highlight (0 = none)
    int                  activeUserId = -1;    ///< User ID for study logging (-1 = not set, 000 = non-logging, 001> = valid user)
    bool                 quitRequested = false;
    InputState           inputState = InputState::IDLE;                         ///< Current single-key input state
    SystemState          systemState = SystemState::IDLE;                       ///< Coarse state, derived from inputState
    int                  fittsTargetId = 0;                                     ///< Randomly selected Fitts target (0 = none)
    int                  lastInputKey = -1;                                     ///< Raw key code of the last processed input (-1 = none yet)
    SerialAction         pendingSerialAction = SerialAction::NONE;              ///< One-shot serial toggle request
    MotorTestRequest     pendingMotorTest;                                      ///< One-shot motor PWM test request
    bool                 pendingPretensionAdvance = false;                      ///< One-shot: Enter pressed during PRETENSION
    TensionAdjustRequest pendingTensionAdjust;                                  ///< One-shot: tension setpoint adjustment (PRETENSION step 3/4)
    GainAdjustRequest    pendingGainAdjust;                                     ///< One-shot: gain tune adjustment ('G' mode)
    RobotStateRequest    pendingRobotStateRequest = RobotStateRequest::NONE;    ///< One-shot: 'e'/'E' pressed
    bool                 pendingStiffnessGainToggle = false;                    ///< One-shot: 'k' pressed (toggle K(theta) application)
    bool                 pendingSetHomePosition = false;                        ///< One-shot: 'Z' pressed (record current encoder pose as home)
    bool                 pendingRandomTarget = false;                           ///< One-shot: 'r' pressed - main picks the (distance-stratified) target
    bool                 pendingLoggingToggle = false;                          ///< One-shot: 'L' pressed - main toggles the trial logger
    bool                 pendingEStopToggle = false;                            ///< One-shot: spacebar pressed - main toggles the guidance-output e-stop
    std::string          inputBuffer;                                           ///< Numeric value currently being typed
    std::string          outputBuffer;                                          ///< Display text for the last executed command
};

// ---- Handler ----------------------------------------------------------------

class KeyboardHandler {
   public:
    KeyboardHandler();

    /**
     * @brief Ingest one raw keystroke value from cv::pollKey().
     * @param key  Raw return value from cv::pollKey() - pass it unmasked.
     *             This function handles the -1 "no key" sentinel internally.
     */
    void ProcessKey( int key );

    /** @brief Returns the current state for main.cpp to distribute. */
    const KeyboardState& GetState() const { return state_; }

    /**
     * @brief Overwrite the output buffer with a message from an external source
     *        (e.g. Cal3Handler status). Does not affect the input buffer or any
     *        other state field.
     */
    void SetExternalStatus( const std::string& msg ) { state_.outputBuffer = msg; }

    /** @brief Clear the serial action after main.cpp has acted on it. */
    void ClearSerialAction() { state_.pendingSerialAction = SerialAction::NONE; }

    /** @brief Clear the motor test request after main.cpp has acted on it. */
    void ClearMotorTest() { state_.pendingMotorTest.active = false; }

    /** @brief Clear the pretension-advance request after main.cpp has acted on it. */
    void ClearPretensionAdvance() { state_.pendingPretensionAdvance = false; }

    /** @brief Clear the tension-adjust request after main.cpp has acted on it. */
    void ClearTensionAdjust() { state_.pendingTensionAdjust.active = false; }

    /** @brief Clear the gain-tune adjust request after main.cpp has acted on it. */
    void ClearGainAdjust() { state_.pendingGainAdjust.active = false; }

    /** @brief Clear the robot-state request after main.cpp has acted on it. */
    void ClearRobotStateRequest() { state_.pendingRobotStateRequest = RobotStateRequest::NONE; }

    /** @brief Clear the stiffness-gain toggle request after main.cpp has acted on it. */
    void ClearStiffnessGainToggle() { state_.pendingStiffnessGainToggle = false; }

    /** @brief Clear the set-home-position request after main.cpp has acted on it. */
    void ClearSetHomePosition() { state_.pendingSetHomePosition = false; }

    /** @brief Clear the one-shot 'r' random-target request (main consumed it). */
    void ClearRandomTarget() { state_.pendingRandomTarget = false; }

    /** @brief Clear the one-shot 'L' logging-toggle request (main consumed it). */
    void ClearLoggingToggle() { state_.pendingLoggingToggle = false; }

    /** @brief Clear the one-shot spacebar e-stop-toggle request (main consumed it). */
    void ClearEStopToggle() { state_.pendingEStopToggle = false; }

    /**
     * @brief Force the input state directly (bypassing the key-table dispatch),
     *        re-deriving systemState. Used e.g. by PretensionHandler's
     *        ZERO->TENSION auto-transition to default into TEN_SEL_ALL.
     */
    void SetInputState( InputState state ) {
        state_.inputState = state;
        state_.systemState = DeriveSystemState( state );
    }

    /** @brief Set the active Fitts target ID directly (e.g. from a flick
     *         gesture during FITTS), clamped to the board's target range.
     *         Mirrors SET_FITTS_TARGET. */
    void SetFittsTargetId( int id ) {
        if ( id < fittsTargetIdMin_ ) id = fittsTargetIdMin_;
        if ( id > fittsTargetIdMax_ ) id = fittsTargetIdMax_;
        state_.fittsTargetId = id;
        state_.activeTagId = id;
    }

    /** @brief Configure the inclusive Fitts target ID range (the fine-marker
     *         band of the multi-scale board). Drives random target selection
     *         ('r') and the flick-step clamp. Defaults to [1, 45]. */
    void SetFittsTargetRange( int minId, int maxId ) {
        fittsTargetIdMin_ = minId;
        fittsTargetIdMax_ = maxId;
    }

    /** @brief Set the board-wide maximum marker ID (includes coarse markers).
     *         Used as the upper clamp for manual ID entry ('m' key) so coarse
     *         marker IDs are reachable. Defaults to fittsTargetIdMax_. */
    void SetFittsBoardMaxId( int maxId ) { fittsBoardMaxId_ = maxId; }

    /** @brief Report whether all three calibrations (AROM, stiffness, fingertip
     *         offset) are complete. Drives the 'F' Fitts-entry gate: if any are
     *         incomplete, 'F' diverts to a (p)roceed/(r)eturn confirmation
     *         (FIT_WARN) instead of entering the task directly. */
    void SetCalibrationsComplete( bool complete ) { calibrationsComplete_ = complete; }

   private:
    /** @brief Route digit/decimal/Backspace/Enter keys while inputState expects
     *         a numeric entry. Returns true if the key was consumed. */
    bool ProcessNumericEntry( int key );

    /** @brief Look up `key` against kKeyCommandTable for the current
     *         inputState (or ANY) and apply the matching transition. */
    void DispatchTableCommand( int key );

    /** @brief Apply the one-shot side effect (if any) for `action`. */
    void ExecuteAction( KeyAction action, int value = -1 );

    /** @brief Substitute [MARKER_ID]/[VAL]/[MOTOR] placeholders in a display
     *         text template. `contextState` is the inputState *before* any
     *         transition, used to resolve [MOTOR]. */
    std::string FormatDisplayText( const std::string& tmpl, int value,
                                   bool isDecimal, InputState contextState ) const;

    /** @brief Format a raw numeric value as text, inserting a decimal point
     *         before the last digit if `isDecimal` (e.g. 25 -> "2.5"). */
    std::string FormatNumericValue( int value, bool isDecimal ) const;

    /** @brief 'A'/'B'/'C'/'D' (D = all motors) for a MOT_PWM_* state. */
    char MotorLetterFromState( InputState state ) const;

    /** @brief Human-readable motor label ("A", "A, B, C", "All", ...) for [MOTOR]. */
    std::string MotorLabelFromState( InputState state ) const;

    KeyboardState state_;
    std::string   inputBuffer_;                        // In-progress numeric entry (mirrors state_.inputBuffer)
    InputState    preGainState_ = InputState::IDLE;    // inputState to restore on EXIT_GAIN_MODE
    int           fittsTargetIdMin_ = 1;               // Fitts target range (set from the board layout)
    int           fittsTargetIdMax_ = 45;
    int           fittsBoardMaxId_  = 45;              // Board-wide max (includes coarse) for manual entry
    bool          calibrationsComplete_ = false;       // All 3 cals done? Gates 'F' Fitts entry
};
