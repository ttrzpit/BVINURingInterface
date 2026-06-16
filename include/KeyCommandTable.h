#pragma once

// =============================================================================
// KeyCommandTable.h - Data tables for KeyboardHandler's single-key FSM
//
// Mirrors keyboard_reference.md 1:1, grouped by section. kKeyCommandTable
// covers immediate single-keypress commands (requiredState == ANY means the
// row applies regardless of the current InputState). kNumericEntryTable
// covers the Enter-confirmed numeric entries (nnn/nnnn/nn/n.n).
//
// Adding a command: add one row here, then (if it needs a side effect) add a
// KeyAction value and handle it in KeyboardHandler::ExecuteAction().
// =============================================================================

#include <string>
#include <vector>

#include "KeyboardHandler.h"  // InputState, KeyAction


struct KeyCommand {
    std::vector<int> keys;       ///< ASCII codes; multiple = alternate bindings (e.g. numpad)
    InputState requiredState;    ///< Required InputState, or InputState::ANY
    InputState newState;         ///< New InputState, or InputState::SAME / InputState::QUIT
    std::string displayText;     ///< May contain [MARKER_ID] / [VAL] / [MOTOR]
    KeyAction action;
};

inline const std::vector<KeyCommand> kKeyCommandTable = {
    // ---- SYSTEM ---------------------------------------------------------------
    { {27}, InputState::ANY, InputState::QUIT, "Exiting.", KeyAction::NONE },
    { {32}, InputState::ANY, InputState::IDLE, "Input cleared.", KeyAction::NONE },
    { {96}, InputState::ANY, InputState::IDLE, "System cleared, returning to IDLE state.", KeyAction::NONE },

    // ---- GUIDANCE OUTPUT TOGGLE ---------------------------------------------------
    // 'e'/'E' work regardless of the current InputState; the real outcome text
    // is set via SetExternalStatus() by main.cpp once it is known.
    { {'e'}, InputState::ANY, InputState::SAME, "Disabling guidance output...", KeyAction::SET_ROBOT_IDLE },
    { {'E'}, InputState::ANY, InputState::SAME, "Enabling guidance output...",  KeyAction::SET_ROBOT_READY },

    // ---- STIFFNESS GAIN TOGGLE ---------------------------------------------------
    // 'k' works regardless of the current InputState; the real outcome text
    // (enabled/disabled) is set via SetExternalStatus() by main.cpp.
    { {'k'}, InputState::ANY, InputState::SAME, "Toggling stiffness gain...", KeyAction::TOGGLE_STIFFNESS_GAIN },

    // ---- CALIBRATION ------------------------------------------------------------
    { {'C'}, InputState::IDLE,    InputState::CAL_SEL, "Select calibration mode: [a] ARoM, [s] Stiffness, [o] Offset...", KeyAction::NONE },
    { {'a'}, InputState::CAL_SEL, InputState::CAL_ROM, "Running ARoM calibration.", KeyAction::NONE },
    { {'s'}, InputState::CAL_SEL, InputState::CAL_STI, "Running stiffness calibration.", KeyAction::NONE },
    { {'o'}, InputState::CAL_SEL, InputState::CAL_OFF, "Running fingertip offset calibration.", KeyAction::NONE },

    // ---- LOGGING ---------------------------------------------------------------
    { {'L'}, InputState::ANY, InputState::LOG,     "Select logging option: [u] Set User ID...", KeyAction::NONE },
    { {'u'}, InputState::LOG, InputState::LOG_UID, "Enter ID for user (000-999)...", KeyAction::NONE },

    // ---- PWM_TEST ---------------------------------------------------------------
    { {'M'},      InputState::ANY,     InputState::MOT_PWM,     "Select motor to test: [a] Motor A, [b] Motor B, [c] Motor C, [d] All Motors...", KeyAction::NONE },
    { {'a', 185}, InputState::MOT_PWM, InputState::MOT_PWM_A,   "Enter PWM for Motor A (0-2047)...", KeyAction::NONE },
    { {'b', 183}, InputState::MOT_PWM, InputState::MOT_PWM_B,   "Enter PWM for Motor B (0-2047)...", KeyAction::NONE },
    { {'c', 178}, InputState::MOT_PWM, InputState::MOT_PWM_C,   "Enter PWM for Motor C (0-2047)...", KeyAction::NONE },
    { {'d', 181}, InputState::MOT_PWM, InputState::MOT_PWM_ALL, "Enter PWM for Motor A, B, C (0-2047)...", KeyAction::NONE },

    // ---- SERIAL ---------------------------------------------------------------
    // Real "established"/"disabled" text is set via SetExternalStatus() by
    // main.cpp once the actual connect/disconnect result is known.
    { {'S'}, InputState::ANY, InputState::SAME, "Toggling serial connection...", KeyAction::TOGGLE_SERIAL },

    // ---- ACCURACY (Task 1 - Fitts) ---------------------------------------------
    { {'F'},   InputState::IDLE,    InputState::FIT_SEL, "Select marker mode: [r] Random, [m] Manual...", KeyAction::NONE },
    { {'m'},   InputState::FIT_SEL, InputState::FIT_ACT, "Which marker (1-45)...", KeyAction::NONE },
    { {'m'},   InputState::FIT_RUN, InputState::FIT_ACT, "Which marker (1-45)...", KeyAction::NONE },
    { {'r'},   InputState::FIT_SEL, InputState::FIT_RUN, "Active marker set to [MARKER_ID].", KeyAction::RANDOM_FITTS_TARGET },
    { {'r'},   InputState::FIT_RUN, InputState::FIT_RUN, "Active marker set to [MARKER_ID].", KeyAction::RANDOM_FITTS_TARGET },

    // ---- TENSION / PRETENSION ---------------------------------------------------
    // 'T' opens the tensioning menu: [p] runs the full guided pretensioning
    // sequence (PretensionHandler, step 1/4); [a]/[b]/[c]/[d] jump straight
    // into standalone tension adjustment (TENSION_ADJUST) without the
    // unspool/zero steps.
    { {'T'}, InputState::ANY, InputState::TEN_MENU,
      "Tensioning: [p] guided pretensioning sequence, or [a/b/c/d] adjust tension directly.",
      KeyAction::NONE },

    // ---- Guided pretensioning sequence (PretensionHandler, steps 1-4) -----------
    // Enter advances UNSPOOL->ZERO and (with no value typed) TENSION->DONE; the
    // displayed status text comes from PretensionHandler::GetStatus() each frame.
    { {'p'},    InputState::TEN_MENU,    InputState::PRE_TENSION, "", KeyAction::NONE },
    { {13, 10}, InputState::PRE_TENSION, InputState::SAME,        "", KeyAction::PRETENSION_ADVANCE },

    // Step 3/4 - select a motor to adjust its preload tension setpoint [N]
    { {'a', 185}, InputState::PRE_TENSION, InputState::TEN_SEL_A,   "", KeyAction::NONE },
    { {'b', 183}, InputState::PRE_TENSION, InputState::TEN_SEL_B,   "", KeyAction::NONE },
    { {'c', 178}, InputState::PRE_TENSION, InputState::TEN_SEL_C,   "", KeyAction::NONE },
    { {'d', 181}, InputState::PRE_TENSION, InputState::TEN_SEL_ALL, "", KeyAction::NONE },

    // Step 3/4 - re-select a different motor while one is already selected
    { {'a', 185}, InputState::TEN_SEL_B,   InputState::TEN_SEL_A,   "", KeyAction::NONE },
    { {'a', 185}, InputState::TEN_SEL_C,   InputState::TEN_SEL_A,   "", KeyAction::NONE },
    { {'a', 185}, InputState::TEN_SEL_ALL, InputState::TEN_SEL_A,   "", KeyAction::NONE },
    { {'b', 183}, InputState::TEN_SEL_A,   InputState::TEN_SEL_B,   "", KeyAction::NONE },
    { {'b', 183}, InputState::TEN_SEL_C,   InputState::TEN_SEL_B,   "", KeyAction::NONE },
    { {'b', 183}, InputState::TEN_SEL_ALL, InputState::TEN_SEL_B,   "", KeyAction::NONE },
    { {'c', 178}, InputState::TEN_SEL_A,   InputState::TEN_SEL_C,   "", KeyAction::NONE },
    { {'c', 178}, InputState::TEN_SEL_B,   InputState::TEN_SEL_C,   "", KeyAction::NONE },
    { {'c', 178}, InputState::TEN_SEL_ALL, InputState::TEN_SEL_C,   "", KeyAction::NONE },
    { {'d', 181}, InputState::TEN_SEL_A,   InputState::TEN_SEL_ALL, "", KeyAction::NONE },
    { {'d', 181}, InputState::TEN_SEL_B,   InputState::TEN_SEL_ALL, "", KeyAction::NONE },
    { {'d', 181}, InputState::TEN_SEL_C,   InputState::TEN_SEL_ALL, "", KeyAction::NONE },

    // Step 3/4 - +/- nudge the selected motor's tension setpoint by 0.1 N
    { {61, 171}, InputState::TEN_SEL_A,   InputState::SAME, "", KeyAction::ADJUST_TENSION_INC },
    { {61, 171}, InputState::TEN_SEL_B,   InputState::SAME, "", KeyAction::ADJUST_TENSION_INC },
    { {61, 171}, InputState::TEN_SEL_C,   InputState::SAME, "", KeyAction::ADJUST_TENSION_INC },
    { {61, 171}, InputState::TEN_SEL_ALL, InputState::SAME, "", KeyAction::ADJUST_TENSION_INC },
    { {45, 173}, InputState::TEN_SEL_A,   InputState::SAME, "", KeyAction::ADJUST_TENSION_DEC },
    { {45, 173}, InputState::TEN_SEL_B,   InputState::SAME, "", KeyAction::ADJUST_TENSION_DEC },
    { {45, 173}, InputState::TEN_SEL_C,   InputState::SAME, "", KeyAction::ADJUST_TENSION_DEC },
    { {45, 173}, InputState::TEN_SEL_ALL, InputState::SAME, "", KeyAction::ADJUST_TENSION_DEC },

    // Step 3/4 - Enter with no value typed advances TENSION->DONE
    { {13, 10}, InputState::TEN_SEL_A,   InputState::SAME, "", KeyAction::PRETENSION_ADVANCE },
    { {13, 10}, InputState::TEN_SEL_B,   InputState::SAME, "", KeyAction::PRETENSION_ADVANCE },
    { {13, 10}, InputState::TEN_SEL_C,   InputState::SAME, "", KeyAction::PRETENSION_ADVANCE },
    { {13, 10}, InputState::TEN_SEL_ALL, InputState::SAME, "", KeyAction::PRETENSION_ADVANCE },

    // ---- Standalone tension adjustment (TENSION_ADJUST, no guided sequence) -----
    // Selecting a motor here enables ControllerHandler's manual tension mode
    // directly (main.cpp), bypassing the unspool/zero steps entirely.
    { {'a', 185}, InputState::TEN_MENU, InputState::TEN_ADJ_A,   "", KeyAction::NONE },
    { {'b', 183}, InputState::TEN_MENU, InputState::TEN_ADJ_B,   "", KeyAction::NONE },
    { {'c', 178}, InputState::TEN_MENU, InputState::TEN_ADJ_C,   "", KeyAction::NONE },
    { {'d', 181}, InputState::TEN_MENU, InputState::TEN_ADJ_ALL, "", KeyAction::NONE },

    // Re-select a different motor while one is already selected
    { {'a', 185}, InputState::TEN_ADJ_B,   InputState::TEN_ADJ_A,   "", KeyAction::NONE },
    { {'a', 185}, InputState::TEN_ADJ_C,   InputState::TEN_ADJ_A,   "", KeyAction::NONE },
    { {'a', 185}, InputState::TEN_ADJ_ALL, InputState::TEN_ADJ_A,   "", KeyAction::NONE },
    { {'b', 183}, InputState::TEN_ADJ_A,   InputState::TEN_ADJ_B,   "", KeyAction::NONE },
    { {'b', 183}, InputState::TEN_ADJ_C,   InputState::TEN_ADJ_B,   "", KeyAction::NONE },
    { {'b', 183}, InputState::TEN_ADJ_ALL, InputState::TEN_ADJ_B,   "", KeyAction::NONE },
    { {'c', 178}, InputState::TEN_ADJ_A,   InputState::TEN_ADJ_C,   "", KeyAction::NONE },
    { {'c', 178}, InputState::TEN_ADJ_B,   InputState::TEN_ADJ_C,   "", KeyAction::NONE },
    { {'c', 178}, InputState::TEN_ADJ_ALL, InputState::TEN_ADJ_C,   "", KeyAction::NONE },
    { {'d', 181}, InputState::TEN_ADJ_A,   InputState::TEN_ADJ_ALL, "", KeyAction::NONE },
    { {'d', 181}, InputState::TEN_ADJ_B,   InputState::TEN_ADJ_ALL, "", KeyAction::NONE },
    { {'d', 181}, InputState::TEN_ADJ_C,   InputState::TEN_ADJ_ALL, "", KeyAction::NONE },

    // +/- nudge the selected motor's tension setpoint by 0.1 N
    { {61, 171}, InputState::TEN_ADJ_A,   InputState::SAME, "", KeyAction::ADJUST_TENSION_INC },
    { {61, 171}, InputState::TEN_ADJ_B,   InputState::SAME, "", KeyAction::ADJUST_TENSION_INC },
    { {61, 171}, InputState::TEN_ADJ_C,   InputState::SAME, "", KeyAction::ADJUST_TENSION_INC },
    { {61, 171}, InputState::TEN_ADJ_ALL, InputState::SAME, "", KeyAction::ADJUST_TENSION_INC },
    { {45, 173}, InputState::TEN_ADJ_A,   InputState::SAME, "", KeyAction::ADJUST_TENSION_DEC },
    { {45, 173}, InputState::TEN_ADJ_B,   InputState::SAME, "", KeyAction::ADJUST_TENSION_DEC },
    { {45, 173}, InputState::TEN_ADJ_C,   InputState::SAME, "", KeyAction::ADJUST_TENSION_DEC },
    { {45, 173}, InputState::TEN_ADJ_ALL, InputState::SAME, "", KeyAction::ADJUST_TENSION_DEC },

    // ---- GAIN TUNING (custom-tuned proportional gain) ---------------------------
    // 'G' enters gain tuning: [+/-] nudges the selected motor's gainTune_X by
    // 0.1. gainTune_X is the custom-tuned proportional gain (seeded from
    // gain_kP), one of the two terms - with K(theta) - that form kP_effective
    // (Stage 1). [a/b/c/d] pick which motor's direction the adjustment applies
    // to ('d' = all three at once). Pressing 'G' again exits gain tuning and
    // resumes whatever task (FITTS, CAL_STI, ...) was active before - these
    // rows must come before the ANY row below so they win while already in a
    // GAIN_* state.
    { {'G'}, InputState::GAIN_ALL, InputState::SAME, "Gain tuning closed.", KeyAction::EXIT_GAIN_MODE },
    { {'G'}, InputState::GAIN_A,   InputState::SAME, "Gain tuning closed.", KeyAction::EXIT_GAIN_MODE },
    { {'G'}, InputState::GAIN_B,   InputState::SAME, "Gain tuning closed.", KeyAction::EXIT_GAIN_MODE },
    { {'G'}, InputState::GAIN_C,   InputState::SAME, "Gain tuning closed.", KeyAction::EXIT_GAIN_MODE },

    { {'G'}, InputState::ANY, InputState::GAIN_ALL,
      "Gain tuning: [+/-] adjust all motors, or [a/b/c/d] select a motor.",
      KeyAction::NONE },

    // Re-select a different motor while one is already selected
    { {'a', 185}, InputState::GAIN_ALL, InputState::GAIN_A, "", KeyAction::NONE },
    { {'a', 185}, InputState::GAIN_B,   InputState::GAIN_A, "", KeyAction::NONE },
    { {'a', 185}, InputState::GAIN_C,   InputState::GAIN_A, "", KeyAction::NONE },
    { {'b', 183}, InputState::GAIN_ALL, InputState::GAIN_B, "", KeyAction::NONE },
    { {'b', 183}, InputState::GAIN_A,   InputState::GAIN_B, "", KeyAction::NONE },
    { {'b', 183}, InputState::GAIN_C,   InputState::GAIN_B, "", KeyAction::NONE },
    { {'c', 178}, InputState::GAIN_ALL, InputState::GAIN_C, "", KeyAction::NONE },
    { {'c', 178}, InputState::GAIN_A,   InputState::GAIN_C, "", KeyAction::NONE },
    { {'c', 178}, InputState::GAIN_B,   InputState::GAIN_C, "", KeyAction::NONE },
    { {'d', 181}, InputState::GAIN_A,   InputState::GAIN_ALL, "", KeyAction::NONE },
    { {'d', 181}, InputState::GAIN_B,   InputState::GAIN_ALL, "", KeyAction::NONE },
    { {'d', 181}, InputState::GAIN_C,   InputState::GAIN_ALL, "", KeyAction::NONE },

    // +/- nudge the selected motor's gain tune by 0.1
    { {61, 171}, InputState::GAIN_ALL, InputState::SAME, "", KeyAction::ADJUST_GAIN_INC },
    { {61, 171}, InputState::GAIN_A,   InputState::SAME, "", KeyAction::ADJUST_GAIN_INC },
    { {61, 171}, InputState::GAIN_B,   InputState::SAME, "", KeyAction::ADJUST_GAIN_INC },
    { {61, 171}, InputState::GAIN_C,   InputState::SAME, "", KeyAction::ADJUST_GAIN_INC },
    { {45, 173}, InputState::GAIN_ALL, InputState::SAME, "", KeyAction::ADJUST_GAIN_DEC },
    { {45, 173}, InputState::GAIN_A,   InputState::SAME, "", KeyAction::ADJUST_GAIN_DEC },
    { {45, 173}, InputState::GAIN_B,   InputState::SAME, "", KeyAction::ADJUST_GAIN_DEC },
    { {45, 173}, InputState::GAIN_C,   InputState::SAME, "", KeyAction::ADJUST_GAIN_DEC },
};


// ---- Numeric entry formats ----------------------------------------------------
// States where digit/decimal-point keys are buffered until Enter confirms.

struct NumericEntryFormat {
    InputState  state;        ///< InputState that expects numeric entry
    int         totalLength;  ///< Total characters, including '.' if hasDecimal
    bool        hasDecimal;   ///< true => format is "n.n" (period at index 1)
    int         minValue;     ///< Minimum accepted value (decimal point removed, e.g. "0.0" -> 0)
    int         maxValue;     ///< Maximum accepted value (decimal point removed, e.g. "5.0" -> 50)
    InputState  newState;     ///< New InputState, or InputState::SAME
    KeyAction   action;
    std::string displayText;  ///< May contain [MARKER_ID] / [VAL] / [MOTOR]
};

inline const std::vector<NumericEntryFormat> kNumericEntryTable = {
    // LOGGING - nnn (000-999) -> user ID
    { InputState::LOG_UID, 3, false, 0, 999, InputState::LOG, KeyAction::SET_USER_ID, "User ID set to [VAL]." },

    // PWM_TEST - nnnn (0000-2047) -> motor PWM, 1 s test pulse
    { InputState::MOT_PWM_A,   4, false, 0, 2047, InputState::MOT_PWM, KeyAction::SET_MOTOR_PWM, "Sending test pulse to motor [MOTOR]." },
    { InputState::MOT_PWM_B,   4, false, 0, 2047, InputState::MOT_PWM, KeyAction::SET_MOTOR_PWM, "Sending test pulse to motor [MOTOR]." },
    { InputState::MOT_PWM_C,   4, false, 0, 2047, InputState::MOT_PWM, KeyAction::SET_MOTOR_PWM, "Sending test pulse to motor [MOTOR]." },
    { InputState::MOT_PWM_ALL, 4, false, 0, 2047, InputState::MOT_PWM, KeyAction::SET_MOTOR_PWM, "Sending test pulse to motor [MOTOR]." },

    // ACCURACY - nn (00-45) -> Fitts target marker
    { InputState::FIT_ACT, 2, false, 0, 45, InputState::FIT_RUN, KeyAction::SET_FITTS_TARGET, "Active marker set to [MARKER_ID]." },

    // PRETENSION step 3/4 - n.n (0.0-10.0) -> tension setpoint [N] for [MOTOR]
    { InputState::TEN_SEL_A,   3, true, 0, 100, InputState::SAME, KeyAction::SET_TENSION, "" },
    { InputState::TEN_SEL_B,   3, true, 0, 100, InputState::SAME, KeyAction::SET_TENSION, "" },
    { InputState::TEN_SEL_C,   3, true, 0, 100, InputState::SAME, KeyAction::SET_TENSION, "" },
    { InputState::TEN_SEL_ALL, 3, true, 0, 100, InputState::SAME, KeyAction::SET_TENSION, "" },

    // TENSION_ADJUST (standalone) - n.n (0.0-10.0) -> tension setpoint [N] for [MOTOR]
    { InputState::TEN_ADJ_A,   3, true, 0, 100, InputState::SAME, KeyAction::SET_TENSION, "" },
    { InputState::TEN_ADJ_B,   3, true, 0, 100, InputState::SAME, KeyAction::SET_TENSION, "" },
    { InputState::TEN_ADJ_C,   3, true, 0, 100, InputState::SAME, KeyAction::SET_TENSION, "" },
    { InputState::TEN_ADJ_ALL, 3, true, 0, 100, InputState::SAME, KeyAction::SET_TENSION, "" },
};
