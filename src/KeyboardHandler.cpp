#include "KeyboardHandler.h"
#include "KeyCommandTable.h"

#include <algorithm>
#include <iostream>
#include <random>

// =============================================================================
// KeyboardHandler.cpp
//
// Every keystroke is processed immediately:
//   1. ESC / SPACE / grave (27/32/96) always apply, even mid numeric-entry -
//      they quit / e-stop-toggle / reset-to-IDLE respectively. (Delete=127
//      clears input to IDLE, but only outside numeric entry, where it is
//      backspace.)
//   2. If the current inputState expects a numeric entry (kNumericEntryTable),
//      digit/decimal/Backspace/Enter keys are buffered until Enter confirms.
//   3. Otherwise, kKeyCommandTable is scanned for a row matching (key,
//      inputState) or (key, ANY); the first match fires its action and
//      transition.
//   4. No match - state is left unchanged.
// =============================================================================

KeyboardHandler::KeyboardHandler() {
    std::cout << "KeyboardHandler: Ready.\n";
}

void KeyboardHandler::ProcessKey(int key) {
    // cv::pollKey() returns -1 when no key is pending - exit immediately
    if (key < 0) return;

    // Strip modifier bits (e.g. Fn keys return values > 255); extract the
    // character byte. This must happen AFTER the -1 check because
    // (-1) & 0xFF == 255 which is not -1 and would slip through the guard.
    key = key & 0xFF;

    // For debugging
    // std::cout << "Key code: " << key << "\n";

    // ---- Global escape hatches - always take priority -----------------------
    // ESC (quit), SPACE (e-stop), grave (reset), Delete (255, cancel to IDLE).
    // These bypass numeric entry so they fire from any state - including mid
    // numeric-entry. Backspace (8/127) is deliberately NOT here, so it still
    // edits the numeric buffer rather than cancelling.
    if (key == 27 || key == 32 || key == 96 || key == 255) {
        DispatchTableCommand(key);
        return;
    }

    // ---- Numeric entry (nnn/nnnn/nn/n.n) -------------------------------------
    if (ProcessNumericEntry(key)) return;

    // ---- Single-key table dispatch -------------------------------------------
    DispatchTableCommand(key);
}

// =============================================================================
// Private
// =============================================================================

bool KeyboardHandler::ProcessNumericEntry(int key) {
    const NumericEntryFormat* fmt = nullptr;
    for (const auto& f : kNumericEntryTable) {
        if (f.state == state_.inputState) {
            fmt = &f;
            break;
        }
    }
    if (!fmt) return false;  // current state isn't a numeric-entry state

    // Backspace / Delete
    if (key == 8 || key == 255) {
        if (!inputBuffer_.empty()) {
            inputBuffer_.pop_back();
            state_.inputBuffer = inputBuffer_;
        }
        return true;
    }

    // Enter - validate and commit
    if (key == 13 || key == 10) {
        // A "bare" Enter (nothing typed yet) isn't a numeric confirmation -
        // let it fall through to DispatchTableCommand (e.g. PRETENSION_ADVANCE
        // while a TEN_SEL_* motor is selected).
        if (inputBuffer_.empty()) return false;
        if (static_cast<int>(inputBuffer_.size()) != fmt->totalLength) return true;
        if (fmt->hasDecimal && inputBuffer_[1] != '.') return true;

        std::string digits;
        for (char c : inputBuffer_) {
            if (c != '.') digits += c;
        }
        int rawValue = std::stoi(digits);  // for n.n, this is value*10 (e.g. "2.5" -> 25)

        if (rawValue < fmt->minValue || rawValue > fmt->maxValue) {
            state_.outputBuffer = "Invalid value - must be " +
                                   FormatNumericValue(fmt->minValue, fmt->hasDecimal) + " to " +
                                   FormatNumericValue(fmt->maxValue, fmt->hasDecimal) + ".";
            state_.lastInputKey = key;
            inputBuffer_.clear();
            state_.inputBuffer.clear();
            return true;
        }

        InputState oldState = state_.inputState;
        ExecuteAction(fmt->action, rawValue);

        if (fmt->newState != InputState::SAME) {
            state_.inputState  = fmt->newState;
            state_.systemState = DeriveSystemState(state_.inputState);
        }
        state_.outputBuffer = FormatDisplayText(fmt->displayText, rawValue, fmt->hasDecimal, oldState);
        state_.lastInputKey = key;

        inputBuffer_.clear();
        state_.inputBuffer.clear();
        return true;
    }

    // Digit - number row (48-57) or numpad (176-185)
    char ch = '\0';
    if (key >= '0' && key <= '9') {
        ch = static_cast<char>(key);
    } else if (key >= 176 && key <= 185) {
        ch = static_cast<char>('0' + (key - 176));
    } else if (fmt->hasDecimal && (key == '.' || key == 174)) {
        ch = '.';
    }

    if (ch != '\0' && static_cast<int>(inputBuffer_.size()) < fmt->totalLength) {
        inputBuffer_ += ch;
        state_.inputBuffer = inputBuffer_;
    }

    // TEN_SEL_* (pretensioning step 3/4) and TEN_ADJ_* (standalone tension
    // adjust) also bind a/b/c/d (reselect motor) and +/- (nudge by 0.1 N) via
    // kKeyCommandTable. Don't swallow those here - let any key that isn't a
    // digit/decimal fall through to DispatchTableCommand.
    switch (state_.inputState) {
        case InputState::TEN_SEL_A:
        case InputState::TEN_SEL_B:
        case InputState::TEN_SEL_C:
        case InputState::TEN_SEL_ALL:
        case InputState::TEN_ADJ_A:
        case InputState::TEN_ADJ_B:
        case InputState::TEN_ADJ_C:
        case InputState::TEN_ADJ_ALL:
            if (ch == '\0') return false;
            break;
        default:
            break;
    }

    return true;  // numeric-entry state consumes (or ignores) all other keys
}

void KeyboardHandler::DispatchTableCommand(int key) {
    for (const auto& row : kKeyCommandTable) {
        bool keyMatches = std::find(row.keys.begin(), row.keys.end(), key) != row.keys.end();
        if (!keyMatches) continue;
        if (row.requiredState != InputState::ANY && row.requiredState != state_.inputState) continue;

        InputState oldState = state_.inputState;
        ExecuteAction(row.action);

        if (row.newState == InputState::QUIT) {
            state_.quitRequested = true;
        } else if (row.newState != InputState::SAME) {
            // Entering the gain-tuning overlay for the first time: remember
            // the state to return to on EXIT_GAIN_MODE, and leave systemState
            // untouched so the active task keeps running underneath it.
            if (IsGainTuneInputState(row.newState) && !IsGainTuneInputState(oldState)) {
                preGainState_ = oldState;
            }

            state_.inputState = row.newState;
            if (!IsGainTuneInputState(row.newState)) {
                state_.systemState = DeriveSystemState(state_.inputState);
            }
        }

        state_.outputBuffer = FormatDisplayText(row.displayText, -1, false, oldState);
        state_.lastInputKey = key;
        return;
    }
    // No match - leave state unchanged (don't clobber e.g. Cal3's live status)
}

void KeyboardHandler::ExecuteAction(KeyAction action, int value) {
    switch (action) {
        case KeyAction::TOGGLE_SERIAL:
            state_.pendingSerialAction = SerialAction::TOGGLE;
            break;

        case KeyAction::SET_USER_ID:
            state_.activeUserId = value;
            break;

        case KeyAction::SET_MOTOR_PWM:
            state_.pendingMotorTest.active = true;
            state_.pendingMotorTest.motor  = MotorLetterFromState(state_.inputState);
            state_.pendingMotorTest.pwm    = static_cast<uint16_t>(value);
            break;

        case KeyAction::RANDOM_FITTS_TARGET:
            // The distance-stratified pick needs marker positions, which live in
            // main with the board layout - just flag the request here.
            state_.pendingRandomTarget = true;
            break;

        case KeyAction::TOGGLE_LOGGING:
            state_.pendingLoggingToggle = true;
            break;

        case KeyAction::SET_FITTS_TARGET:
            if ( value < fittsTargetIdMin_ ) value = fittsTargetIdMin_;
            if ( value > fittsBoardMaxId_  ) value = fittsBoardMaxId_;
            state_.fittsTargetId = value;
            state_.activeTagId   = value;
            break;

        case KeyAction::PRETENSION_ADVANCE:
            state_.pendingPretensionAdvance = true;
            break;

        case KeyAction::ADJUST_TENSION_INC:
            state_.pendingTensionAdjust.active     = true;
            state_.pendingTensionAdjust.motor      = MotorLetterFromState(state_.inputState);
            state_.pendingTensionAdjust.isAbsolute = false;
            state_.pendingTensionAdjust.deltaN     = 0.1f;
            break;

        case KeyAction::ADJUST_TENSION_DEC:
            state_.pendingTensionAdjust.active     = true;
            state_.pendingTensionAdjust.motor      = MotorLetterFromState(state_.inputState);
            state_.pendingTensionAdjust.isAbsolute = false;
            state_.pendingTensionAdjust.deltaN     = -0.1f;
            break;

        case KeyAction::SET_TENSION:
            state_.pendingTensionAdjust.active     = true;
            state_.pendingTensionAdjust.motor      = MotorLetterFromState(state_.inputState);
            state_.pendingTensionAdjust.isAbsolute = true;
            state_.pendingTensionAdjust.valueN     = static_cast<float>(value) / 10.0f;
            break;

        case KeyAction::ADJUST_GAIN_INC:
            state_.pendingGainAdjust.active    = true;
            state_.pendingGainAdjust.motor     = MotorLetterFromState(state_.inputState);
            state_.pendingGainAdjust.deltaGain = 0.01f;
            break;

        case KeyAction::ADJUST_GAIN_DEC:
            state_.pendingGainAdjust.active    = true;
            state_.pendingGainAdjust.motor     = MotorLetterFromState(state_.inputState);
            state_.pendingGainAdjust.deltaGain = -0.01f;
            break;

        case KeyAction::EXIT_GAIN_MODE:
            state_.inputState = preGainState_;
            break;

        case KeyAction::SET_ROBOT_IDLE:
            state_.pendingRobotStateRequest = RobotStateRequest::GO_IDLE;
            break;

        case KeyAction::SET_ROBOT_READY:
            state_.pendingRobotStateRequest = RobotStateRequest::GO_READY;
            break;

        case KeyAction::TOGGLE_STIFFNESS_GAIN:
            state_.pendingStiffnessGainToggle = true;
            break;

        case KeyAction::TOGGLE_ESTOP:
            state_.pendingEStopToggle = true;
            break;

        case KeyAction::SET_HOME_POSITION:
            state_.pendingSetHomePosition = true;
            break;

        case KeyAction::NONE:
        default:
            break;
    }
}

std::string KeyboardHandler::FormatDisplayText(const std::string& tmpl, int value,
                                                bool isDecimal, InputState contextState) const {
    std::string text = tmpl;

    auto replace = [&text](const std::string& token, const std::string& val) {
        size_t pos = text.find(token);
        if (pos != std::string::npos) text.replace(pos, token.size(), val);
    };

    replace("[MARKER_ID]", std::to_string(state_.fittsTargetId));

    if (value >= 0) {
        replace("[VAL]", FormatNumericValue(value, isDecimal));
    }

    replace("[MOTOR]", MotorLabelFromState(contextState));

    return text;
}

std::string KeyboardHandler::FormatNumericValue(int value, bool isDecimal) const {
    return isDecimal
        ? (std::to_string(value / 10) + "." + std::to_string(value % 10))
        : std::to_string(value);
}

char KeyboardHandler::MotorLetterFromState(InputState state) const {
    switch (state) {
        case InputState::MOT_PWM_A:
        case InputState::TEN_SEL_A:
        case InputState::TEN_ADJ_A:
        case InputState::GAIN_A: return 'A';
        case InputState::MOT_PWM_B:
        case InputState::TEN_SEL_B:
        case InputState::TEN_ADJ_B:
        case InputState::GAIN_B: return 'B';
        case InputState::MOT_PWM_C:
        case InputState::TEN_SEL_C:
        case InputState::TEN_ADJ_C:
        case InputState::GAIN_C: return 'C';
        case InputState::MOT_PWM_ALL:
        case InputState::TEN_SEL_ALL:
        case InputState::TEN_ADJ_ALL:
        case InputState::GAIN_ALL: return 'D';
        default: return 'A';
    }
}

std::string KeyboardHandler::MotorLabelFromState(InputState state) const {
    switch (state) {
        case InputState::MOT_PWM_A:
        case InputState::TEN_SEL_A:
        case InputState::TEN_ADJ_A:
            return "A";
        case InputState::MOT_PWM_B:
        case InputState::TEN_SEL_B:
        case InputState::TEN_ADJ_B:
            return "B";
        case InputState::MOT_PWM_C:
        case InputState::TEN_SEL_C:
        case InputState::TEN_ADJ_C:
            return "C";
        case InputState::MOT_PWM_ALL:
            return "A, B, C";
        case InputState::TEN_SEL_ALL:
        case InputState::TEN_ADJ_ALL:
        case InputState::GAIN_ALL:
            return "All";
        case InputState::GAIN_A:
            return "A";
        case InputState::GAIN_B:
            return "B";
        case InputState::GAIN_C:
            return "C";
        default:
            return "";
    }
}

bool IsGainTuneInputState(InputState state) {
    switch (state) {
        case InputState::GAIN_ALL:
        case InputState::GAIN_A:
        case InputState::GAIN_B:
        case InputState::GAIN_C:
            return true;
        default:
            return false;
    }
}

SystemState DeriveSystemState(InputState state) {
    switch (state) {
        case InputState::CAL_SEL:
        case InputState::CAL_ROM:
        case InputState::CAL_STI:
            return SystemState::CALIBRATING;
        case InputState::CAL_OFF:
            return SystemState::CAL3;
        case InputState::FIT_SEL:
        case InputState::FIT_RUN:
        case InputState::FIT_ACT:
            return SystemState::FITTS;
        case InputState::PRE_TENSION:
        case InputState::TEN_SEL_ALL:
        case InputState::TEN_SEL_A:
        case InputState::TEN_SEL_B:
        case InputState::TEN_SEL_C:
            return SystemState::PRETENSION;
        case InputState::TEN_ADJ_ALL:
        case InputState::TEN_ADJ_A:
        case InputState::TEN_ADJ_B:
        case InputState::TEN_ADJ_C:
            return SystemState::TENSION_ADJUST;
        default:
            return SystemState::IDLE;
    }
}
