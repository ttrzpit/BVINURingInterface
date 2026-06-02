#pragma once

// =============================================================================
// KeyboardHandler.h — Keyboard command buffer with Enter-to-execute
//
// Keystrokes from cv::pollKey() are accumulated into a string buffer that is
// echoed to the console in real-time. Pressing Enter executes the command.
// Backspace removes the last character. ESC clears the buffer and quits.
//
// The current buffer and any executed state are exposed in KeyboardState, which
// main.cpp reads each loop and passes to whichever handlers need it.
//
// Current commands (type the string then press Enter):
//   a<NN>  — set the active ArUco tag to ID NN (two digits, 00–49)
//             a00  →  clear active tag (no overlay)
//             a01  →  highlight marker ID 1 with a green outline
//
// Adding a new command:
//   1. Add output fields to KeyboardState as needed.
//   2. Add an else-if branch in ParseCommand().
// =============================================================================

#include <string>


// ---- System state -----------------------------------------------------------

enum class SystemState {
    IDLE,         ///< Default — no active task, ArUco grid hidden
    CALIBRATING,  ///< General calibration — ArUco grid shown on touchscreen
    CAL3,         ///< Calibration Stage 3: camera-to-fingertip offset collection
    FITTS         ///< Fitts task running
};

// ---- Output type ------------------------------------------------------------

/**
 * @brief Keyboard-driven state distributed by main.cpp each loop.
 */
// ---- Serial connection action -----------------------------------------------
// Set by "connect" / "disconnect" commands; cleared by main.cpp after acting.

enum class SerialAction { NONE, CONNECT, DISCONNECT };

// ---- Output type ------------------------------------------------------------

struct KeyboardState {
    int          activeTagId         = 0;                  ///< ArUco ID to highlight (0 = none)
    int          activeUserId        = -1;                 ///< User ID for study logging (-1 = not set, 000 = non-logging, 001> = valid user)
    bool         quitRequested       = false;
    SystemState  systemState         = SystemState::IDLE;  ///< Current system operating state
    int          fittsTargetId       = 0;                  ///< Randomly selected Fitts target (0 = none)
    SerialAction pendingSerialAction = SerialAction::NONE; ///< One-shot connect/disconnect request
    std::string  inputBuffer;                              ///< Command currently being typed
    std::string  outputBuffer;                             ///< Result of the last executed command
};


// ---- Handler ----------------------------------------------------------------

class KeyboardHandler {
public:
    KeyboardHandler();

    /**
     * @brief Ingest one raw keystroke value from cv::pollKey().
     * @param key  Raw return value from cv::pollKey() — pass it unmasked.
     *             This function handles the -1 "no key" sentinel internally.
     */
    void ProcessKey(int key);

    /** @brief Returns the current state for main.cpp to distribute. */
    const KeyboardState& GetState() const { return state_; }

    /**
     * @brief Overwrite the output buffer with a message from an external source
     *        (e.g. Cal3Handler status). Does not affect the input buffer or any
     *        other state field.
     */
    void SetExternalStatus(const std::string& msg) { state_.outputBuffer = msg; }

    /** @brief Clear the serial action after main.cpp has acted on it. */
    void ClearSerialAction() { state_.pendingSerialAction = SerialAction::NONE; }

private:
    void ParseCommand(const std::string& cmd);  // Called on Enter
    void EchoBuffer() const;                    // Redraws the current buffer on one console line

    KeyboardState state_;
    std::string   inputBuffer_;   // In-progress command (mirrors state_.inputBuffer)
};
