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
    CALIBRATING,  ///< Calibration in progress — ArUco grid shown on touchscreen
    FITTS         ///< Fitts task running
};

// ---- Output type ------------------------------------------------------------

/**
 * @brief Keyboard-driven state distributed by main.cpp each loop.
 */
struct KeyboardState {
    int         activeTagId    = 0;                ///< ArUco ID to highlight (0 = none)
    bool        quitRequested  = false;
    SystemState systemState    = SystemState::IDLE; ///< Current system operating state
    int         fittsTargetId  = 0;                ///< Randomly selected Fitts target (0 = none)
    std::string inputBuffer;                        ///< Command currently being typed
    std::string outputBuffer;                       ///< Result of the last executed command
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

private:
    void ParseCommand(const std::string& cmd);  // Called on Enter
    void EchoBuffer() const;                    // Redraws the current buffer on one console line

    KeyboardState state_;
    std::string   inputBuffer_;   // In-progress command (mirrors state_.inputBuffer)
};
