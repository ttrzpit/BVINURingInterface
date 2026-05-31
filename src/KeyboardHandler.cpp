#include "KeyboardHandler.h"

#include <cctype>
#include <iostream>
#include <random>

// =============================================================================
// KeyboardHandler.cpp
//
// Key handling rules:
//   Printable ASCII (32–126) → append to buffer, echo to console
//   Enter (13 or 10)         → parse buffer, clear buffer
//   Backspace (8 or 127)     → remove last character, re-echo
//   ESC (27)                 → clear buffer, set quitRequested
//   Everything else          → ignored
//
// Console echo: each keystroke overwrites the same console line using \r so
// the user can see what they have typed without scrolling the terminal output.
// When Enter is pressed the result is printed on a new line.
// =============================================================================

KeyboardHandler::KeyboardHandler() {
    std::cout << "KeyboardHandler: Ready.  Commands (type then press Enter):\n"
              << "  a<NN>   set active ArUco tag  (a00 = clear)\n\n";
}

void KeyboardHandler::ProcessKey(int key) {

    // cv::pollKey() returns -1 when no key is pending — exit immediately
    if (key < 0) return;

    // Strip modifier bits (e.g. Fn keys return values > 255); extract the
    // character byte. This must happen AFTER the -1 check because
    // (-1) & 0xFF == 255 which is not -1 and would slip through the guard.
    key = key & 0xFF;

    // ---- ESC ----------------------------------------------------------------
    if (key == 27) {
        inputBuffer_.clear();
        state_.inputBuffer.clear();
        state_.quitRequested = true;
        std::cout << "\n";
        return;
    }

    // ---- Enter (carriage return or line feed) --------------------------------
    if (key == 13 || key == 10) {
        if (!inputBuffer_.empty()) {
            ParseCommand(inputBuffer_);
            inputBuffer_.clear();
            state_.inputBuffer.clear();
        }
        return;
    }

    // ---- Backspace or Delete -------------------------------------------------
    if (key == 8 || key == 127) {
        if (!inputBuffer_.empty()) {
            inputBuffer_.pop_back();
            state_.inputBuffer = inputBuffer_;
            EchoBuffer();
        }
        return;
    }

    // ---- Printable character ------------------------------------------------
    if (key >= 32 && key < 127) {
        inputBuffer_ += static_cast<char>(key);
        state_.inputBuffer = inputBuffer_;
        EchoBuffer();
    }
    // All other key codes (special keys, Fn, arrows, etc.) are silently ignored
}

// =============================================================================
// Private
// =============================================================================

void KeyboardHandler::ParseCommand(const std::string& cmd) {

    // Command: a<NN> — set active ArUco tag
    if (cmd.size() == 3
        && cmd[0] == 'a'
        && std::isdigit(static_cast<unsigned char>(cmd[1]))
        && std::isdigit(static_cast<unsigned char>(cmd[2])))
    {
        int id = (cmd[1] - '0') * 10 + (cmd[2] - '0');
        state_.activeTagId = id;
        state_.outputBuffer = (id == 0) ? "Active tag cleared"
                                        : "Active tag: ID " + std::to_string(id);
        return;
    }

    // Fitts task — select a new random target marker (only active in FITTS state)
    if (cmd == "r") {
        if (state_.systemState == SystemState::FITTS) {
            // Uniform distribution over the 45 markers displayed on the grid (1–45)
            static std::mt19937 rng{ std::random_device{}() };
            static std::uniform_int_distribution<int> dist(1, 45);
            state_.fittsTargetId = dist(rng);
            state_.activeTagId   = state_.fittsTargetId;   // green outline on operator display
            state_.outputBuffer  = "Target: marker " + std::to_string(state_.fittsTargetId);
        } else {
            state_.outputBuffer = "'r' is only active in FITTS state";
        }
        return;
    }

    // State commands
    if (cmd == "cal") {
        state_.systemState  = SystemState::CALIBRATING;
        state_.outputBuffer = "State: CALIBRATING";
        return;
    }
    if (cmd == "cal3") {
        state_.systemState  = SystemState::CAL3;
        state_.outputBuffer = "State: CAL3 — touch screen 10 times";
        return;
    }
    if (cmd == "idle") {
        state_.systemState  = SystemState::IDLE;
        state_.outputBuffer = "State: IDLE";
        return;
    }
    if (cmd == "fitts") {
        state_.systemState  = SystemState::FITTS;
        state_.outputBuffer = "State: FITTS";
        return;
    }

    // Add more commands here with additional else-if branches

    state_.outputBuffer = "Unknown: '" + cmd + "'";
}

void KeyboardHandler::EchoBuffer() const {
    // Input is now shown in the telemetry panel — no console echo needed
}
