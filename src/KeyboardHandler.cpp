#include "KeyboardHandler.h"

#include <algorithm>
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
    std::cout << "KeyboardHandler: Ready.\n";
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
    if (cmd.size() == 3 && cmd[0] == 'a' && std::isdigit(static_cast<unsigned char>(cmd[1])) && std::isdigit(static_cast<unsigned char>(cmd[2]))) {
        int id = (cmd[1] - '0') * 10 + (cmd[2] - '0');
        state_.activeTagId = id;
        state_.outputBuffer = (id == 0) ? "Active tag cleared"
                                        : "Active tag: ID " + std::to_string(id);
        return;
    }

    // Study commands
    // Command: u<NNN> — set current userID
    if (cmd.size() == 4 && cmd[0] == 'u' && std::isdigit(static_cast<unsigned char>(cmd[1])) && std::isdigit(static_cast<unsigned char>(cmd[2])) && std::isdigit(static_cast<unsigned char>(cmd[3]))) {
        // int id = (cmd[1] - '0') * 10 + (cmd[2] - '0');
        int userId = ((cmd[1] - '0') * 100) + ((cmd[2] - '0') * 10) + (cmd[3] - '0');
        state_.activeUserId = userId;
        state_.outputBuffer = (userId == 0) ? "Active user ID cleared"
                                        : "Active user ID set to " + std::to_string(userId);
        return;
    }

    // Serial connection management
    if (cmd == "connect" || cmd == "con") {
        state_.pendingSerialAction = SerialAction::CONNECT;
        state_.outputBuffer = "Connecting to Teensy...";
        return;
    }
    if (cmd == "disconnect" || cmd == "dis") {
        state_.pendingSerialAction = SerialAction::DISCONNECT;
        state_.outputBuffer = "Disconnecting from Teensy...";
        return;
    }

    // Fitts task — select a new random target marker (only active in FITTS state)
    if (cmd == "r") {
        if (state_.systemState == SystemState::FITTS) {
            // Uniform distribution over the 45 markers displayed on the grid (1–45)
            static std::mt19937 rng{std::random_device{}()};
            static std::uniform_int_distribution<int> dist(1, 45);
            state_.fittsTargetId = dist(rng);
            state_.activeTagId = state_.fittsTargetId;  // green outline on operator display
            state_.outputBuffer = "Target: marker " + std::to_string(state_.fittsTargetId);
        } else {
            state_.outputBuffer = "'r' is only active in FITTS state";
        }
        return;
    }

    // State commands
    if (cmd == "cal") {
        state_.systemState = SystemState::CALIBRATING;
        state_.outputBuffer = "State: CALIBRATING";
        return;
    }
    if (cmd == "cal3") {
        state_.systemState = SystemState::CAL3;
        state_.outputBuffer = "State: CAL3 — touch screen 10 times";
        return;
    }
    if (cmd == "idle") {
        state_.systemState = SystemState::IDLE;
        state_.outputBuffer = "State: IDLE";
        return;
    }
    if (cmd == "fitts" || cmd == "study1") {
        state_.systemState = SystemState::FITTS;
        state_.outputBuffer = "State: FITTS";
        return;
    }

    // testA<pwm> / testB<pwm> / testC<pwm> — drive one motor for 1 second then stop
    // Example: testA1984 → motor A at PWM 1984 for 1 s, then PWM 2047 (off)
    if (cmd.size() >= 6 && cmd.substr(0, 4) == "test" &&
        (cmd[4] == 'A' || cmd[4] == 'B' || cmd[4] == 'C')) {
        std::string numStr = cmd.substr(5);
        bool allDigits = !numStr.empty() && std::all_of(numStr.begin(), numStr.end(),
            [](unsigned char c){ return std::isdigit(c); });
        if (allDigits) {
            int pwmVal = std::stoi(numStr);
            if (pwmVal >= 0 && pwmVal <= 2047) {
                state_.pendingMotorTest.active = true;
                state_.pendingMotorTest.motor  = cmd[4];
                state_.pendingMotorTest.pwm    = static_cast<uint16_t>(pwmVal);
                state_.outputBuffer = "Test motor " + std::string(1, cmd[4]) +
                                      " PWM=" + numStr + " for 1 s...";
                return;
            }
        }
        state_.outputBuffer = "Bad test command — use testA<0-2047>";
        return;
    }

    // Add more commands here with additional else-if branches

    state_.outputBuffer = "Unknown: '" + cmd + "'";
}

void KeyboardHandler::EchoBuffer() const {
    // Input is now shown in the telemetry panel — no console echo needed
}
