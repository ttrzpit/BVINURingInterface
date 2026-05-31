#include "KeyboardHandler.h"

#include <cctype>
#include <iostream>

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
            std::cout << "\n";           // Move off the echo line before printing result
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
        if (id == 0)
            std::cout << "KeyboardHandler: Active tag cleared.\n";
        else
            std::cout << "KeyboardHandler: Active tag → ID " << id << "\n";
        return;
    }

    // Add more commands here with additional else-if branches

    std::cout << "KeyboardHandler: Unknown command '" << cmd << "'\n";
}

void KeyboardHandler::EchoBuffer() const {
    // \r returns the cursor to the start of the current line so each keystroke
    // overwrites the previous echo rather than printing a new line each time.
    // The trailing spaces erase any characters left over from a longer previous buffer.
    std::cout << "\rCmd> " << inputBuffer_ << "   \rCmd> " << inputBuffer_;
    std::cout.flush();
}
