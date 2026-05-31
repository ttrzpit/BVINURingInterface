#include "SerialHandler.h"

#include <chrono>
#include <iostream>

// =============================================================================
// SerialHandler.cpp — STUB for Teensy serial interface
//
// The public interface (send / getLatestReceived / start / stop) is complete
// and wired into main.cpp. The actual POSIX termios I/O is not yet implemented.
//
// To implement:
//   1. openPort(path, baudRate) — open file descriptor, configure termios
//   2. start() — call openPort for both fdSend_ and fdReceive_
//   3. send() — write(fdSend_, msg.c_str(), msg.size())
//   4. receiveLoop() — read(fdReceive_, ...) in a loop, store in latestReceived_
//   5. stop() — close both file descriptors
//
// main.cpp does not need to change when this stub is replaced.
// =============================================================================


SerialHandler::SerialHandler(const SerialConfig& cfg) : cfg_(cfg) {
    std::cout << "SerialHandler: STUB\n"
              << "SerialHandler: TX=" << cfg_.portSend
              << "  RX=" << cfg_.portReceive
              << "  baud=" << cfg_.baudRate << "\n";
}

SerialHandler::~SerialHandler() { stop(); }


// ---- Public -----------------------------------------------------------------

void SerialHandler::start() {
    // TODO: open cfg_.portSend  → fdSend_
    // TODO: open cfg_.portReceive → fdReceive_
    // Both should be configured with termios at cfg_.baudRate, 8N1, raw mode

    running_ = true;
    receiveThread_ = std::thread(&SerialHandler::receiveLoop, this);

    std::cout << "SerialHandler: STUB — start() called. (ports not open yet)\n";
}

void SerialHandler::stop() {
    running_ = false;
    if (receiveThread_.joinable()) receiveThread_.join();

    // TODO: close(fdSend_); close(fdReceive_);

    std::cout << "SerialHandler: STUB — stop() called.\n";
}

void SerialHandler::send(const std::string& msg) {
    // TODO: ::write(fdSend_, msg.c_str(), msg.size())
    (void)msg;   // Suppress unused-variable warning until implemented
}

std::string SerialHandler::getLatestReceived() {
    std::lock_guard<std::mutex> lock(receiveMutex_);
    std::string out = latestReceived_;
    latestReceived_.clear();
    return out;
}


// ---- Private ----------------------------------------------------------------

void SerialHandler::receiveLoop() {
    // TODO: read lines from fdReceive_ and store in latestReceived_
    // Example structure:
    //   char buf[256];
    //   while (running_) {
    //       int n = ::read(fdReceive_, buf, sizeof(buf) - 1);
    //       if (n > 0) {
    //           buf[n] = '\0';
    //           std::lock_guard<std::mutex> lock(receiveMutex_);
    //           latestReceived_ = std::string(buf, n);
    //       }
    //   }

    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
