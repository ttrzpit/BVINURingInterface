#pragma once

// =============================================================================
// SerialHandler.h — Serial communication stub for the Teensy interface
//
// STUB: The interface is fully defined and wired into main.cpp, but the actual
// POSIX termios I/O is not yet implemented. Replace the TODO bodies in
// SerialHandler.cpp to activate; main.cpp does not need to change.
//
// Design:
//   - send()               writes a string to the TX port (called from main loop)
//   - getLatestReceived()  returns the most recent RX string and clears it
//   - A background receive thread reads the RX port so incoming data never
//     blocks the main loop.
//
// Two separate serial ports are used (one TX, one RX) to allow the lowest
// possible latency in both directions — matching the hardware setup.
// =============================================================================

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "Config.h"


class SerialHandler {
public:
    /**
     * @param cfg  Serial config (port paths, baud rate).
     *             Must outlive this object.
     */
    explicit SerialHandler(const SerialConfig& cfg);
    ~SerialHandler();

    /** @brief Open serial ports and launch the receive thread. (STUB) */
    void start();

    /** @brief Close serial ports and stop the receive thread. (STUB) */
    void stop();

    /**
     * @brief Transmit a message on the send port.
     * @param msg  String to transmit (caller formats the payload)
     */
    void send(const std::string& msg);

    /**
     * @brief Returns the most recent received message and clears it.
     *        Returns an empty string if nothing has been received since the last call.
     */
    std::string getLatestReceived();

private:
    void receiveLoop();  // Runs on receiveThread_ — reads incoming serial data

    const SerialConfig& cfg_;

    // Latest RX string — written by receiveLoop, read by getLatestReceived
    std::string latestReceived_;
    std::mutex  receiveMutex_;

    // TX file descriptor (-1 = not open)
    int fdSend_    = -1;
    // RX file descriptor (-1 = not open)
    int fdReceive_ = -1;

    std::thread       receiveThread_;
    std::atomic<bool> running_{ false };
};
