#include "SerialHandler.h"

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// =============================================================================
// SerialHandler.cpp - PC ↔ Teensy binary serial protocol
//
// Wire format: [0xAA] [N payload bytes] [XOR checksum]
//   N = sizeof(TeensyToPcPacket) = 20 bytes for incoming packets
//
// Receive state machine:
//   WAIT_START  → scan for 0xAA
//   READ_PAYLOAD → accumulate sizeof(TeensyToPcPacket) bytes
//   READ_CHECKSUM → validate XOR; store packet if valid, discard if not
//
// The state machine re-syncs automatically after corruption: it always waits
// for the next 0xAA to start a new frame.
// =============================================================================

// ---- Construction -----------------------------------------------------------

SerialHandler::SerialHandler(const SerialConfig& cfg) : cfg_(cfg) {
    std::cout << "SerialHandler: Port " << cfg_.port
              << " at " << cfg_.baudRate << " baud\n";
}

SerialHandler::~SerialHandler() { stop(); }


// ---- Lifecycle --------------------------------------------------------------

void SerialHandler::start() {
    // Port is NOT opened here - call Connect() explicitly via the "connect" command.
    std::cout << "SerialHandler: Ready. Type 'connect' to open the Teensy port.\n";
}

void SerialHandler::stop() {
    Disconnect();   // Safe no-op if already disconnected
}

void SerialHandler::Connect() {
    if (fd_ >= 0) {
        std::cout << "SerialHandler: Already connected.\n";
        return;
    }
    if (!OpenPort()) {
        std::cerr << "SerialHandler: Connect failed - is the Teensy plugged in?\n";
        return;
    }
    running_       = true;
    txThread_      = std::thread(&SerialHandler::TxLoop, this);
    receiveThread_ = std::thread(&SerialHandler::ReceiveLoop, this);
    std::cout << "SerialHandler: Connected - TX at 200 Hz, RX listening.\n";
}

void SerialHandler::Disconnect() {
    if (fd_ < 0) return;
    running_ = false;

    // Join TX thread first - once it exits, no concurrent write() calls remain
    // and we can safely write the failsafe packet from this thread.
    if (txThread_.joinable()) txThread_.join();

    // Send one final packet with zero force (pwm = 2047) and IDLE state so the
    // Teensy always returns to a safe condition regardless of why we disconnected.
    PcToTeensyPacket safe = {};
    safe.state        = PcState::IDLE;
    safe.packet_index = lastSentIndex_.load();
    safe.pwm_A        = 2047;
    safe.pwm_B        = 2047;
    safe.pwm_C        = 2047;
    Send(safe);
    std::cout << "SerialHandler: Failsafe packet sent (IDLE, pwm=2047).\n";

    // RX thread may block up to 1 s on read() timeout before noticing running_=false
    if (receiveThread_.joinable()) receiveThread_.join();

    ClosePort();
    txFrequencyHz_ = 0.0f;
    std::cout << "SerialHandler: Disconnected.\n";
}


// ---- TX ---------------------------------------------------------------------

void SerialHandler::SetPendingTx(const PcToTeensyPacket& cmd) {
    std::lock_guard<std::mutex> lock(txMutex_);
    pendingTx_ = cmd;
    // packet_index is managed by TxLoop - whatever was set here will be overwritten
}

void SerialHandler::TxLoop() {
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;

    static constexpr int TX_PERIOD_US = 5000;  // 200 Hz = 5 ms

    uint8_t txIndex    = 0;
    int     freqCount  = 0;
    auto    nextWake   = Clock::now();
    auto    freqWindow = Clock::now();

    while (running_) {
        nextWake += Micros(TX_PERIOD_US);

        // If we've fallen more than one period behind (e.g. after a long stall),
        // reset the wake time to avoid a burst of catch-up packets.
        auto now = Clock::now();
        if (nextWake < now - Micros(TX_PERIOD_US)) {
            nextWake = now;
        }

        std::this_thread::sleep_until(nextWake);

        if (fd_ < 0) continue;

        // Snapshot the pending command and stamp the rolling packet index
        PcToTeensyPacket pkt;
        {
            std::lock_guard<std::mutex> lock(txMutex_);
            pkt = pendingTx_;
        }
        pkt.packet_index = txIndex;
        lastSentIndex_.store(txIndex);   // Record before incrementing so display matches what was sent
        txIndex = (txIndex + 1) % 100;

        Send(pkt);

        // Update measured TX frequency once per second
        freqCount++;
        double elapsed = std::chrono::duration<double>(Clock::now() - freqWindow).count();
        if (elapsed >= 1.0) {
            txFrequencyHz_.store(static_cast<float>(freqCount / elapsed));
            freqCount  = 0;
            freqWindow = Clock::now();
        }
    }
}

void SerialHandler::Send(const PcToTeensyPacket& pkt) {
    if (fd_ < 0) return;

    constexpr size_t PAYLOAD = sizeof(PcToTeensyPacket);
    uint8_t buf[PAYLOAD + 2];   // start byte + payload + checksum

    buf[0] = PACKET_START_BYTE;
    memcpy(&buf[1], &pkt, PAYLOAD);
    buf[PAYLOAD + 1] = ComputeChecksum(reinterpret_cast<const uint8_t*>(&pkt), PAYLOAD);

    // write() is safe to call from the main thread while the receive thread
    // does read() - Linux guarantees separate TX/RX for full-duplex serial fds.
    [[maybe_unused]] ssize_t n = write(fd_, buf, sizeof(buf));
}


// ---- RX ---------------------------------------------------------------------

bool SerialHandler::GetLatestPacket(TeensyToPcPacket& out) {
    std::lock_guard<std::mutex> lock(rxMutex_);
    if (!rxReady_) return false;
    out = latestRx_;
    return true;
}


// ---- Private - port setup ---------------------------------------------------

bool SerialHandler::OpenPort() {
    fd_ = open(cfg_.port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        std::cerr << "SerialHandler: Cannot open " << cfg_.port
                  << " - is the Teensy connected?\n";
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd_, &tty);

    // Baud rate - B1000000 = 1 Mbaud, defined in <termios.h> on Linux.
    // For USB CDC (virtual COM) ports this is informational; real speed
    // is determined by USB scheduling, not baud rate.
    cfsetispeed(&tty, B1000000);
    cfsetospeed(&tty, B1000000);

    // 8N1, no flow control
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= CREAD | CLOCAL;

    // Raw mode - no line discipline, no echo, no signals
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_iflag |= IGNBRK;

    // Blocking read: return when ≥1 byte available, or after 1 s timeout.
    // The timeout lets ReceiveLoop check running_ and exit cleanly on stop().
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 10;   // tenths of seconds

    tcsetattr(fd_, TCSANOW, &tty);
    tcflush(fd_, TCIOFLUSH);   // Discard any stale bytes

    std::cout << "SerialHandler: Opened " << cfg_.port << "\n";
    return true;
}

void SerialHandler::ClosePort() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}


// ---- Private - receive thread -----------------------------------------------

void SerialHandler::ReceiveLoop() {

    if (fd_ < 0) {
        // Port failed to open - sleep until stop() is called
        while (running_) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return;
    }

    constexpr size_t PAYLOAD = sizeof(TeensyToPcPacket);

    enum class Phase { WAIT_START, READ_PAYLOAD, READ_CHECKSUM };
    Phase   phase      = Phase::WAIT_START;
    uint8_t buf[PAYLOAD];
    size_t  bufIdx     = 0;

    while (running_) {
        uint8_t byte;
        ssize_t n = read(fd_, &byte, 1);

        if (n <= 0) {
            // Timeout (VTIME elapsed) or error - check running_ and continue
            continue;
        }

        switch (phase) {

            case Phase::WAIT_START:
                if (byte == PACKET_START_BYTE) {
                    phase  = Phase::READ_PAYLOAD;
                    bufIdx = 0;
                }
                break;

            case Phase::READ_PAYLOAD:
                buf[bufIdx++] = byte;
                if (bufIdx == PAYLOAD) {
                    phase = Phase::READ_CHECKSUM;
                }
                break;

            case Phase::READ_CHECKSUM: {
                uint8_t expected = ComputeChecksum(buf, PAYLOAD);
                if (byte == expected) {
                    // Valid packet - publish to main thread
                    std::lock_guard<std::mutex> lock(rxMutex_);
                    memcpy(&latestRx_, buf, PAYLOAD);
                    rxReady_ = true;
                } else {
                    // Checksum mismatch - log and wait for next start byte
                    // (silent discard is fine; index counter will reveal the miss)
                    // std::cerr << "SerialHandler: Checksum mismatch - packet discarded.\n";
                }
                phase = Phase::WAIT_START;
                break;
            }
        }
    }
}


// ---- Private - checksum -----------------------------------------------------

uint8_t SerialHandler::ComputeChecksum(const uint8_t* data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}
