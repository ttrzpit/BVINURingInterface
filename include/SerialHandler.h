#pragma once

// =============================================================================
// SerialHandler.h — Bi-directional serial communication with the Teensy
//
// Single full-duplex USB CDC port (/dev/ttyACM0).
//
// TX thread (200 Hz timer):
//   Wakes every 5 ms, reads the latest pending packet set by the main loop via
//   SetPendingTx(), stamps the packet_index, and sends. Rate is independent of
//   the camera frame rate so state changes propagate in ≤5 ms.
//
// RX thread (blocking read):
//   Runs a start-byte + fixed-payload + XOR-checksum state-machine parser.
//   Stores the latest valid TeensyToPcPacket for the main loop to read via
//   GetLatestPacket().
//
// Both threads start on Connect() and stop on Disconnect().
// =============================================================================

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "Config.h"
#include "PacketTypes.h"


class SerialHandler {
public:
    explicit SerialHandler(const SerialConfig& cfg);
    ~SerialHandler();

    /** @brief Initialize the handler. Does NOT open the port — call Connect() for that. */
    void start();

    /** @brief Disconnect if connected, then shut down cleanly. */
    void stop();

    /** @brief Open the port and launch both TX and RX threads. No-op if already connected. */
    void Connect();

    /** @brief Stop both threads and close the port. No-op if already disconnected. */
    void Disconnect();

    /**
     * @brief Update the pending TX packet. The TX timer thread reads this every
     *        5 ms and sends it automatically — do not call Send() directly.
     *        The packet_index field is managed internally and will be overwritten.
     */
    void SetPendingTx(const PcToTeensyPacket& cmd);

    /**
     * @brief Copy the most recently received valid packet into @p out.
     * @return true if a valid packet has been received; false if nothing yet.
     */
    bool GetLatestPacket(TeensyToPcPacket& out);

    /** @brief Measured TX rate in Hz (updated once per second by the TX thread). */
    float GetTxFrequency() const { return txFrequencyHz_.load(); }

    /** @brief The packet_index value stamped on the most recently sent packet. */
    uint8_t GetLastSentIndex() const { return lastSentIndex_.load(); }

    bool IsConnected() const { return fd_ != -1; }

    /**
     * @brief Low-level send — frames and writes one packet directly.
     *        Normally called by TxLoop; exposed for direct use if needed.
     */
    void Send(const PcToTeensyPacket& pkt);

private:
    void TxLoop();         // 200 Hz timer thread
    void ReceiveLoop();    // Blocking-read RX thread
    bool OpenPort();
    void ClosePort();

    static uint8_t ComputeChecksum(const uint8_t* data, size_t len);

    const SerialConfig& cfg_;
    int                 fd_ = -1;

    // ---- TX -----------------------------------------------------------------
    std::mutex            txMutex_;
    PcToTeensyPacket      pendingTx_ = {};         // Written by main, read by TxLoop
    std::atomic<float>    txFrequencyHz_{ 0.0f };
    std::atomic<uint8_t>  lastSentIndex_{ 0 };     // Index stamped on last sent packet
    std::thread           txThread_;

    // ---- RX -----------------------------------------------------------------
    std::mutex          rxMutex_;
    TeensyToPcPacket    latestRx_  = {};
    bool                rxReady_   = false;
    std::thread         receiveThread_;

    std::atomic<bool>   running_{ false };
};
