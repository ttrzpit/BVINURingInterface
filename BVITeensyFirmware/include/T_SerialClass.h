#pragma once

// =============================================================================
// T_SerialClass.h — Full-duplex USB CDC serial handler
//
// RX: parse incoming PcToTeensyPacket frames, write commanded PWM and state
//     to shared data so T_AmplifierClass can act on them.
//
// TX: build TeensyToPcPacket from shared data (encoder counts, current) and
//     send at 200 Hz when flagSendToPC is set.
//
// The two functions are now split:
//   ReadFromPC()  — call every loop() to parse incoming bytes
//   SendToPC()    — call from the 200 Hz flagged path in loop()
// =============================================================================

#include <Arduino.h>
#include "T_PacketTypes.h"
#include "T_SharedDataManager.h"


class T_SerialClass {
public:
    explicit T_SerialClass(SharedDataManager& dataHandle);

    /** @brief Open the USB serial port. */
    void Begin();

    /**
     * @brief Parse any available incoming bytes from the PC.
     *        When a valid PcToTeensyPacket is received, updates shared data
     *        (commandedPwm, state, packetIndex).
     *        Call every loop() iteration.
     */
    void ReadFromPC();

    /**
     * @brief Build and transmit one TeensyToPcPacket from shared data.
     *        Call at 200 Hz from the flagged path in loop().
     */
    void SendToPC();

    bool IsConnected() const { return connected_; }

private:
    uint8_t ComputeChecksum(const uint8_t* data, size_t len) const;

    enum class RxPhase : uint8_t { WAIT_START, READ_PAYLOAD, READ_CHECKSUM };

    RxPhase  rxPhase_ = RxPhase::WAIT_START;
    uint8_t  rxBuf_[sizeof(PcToTeensyPacket)];
    uint8_t  rxIdx_   = 0;

    bool     connected_ = false;

    SharedDataManager&           dataHandle_;
    std::shared_ptr<ManagedData> shared_;
};
