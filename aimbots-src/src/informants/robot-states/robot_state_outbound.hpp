#pragma once

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
#include "robot_state_interface.hpp"

namespace src::robotStates {

class RobotStateOutBound : public modm::pt::Protothread {
private:
    src::Drivers* drivers;
    tap::communication::serial::RefSerialTransmitter refSerial;
    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;

    RobotStates* robotStateInterface;

    // #ifdef TARGET_SENTRY
    static constexpr uint16_t SENTRY_REQUEST_ROBOT_ID = 0x200;
    MessageType lastSentMessage;
#ifdef TARGET_SENTRY
    lastSentMessage = MessageType::TEAM_MESSAGE;
#else
    lastSentMessage = MessageType::ROBOT_STATE;
#endif
    uint32_t queuedMessageType{};
    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;

    inline void getNextMessageToSend() {
        // either no queued messages or lastSentMessage is the only message to send, return
        // w/o trying to find a new message
        if (queuedMessageType == 0) {
            return;
        }

        if ((queuedMessageType & ~(1 << static_cast<uint8_t>(lastSentMessage))) == 0) {
            return;
        }

        // otherwise, iterate through message types until you find one that is queued
        auto nextMessageType = [](MessageType type) {
            return static_cast<MessageType>((static_cast<uint8_t>(type) + 1) % static_cast<uint8_t>(MessageType::NUM_MESSAGE_TYPES));
        };

        lastSentMessage = nextMessageType(lastSentMessage);

        while ((queuedMessageType & (1 << static_cast<uint8_t>(lastSentMessage))) == 0) {
            lastSentMessage = nextMessageType(lastSentMessage);
        }
    }

    // #endif
public:
    RobotStateOutBound(src::Drivers* drivers);

    bool send();

    void updateQue();
};

}  // namespace src::robotStates
