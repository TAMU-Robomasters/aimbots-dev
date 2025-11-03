#pragma once

#include <array>
#include "drivers.hpp"
#include "subsystems/jetson/jetson_constants.hpp"
#include "jetson_message.hpp"

/*
TODO: look into ref_serial from taproot. UW inherents a ref_serial class for there communication
If we decide on this then see if they put that ref_serial class as a singleton in their drivers
*/

//TODO: look into why shakira command has ballistics

#ifdef JETSON_COMPATIBLE

#define READ(data, length) drivers->uart.read(src::Jetson::JETSON_UART_PORT, data, length)
#define INIT_UART() drivers->uart.init<src::Jetson::JETSON_UART_PORT, src::Jetson::JETSON_BAUD_RATE>()

namespace src::Communication {

template <size_t messageCount>
class JetsonMessageHandler {
    src::Drivers* drivers;
    std::array<MessageFromJetson* const, messageCount> jetsonMessages = {};
public:
    JetsonMessageHandler(const std::array<MessageFromJetson* const, messageCount>& jetsonMessages)
    : jetsonMessages(jetsonMessages)
    {
        INIT_UART();
    };

    void checkForMessage() {
        uint8_t header;
        READ(&header, 1);
        if (header != src::Jetson::jetsonMessageHeader) return; // no message found

        uint8_t messageID;
        READ(&messageID, 1);
        for (MessageFromJetson* const message : jetsonMessages) {
            if (messageID == message->getID()) {
                message->processMessage();
                break;
            } 
        }
    }
};    

} // namespace src::Communication

#endif // #ifdef JETSON_COMPATIBLE
