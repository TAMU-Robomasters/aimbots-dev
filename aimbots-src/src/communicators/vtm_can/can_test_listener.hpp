#ifndef CAN351LISTENER_HPP_
#define CAN351LISTENER_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/can/can_rx_listener.hpp"

// this is a debug class for seeing if non DJI CAN devices (non DJI-specific CAN ID's) can communicate with the dev board
// they can :)

namespace src::communicators::vtm_can
{
class Can351Listener : public tap::can::CanRxListener
{
public:

    bool attachedDisplay = false;
    explicit Can351Listener(tap::Drivers* drivers)
        : tap::can::CanRxListener(drivers, 0x351, tap::can::CanBus::CAN_BUS1)
    {}

    void initialize() { 
        attachedDisplay = true;
        attachSelfToRxHandler(); 
    }

    void processMessage(const modm::can::Message& msg) override
    {
        rxCount++;
        lastLen = msg.getLength();
        lastByte0 = msg.data[0];
        lastByte1 = msg.data[1];
    }

    volatile uint32_t rxCount = 0;
    volatile uint8_t lastLen = 0;
    volatile uint8_t lastByte0 = 0;
    volatile uint8_t lastByte1 = 0;
};
}
#endif