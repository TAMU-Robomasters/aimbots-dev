#include "power_can_communicator.hpp"

#include "drivers.hpp"

namespace src::Informants::PowerComms{



PowerCommunicator::PowerCommunicator(src::Drivers* drivers, CANBus bus)
    : drivers(drivers),
      bus(bus),
      disconnectedTimeout(COMMS_DISCONNECTED_TIMEOUT),
      powerDataRXHandler(drivers, static_cast<uint32_t>(CanID::Power), bus, this, &PowerCommunicator::handlePowerDataRX) {}


int current1Display = 0;
void PowerCommunicator::handlePowerDataRX(const modm::can::Message& message) {
    PowerMessageData const* data = reinterpret_cast<PowerMessageData const*>(message.data);
    current1Display = data->current1;
}

void PowerCommunicator::init() {
    powerDataRXHandler.attachSelfToRxHandler();
}


int powerRequestDataDisplay = 0;
void PowerCommunicator::requestTest() {
    if(sendTimer.execute()) {
        modm::can::Message msg(static_cast<uint32_t>(CanID::Power), 1);
        msg.setExtended(false);
        msg.data[0] = 0b00101;
        drivers->can.sendMessage(bus, msg);

        powerRequestDataDisplay = msg.data[0];

        powerRequestData = 0;
        sendTimer.restart();
    }
    
}
PowerCommunicator::RXHandler::RXHandler( // src::Drivers* drivers, uint32_t id, CANBus bus, PowerCommunicator* ctx, CANListenerProc proc
    src::Drivers* drivers,
    uint32_t id,
    CANBus bus,
    PowerCommunicator* ctx,
    CANListenerProc proc)
    : CanRxListener(drivers, id, bus),
      ctx(ctx),
      proc(proc) {}

void PowerCommunicator::RXHandler::processMessage(modm::can::Message const& msg) { 
    (ctx->*proc)(msg);

}


} // namespace src::Informants::PowerComms

