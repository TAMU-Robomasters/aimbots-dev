#include "power_can_communicator.hpp"

#include "tap/communication/sensors/imu/bmi088/bmi088.hpp"

#include "drivers.hpp"

namespace src::Informants::PowerComms{

PowerCommunicator::RXHandler::RXHandler( // src::Drivers* drivers, uint32_t id, CANBus bus, PowerCommunicator* ctx, CANListenerProc proc
    src::Drivers* drivers,
    uint32_t id,
    CANBus bus,
    PowerCommunicator* ctx,
    CANListenerProc proc)
    : CanRxListener(drivers, id, bus),
      ctx(ctx),
      proc(proc) {}

PowerCommunicator::PowerCommunicator(src::Drivers* drivers, CANBus bus)
    : drivers(drivers),
      bus(bus),
      disconnectedTimeout(COMMS_DISCONNECTED_TIMEOUT),
      powerDataRXHandler(drivers, static_cast<uint32_t>(CanID::Power), bus, this, &PowerCommunicator::handlePowerDataRX) {}


void PowerCommunicator::handlePowerDataRX(const modm::can::Message& message) {
    PowerMessageData const* data = reinterpret_cast<PowerMessageData const*>(message.data);
    return;
}

void PowerCommunicator::init() {
    powerDataRXHandler.attachSelfToRxHandler();
}

void PowerCommunicator::request() {
    if(sendTimer.execute()) {
        modm::can::Message msg(static_cast<uint32_t>(CanID::Power), 1);
        msg.setExtended(false);
        msg.data[0] = powerRequestData;
        drivers->can.sendMessage(bus, msg);

        powerRequestData = 0;
        sendTimer.restart();
    }
    
}

void PowerCommunicator::RXHandler::processMessage(modm::can::Message const& msg) { (ctx->*proc)(msg); }


} // namespace src::Informants::PowerComms

