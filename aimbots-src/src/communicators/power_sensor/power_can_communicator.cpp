#include "power_can_communicator.hpp"


#include "drivers.hpp"

namespace src::Informants::PowerComms{

PowerCommunicator::PowerCommunicator(src::Drivers* drivers, CANBus bus)
    : drivers(drivers),
      bus(bus),
      disconnectedTimeout(COMMS_DISCONNECTED_TIMEOUT) {}

PowerCommunicator::RXHandler::RXHandler( // src::Drivers* drivers, uint32_t id, CANBus bus, PowerCommunicator* ctx, CANListenerProc proc
    src::Drivers* drivers,
    uint32_t id,
    CANBus bus,
    PowerCommunicator* ctx,
    CANListenerProc proc)
    : CanRxListener(drivers, id, bus),
      ctx(ctx),
      proc(proc) {}
}