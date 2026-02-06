#include "power_can_communicator.hpp"


#include "drivers.hpp"


TurretCommunicator::RXHandler::RXHandler(
    src::Drivers* drivers,
    uint32_t id,
    CANBus bus,
    TurretCommunicator* ctx,
    CANListenerProc proc)
    : CanRxListener(drivers, id, bus),
      ctx(ctx),
      proc(proc) {}