#include "power_can_communicator.hpp"

namespace src::Informants::PowerComms
{
PowerSensor::PowerSensor(tap::Drivers* drivers, tap::can::CanBus canBus)
    : tap::can::CanRxListener(drivers, CHASSIS_SENSOR_CAN_ID, canBus)
{
}

void PowerSensor::processMessage(const modm::can::Message& message)
{
    this->heartbeat.restart(100);
    this->voltage = message.data[1] << 8 | message.data[0];
    this->current = message.data[3] << 8 | message.data[2];
}

void PowerSensor::initialize()
{
    this->attachSelfToRxHandler();
    this->heartbeat.restart(0);
}
}