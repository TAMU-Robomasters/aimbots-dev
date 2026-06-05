#include "esp_power_sensor.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "modm/architecture/interface/can_message.hpp"

namespace src::communicators::esp_power_sensor
{
using tap::arch::clock::getTimeMilliseconds;

EspPowerSensor::RespListener::RespListener(
    tap::Drivers* drivers,
    uint32_t id,
    tap::can::CanBus bus,
    EspPowerSensor* parent)
    : tap::can::CanRxListener(drivers, id, bus),
      parent(parent)
{
}

void EspPowerSensor::RespListener::processMessage(const modm::can::Message& rxMessage)
{
    parent->handleRespFrame(rxMessage);
}

EspPowerSensor::EspPowerSensor(tap::Drivers* drivers)
    : drivers(drivers),
      respListener(drivers, CAN_ID_POWER_RESP, BUS, this)
{
}

void EspPowerSensor::initialize()
{
    respListener.attachSelfToRxHandler();

    lastPollTimeMs = getTimeMilliseconds();
    lastPacketTimeMs = 0;
    updateCounter = 0;

    voltage_mV = 0;
    current_mA = 0;
    power_cW = 0;
    energyBuffer_mJ = 60000;
}

void EspPowerSensor::read()
{
    sendPollRtrIfDue();
}

bool EspPowerSensor::hasFreshPacket(uint32_t timeoutMs) const
{
    if (updateCounter == 0) return false;

    const uint32_t now = getTimeMilliseconds();
    return (now - lastPacketTimeMs) <= timeoutMs;
}

void EspPowerSensor::sendPollRtrIfDue()
{
    const uint32_t now = getTimeMilliseconds();
    if (now - lastPollTimeMs < pollPeriodMs) return;
    lastPollTimeMs = now;

    modm::can::Message msg(CAN_ID_POWER_POLL_RTR, 0);
    msg.setExtended(false);
    msg.setRemoteTransmitRequest(true);

    if (drivers->can.isReadyToSend(BUS))
    {
        (void)drivers->can.sendMessage(BUS, msg);
    }
}

void EspPowerSensor::handleRespFrame(const modm::can::Message& rxMessage)
{
    if (rxMessage.getLength() != PAYLOAD_LEN) return;

    voltage_mV      = readU16LE(&rxMessage.data[0]);
    current_mA      = readI16LE(&rxMessage.data[2]);
    power_cW        = readU16LE(&rxMessage.data[4]);
    energyBuffer_mJ = readU16LE(&rxMessage.data[6]);

    updateCounter++;
    lastPacketTimeMs = getTimeMilliseconds();
}

uint16_t EspPowerSensor::readU16LE(const uint8_t* src)
{
    return static_cast<uint16_t>(src[0])
        | (static_cast<uint16_t>(src[1]) << 8);
}

int16_t EspPowerSensor::readI16LE(const uint8_t* src)
{
    return static_cast<int16_t>(readU16LE(src));
}

}  // namespace src::communicators::esp_power_sensor
