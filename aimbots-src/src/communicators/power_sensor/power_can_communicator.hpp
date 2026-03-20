#pragma once

#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/sensors/current/current_sensor_interface.hpp"
#include "tap/communication/sensors/voltage/voltage_sensor_interface.hpp"
#include "tap/drivers.hpp"

#include "modm/architecture/interface/can_message.hpp"

namespace src::Informants::PowerComms
{
static constexpr uint16_t CHASSIS_SENSOR_CAN_ID = 0x446;

class PowerSensor
    : public tap::can::CanRxListener,
      public tap::communication::sensors::voltage::VoltageSensorInterface,
      public tap::communication::sensors::current::CurrentSensorInterface
{
public:
    PowerSensor(tap::Drivers* drivers, tap::can::CanBus canBus);

    void processMessage(const modm::can::Message& message) override;

    mockable void initialize();

    float getVoltageMv() const override { return this->voltage; };
    float getCurrentMa() const override { return this->current; };
    float getPowerWatts() const {return this->power;};

    void update() override{};

    void requestTest();

    bool isOnline() const { return !this->heartbeat.isExpired(); }

private:
    float voltage = 0;
    float current = 0;
    float power = 0;

    tap::arch::MilliTimeout heartbeat;
    tap::arch::PeriodicMilliTimer sendDataTimer;
};
}  


