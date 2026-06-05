#ifndef ESP_POWER_SENSOR_HPP_
#define ESP_POWER_SENSOR_HPP_

#include <cstdint>

#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/can/can_rx_listener.hpp"

namespace src::communicators::esp_power_sensor
{
/**
 * ESP32-S3 INA260 CAN power-sensor CAN bus reader
 *
 *  Dev board sends an RTR poll frame on CAN2 every 2 ms
 *  ESP32 replies with one 8-byte standard CAN data frame
 *
 * response payload, little-endian:
 *   CAN_ID_POWER_RESP (standard 11-bit), DLC=8
 *    bytes 0-1: voltage_mV              uint16_t   1 mV/bit
 *    bytes 2-3: current_mA              int16_t    1 mA/bit, signed
 *    bytes 4-5: instantaneous_power_cW  uint16_t   0.01 W/bit
 *    bytes 6-7: remaining_energy_mJ     uint16_t   1 mJ/bit
 *
 */
class EspPowerSensor
{
public:
    static constexpr tap::can::CanBus BUS = tap::can::CanBus::CAN_BUS2; //chassis bus

    static constexpr uint16_t CAN_ID_POWER_POLL_RTR = 0x354;  //dev board -> ESP32 (RTR)
    static constexpr uint16_t CAN_ID_POWER_RESP     = 0x355;  //ESP32 -> dev board (data)

    static constexpr uint8_t PAYLOAD_LEN = 8;

    explicit EspPowerSensor(tap::Drivers* drivers);

    void initialize();

    void read();

    bool hasFreshPacket(uint32_t timeoutMs = 20) const;

    uint32_t getUpdateCounter() const { return updateCounter; }
    uint32_t getLastPacketTimeMs() const { return lastPacketTimeMs; }

    void setPollPeriodMs(uint32_t ms) { pollPeriodMs = ms; }

    //not sure if integers or floats are more preferrable

    //float getters
    float getVoltage() const { return voltage_mV * 0.001f; }
    float getCurrent() const { return current_mA * 0.001f; }
    float getPower() const { return power_cW * 0.01f; }
    float getEnergyBuffer() const { return energyBuffer_mJ * 0.001f; }

    //integer getters
    uint16_t getVoltageMilliVolts() const { return voltage_mV; }
    int16_t  getCurrentMilliAmps() const { return current_mA; }
    uint16_t getPowerCentiWatts() const { return power_cW; }
    uint16_t getEnergyBufferMilliJoules() const { return energyBuffer_mJ; }

private:
    class RespListener : public tap::can::CanRxListener
    {
    public:
        RespListener(
            tap::Drivers* drivers,
            uint32_t id,
            tap::can::CanBus bus,
            EspPowerSensor* parent);

        void processMessage(const modm::can::Message& rxMessage) override;

    private:
        EspPowerSensor* parent;
    };

    void handleRespFrame(const modm::can::Message& rxMessage);
    void sendPollRtrIfDue();

    static uint16_t readU16LE(const uint8_t* src);
    static int16_t readI16LE(const uint8_t* src);

private:
    tap::Drivers* drivers;

    RespListener respListener;

    uint16_t voltage_mV = 0;
    int16_t current_mA = 0;
    uint16_t power_cW = 0;
    uint16_t energyBuffer_mJ = 60000;

    uint32_t updateCounter = 0;
    uint32_t lastPacketTimeMs = 0;

    uint32_t lastPollTimeMs = 0;
    uint32_t pollPeriodMs = 2;  //500 Hz poll rate
};

}  // namespace src::communicators::esp_power_sensor

#endif  // ESP_POWER_SENSOR_HPP_
