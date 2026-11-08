#ifndef SRC_COMMUNICATORS_CANCODER_CANCODER_HPP_
#define SRC_COMMUNICATORS_CANCODER_CANCODER_HPP_

#include <cstdint>

#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/can/can_rx_listener.hpp"

namespace src::communicators::cancoder
{
/**
 * Passive CTRE CANcoder receiver, empirically decoded from firmware 26.1.1.0
 * using standard (non-FD) 8-byte payloads with 29-bit EXTENDED CAN IDs.
 *
 * Observed automatic broadcast frames for CTRE device number 0:
 *    0x05042CC0 (100 Hz): velocity (19-bit signed, /512 RPS),
 *                         position (27-bit signed, /4096 turns),
 *                         absolute position (13-bit signed, /4096 turns).
 *    0x05043440 (4 Hz):   unfiltered velocity (16-bit signed, /4 RPS),
 *                         position since boot (27-bit signed, /4096 turns).
 *    0x05046BC0 (4 Hz):   firmware version in bytes 0-3 (build, fix, minor, major).
 *
 */
class CANcoder
{
public:
    static constexpr tap::can::CanBus DEFAULT_BUS = tap::can::CanBus::CAN_BUS2;

    static constexpr uint32_t SENSOR_BASE_ID = 0x05042CC0u;
    static constexpr uint32_t AUX_BASE_ID    = 0x05043440u;
    static constexpr uint32_t VERSION_BASE_ID = 0x05046BC0u;

    // CTRE's extended CAN identifiers use the bottom six bits for the device number
    static constexpr uint32_t SENSOR_TIMEOUT_MS = 100u;  // 100 Hz sensor frame
    static constexpr uint32_t AUX_TIMEOUT_MS = 750u;     // 4 Hz auxiliary frame

    explicit CANcoder(tap::Drivers* drivers,
                     tap::can::CanBus bus = DEFAULT_BUS,
                     uint8_t deviceId = 0);

    void initialize();

    // Absolute (wrapped) encoder angle, normally in [-0.5,+0.5) rotations
    // for the observed magnet configuration. The wrap point is configurable
    // in Phoenix and can change if the CANcoder is reconfigured.
    float getAbsolutePositionRotations() const { return absolutePositionRotations; }
    float getAbsolutePositionRadians() const;
    float getAbsolutePositionDegrees() const;

    // Continuous position signal which starts at the absolute angle on
    // encoder power-up and then tracks multi-turn rotation.
    float getPositionRotations() const { return positionRotations; }
    float getPositionRadians() const;
    float getPositionDegrees() const;

    // Actual position SINCE CANcoder BOOT
    // Comes from 0x05043440 at ~4 Hz and resets to zero after encoder reboot.
    float getPositionSinceBootRotations() const { return positionSinceBootRotations; }
    float getPositionSinceBootRadians() const;
    float getPositionSinceBootDegrees() const;

    // Filtered velocity at ~100 Hz.
    float getFilteredVelocityRotationsPerSecond() const { return filteredVelocityRps; }
    float getFilteredVelocityRPM() const;
    float getFilteredVelocityRadiansPerSecond() const;

    // Unfiltered 0x05043440 velocity at ~4 Hz; quantized in 0.25 RPS.
    float getUnfilteredVelocityRotationsPerSecond() const { return unfilteredVelocityRps; }
    float getUnfilteredVelocityRPM() const;
    float getUnfilteredVelocityRadiansPerSecond() const;

    float getVelocityRPM() const { return getFilteredVelocityRPM(); }
    float getVelocityRadiansPerSecond() const { return getFilteredVelocityRadiansPerSecond(); }

    bool hasFreshSensorData(uint32_t timeoutMs = SENSOR_TIMEOUT_MS) const;
    bool hasFreshAuxData(uint32_t timeoutMs = AUX_TIMEOUT_MS) const;
    bool hasFirmwareVersion() const { return versionReceived; }
    uint8_t getFirmwareMajor() const { return firmwareMajor; }
    uint8_t getFirmwareMinor() const { return firmwareMinor; }
    uint8_t getFirmwareBugfix() const { return firmwareBugfix; }
    uint8_t getFirmwareBuild() const { return firmwareBuild; }

    uint32_t getSensorFrameCount() const { return sensorFrameCount; }
    uint32_t getAuxFrameCount() const { return auxFrameCount; }
    uint32_t getLastSensorTimeMs() const { return lastSensorTimeMs; }
    uint32_t getLastAuxTimeMs() const { return lastAuxTimeMs; }
    uint8_t getDeviceId() const { return deviceId; }

private:
    enum class FrameKind : uint8_t { SENSOR, AUX, VERSION };

    class FrameListener : public tap::can::CanRxListener
    {
    public:
        FrameListener(tap::Drivers* drivers, uint32_t id,
                      tap::can::CanBus bus, CANcoder* parent, FrameKind kind);
        void processMessage(const modm::can::Message& message) override;

    private:
        CANcoder* parent;
        FrameKind kind;
    };

    void handleFrame(FrameKind kind, const modm::can::Message& message);
    static uint64_t asLittleEndian64(const uint8_t* data);
    static int32_t signExtend(uint32_t value, uint8_t width);

    const uint8_t deviceId;
    FrameListener sensorListener;
    FrameListener auxListener;
    FrameListener versionListener;

    float absolutePositionRotations = 0.0f;
    float positionRotations = 0.0f;
    float positionSinceBootRotations = 0.0f;
    float filteredVelocityRps = 0.0f;
    float unfilteredVelocityRps = 0.0f;

    uint32_t sensorFrameCount = 0;
    uint32_t auxFrameCount = 0;
    uint32_t lastSensorTimeMs = 0;
    uint32_t lastAuxTimeMs = 0;
    bool sensorReceived = false;
    bool auxReceived = false;
    bool versionReceived = false;
    uint8_t firmwareMajor = 0;
    uint8_t firmwareMinor = 0;
    uint8_t firmwareBugfix = 0;
    uint8_t firmwareBuild = 0;
};

}  // namespace src::communicators::cancoder

#endif  // SRC_COMMUNICATORS_CANCODER_CANCODER_HPP_
