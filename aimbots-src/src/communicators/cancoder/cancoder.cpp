#include "cancoder.hpp"

#include "modm/architecture/interface/can_message.hpp"
#include "modm/math/geometry/angle.hpp"
#include "tap/architecture/clock.hpp"

namespace src::communicators::cancoder
{

using tap::arch::clock::getTimeMilliseconds;

CANcoder::FrameListener::FrameListener(tap::Drivers* drivers,
                                        uint32_t id,
                                        tap::can::CanBus bus,
                                        CANcoder* parent,
                                        FrameKind kind)
    : tap::can::CanRxListener(drivers, id, bus),
      parent(parent), kind(kind)
{
}

void CANcoder::FrameListener::processMessage(const modm::can::Message& message)
{
    parent->handleFrame(kind, message);
}

CANcoder::CANcoder(tap::Drivers* drivers, tap::can::CanBus bus, uint8_t deviceId)
    : deviceId(deviceId & 0x3Fu),
      sensorListener(drivers, SENSOR_BASE_ID | (deviceId & 0x3Fu),
                     bus, this, FrameKind::SENSOR),
      auxListener(drivers, AUX_BASE_ID | (deviceId & 0x3Fu),
                  bus, this, FrameKind::AUX),
      versionListener(drivers, VERSION_BASE_ID | (deviceId & 0x3Fu),
                      bus, this, FrameKind::VERSION)
{
}

void CANcoder::initialize()
{
    sensorFrameCount = 0;
    auxFrameCount = 0;
    lastSensorTimeMs = 0;
    lastAuxTimeMs = 0;
    sensorReceived = false;
    auxReceived = false;
    versionReceived = false;
    absolutePositionRotations = 0.0f;
    positionRotations = 0.0f;
    positionSinceBootRotations = 0.0f;
    filteredVelocityRps = 0.0f;
    unfilteredVelocityRps = 0.0f;

    sensorListener.attachSelfToRxHandler();
    auxListener.attachSelfToRxHandler();
    versionListener.attachSelfToRxHandler();
}

uint64_t CANcoder::asLittleEndian64(const uint8_t* data)
{
    uint64_t value = 0;
    for (uint8_t i = 0; i < 8; ++i)
    {
        value |= static_cast<uint64_t>(data[i]) << (8u * i);
    }
    return value;
}

int32_t CANcoder::signExtend(uint32_t value, uint8_t width)
{
    const uint32_t signMask = 1u << (width - 1u);
    const int32_t decoded = static_cast<int32_t>(value);
    return (value & signMask)
               ? decoded - static_cast<int32_t>(1u << width)
               : decoded;
}

void CANcoder::handleFrame(FrameKind kind, const modm::can::Message& message)
{
    if (kind == FrameKind::VERSION)
    {
        if (message.getLength() < 4) return;
        firmwareBuild = message.data[0];
        firmwareBugfix = message.data[1];
        firmwareMinor = message.data[2];
        firmwareMajor = message.data[3];
        versionReceived = true;
        return;
    }

    if (message.getLength() != 8) return;
    const uint64_t bits = asLittleEndian64(message.data);
    const uint32_t now = getTimeMilliseconds();

    if (kind == FrameKind::SENSOR)
    {
        // 0x05042CC0, 29-bit EXTENDED frame, 8-byte LITTLE-ENDIAN payload.
        // Field       Bits     Width    Signed    Scale / turns or RPS
        // Velocity    0..18      19       yes     /512 RPS (filtered)
        // Position   19..45      27       yes     /4096 turns (continuous)
        // Absolute   46..58      13       yes     /4096 turns (wrapped)
        // Reserved   59..63       5       ?      constant 0b00100 in captures
        const uint32_t rawVelocity = static_cast<uint32_t>((bits >> 0u) & 0x7FFFFu);
        const uint32_t rawPosition = static_cast<uint32_t>((bits >> 19u) & 0x7FFFFFFu);
        const uint32_t rawAbsolute = static_cast<uint32_t>((bits >> 46u) & 0x1FFFu);

        filteredVelocityRps = static_cast<float>(signExtend(rawVelocity, 19)) / 512.0f;
        positionRotations = static_cast<float>(signExtend(rawPosition, 27)) / 4096.0f;
        absolutePositionRotations = static_cast<float>(signExtend(rawAbsolute, 13)) / 4096.0f;

        lastSensorTimeMs = now;
        sensorReceived = true;
        ++sensorFrameCount;
    }
    else // FrameKind::AUX
    {
        // 0x05043440, at ~4 Hz, 8-byte little-endian payload.
        // Bits  0..15: signed 16-bit unfiltered velocity, ~0.25 RPS per count.
        // Bits 16..42: signed 27-bit *relative since CANcoder boot*, /4096 turns.
        // Bits 43..63: not identified; zero in our recorded test cases.
        const uint32_t rawUnfiltered = static_cast<uint32_t>(bits & 0xFFFFu);
        const uint32_t rawSinceBoot = static_cast<uint32_t>((bits >> 16u) & 0x7FFFFFFu);

        unfilteredVelocityRps = static_cast<float>(signExtend(rawUnfiltered, 16)) / 4.0f;
        positionSinceBootRotations = static_cast<float>(signExtend(rawSinceBoot, 27)) / 4096.0f;

        lastAuxTimeMs = now;
        auxReceived = true;
        ++auxFrameCount;
    }
}

float CANcoder::getAbsolutePositionRadians() const
{
    return getAbsolutePositionRotations() * M_TWOPI;
}
float CANcoder::getAbsolutePositionDegrees() const
{
    return getAbsolutePositionRotations() * 360.0f;
}
float CANcoder::getPositionRadians() const
{
    return getPositionRotations() * M_TWOPI;
}
float CANcoder::getPositionDegrees() const
{
    return getPositionRotations() * 360.0f;
}
float CANcoder::getPositionSinceBootRadians() const
{
    return getPositionSinceBootRotations() * M_TWOPI;
}
float CANcoder::getPositionSinceBootDegrees() const
{
    return getPositionSinceBootRotations() * 360.0f;
}
float CANcoder::getFilteredVelocityRPM() const
{
    return getFilteredVelocityRotationsPerSecond() * 60.0f;
}
float CANcoder::getFilteredVelocityRadiansPerSecond() const
{
    return getFilteredVelocityRotationsPerSecond() * M_TWOPI;
}
float CANcoder::getUnfilteredVelocityRPM() const
{
    return getUnfilteredVelocityRotationsPerSecond() * 60.0f;
}
float CANcoder::getUnfilteredVelocityRadiansPerSecond() const
{
    return getUnfilteredVelocityRotationsPerSecond() * M_TWOPI;
}

bool CANcoder::hasFreshSensorData(uint32_t timeoutMs) const
{
    return sensorReceived &&
           (getTimeMilliseconds() - lastSensorTimeMs <= timeoutMs);
}

bool CANcoder::hasFreshAuxData(uint32_t timeoutMs) const
{
    return auxReceived &&
           (getTimeMilliseconds() - lastAuxTimeMs <= timeoutMs);
}

}  // namespace src::communicators::cancoder
