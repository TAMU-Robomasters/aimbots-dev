#include "custom_controller.hpp"

#include <algorithm>

#include "drivers.hpp"
#include "communicators/vtm_can/vtm_can.hpp"

namespace src::communicators::custom_controller
{
// read an unsigned bitfield from a little-endian bitstream
static uint32_t readBitsLE(const std::array<uint8_t, 30>& bytes, uint16_t bitOffset, uint8_t bitLen)
{
    uint32_t out = 0;
    for (uint8_t i = 0; i < bitLen; i++)
    {
        const uint16_t b = bitOffset + i;
        const uint16_t byteIdx = b / 8;
        const uint8_t bitIdx = static_cast<uint8_t>(b % 8);
        const uint8_t bit = (bytes[byteIdx] >> bitIdx) & 0x01;
        out |= (static_cast<uint32_t>(bit) << i);
    }
    return out;
}

static int16_t signExtend12(uint16_t raw12)
{
    raw12 &= 0x0FFF;
    if (raw12 & 0x0800)
    {
        return static_cast<int16_t>(raw12 | 0xF000);
    }
    return static_cast<int16_t>(raw12);
}

static int16_t clampSigned12(int16_t v)
{
    return static_cast<int16_t>(std::clamp<int32_t>(v, -1024, 1024));
}

static uint16_t clampAnalog(uint16_t v)
{
    return static_cast<uint16_t>(std::min<uint32_t>(v, 2048));
}

void CustomController::initialize()
{
    updateCounter = 0;
    lastSeenVtmCounter = 0;
    data.fill(0);
    state = CustomControllerState{};
    disconnectTimeout.stop();
}

void CustomController::read()
{
    if (vtmCan == nullptr)
    {
        return;
    }

    vtmCan->read();

    const uint32_t vtmCounter = vtmCan->getUpdateCounter();
    if (vtmCounter == 0 || vtmCounter == lastSeenVtmCounter)
    {
        return;
    }

    lastSeenVtmCounter = vtmCounter;
    data = vtmCan->getPayload();
    updateCounter++;

    decodeStateFromData();

    disconnectTimeout.restart(CUSTOM_CONTROLLER_DISCONNECT_TIMEOUT_MS);
}

void CustomController::decodeStateFromData()
{
    // 12-bit signed joystick data
    state.joystick1X = clampSigned12(signExtend12(static_cast<uint16_t>(readBitsLE(data, 0, 12))));
    state.joystick1Y = clampSigned12(signExtend12(static_cast<uint16_t>(readBitsLE(data, 12, 12))));
    state.joystick2X = clampSigned12(signExtend12(static_cast<uint16_t>(readBitsLE(data, 24, 12))));
    state.joystick2Y = clampSigned12(signExtend12(static_cast<uint16_t>(readBitsLE(data, 36, 12))));

    // 12-bit unsigned analog input data
    state.analog1 = clampAnalog(static_cast<uint16_t>(readBitsLE(data, 48, 12)));
    state.analog2 = clampAnalog(static_cast<uint16_t>(readBitsLE(data, 60, 12)));

    // buttons/switches
    state.button1 = readBitsLE(data, 72, 1) != 0;
    state.button2 = readBitsLE(data, 73, 1) != 0;
    state.button3 = readBitsLE(data, 74, 1) != 0;
    state.button4 = readBitsLE(data, 75, 1) != 0;
    state.switch1 = readBitsLE(data, 76, 1) != 0;
    state.switch2 = readBitsLE(data, 77, 1) != 0;

    // engineer arm joints (6 x 12-bit signed)
    const uint16_t armBase = 78;
    for (int i = 0; i < 6; i++)
    {
        const uint16_t off = static_cast<uint16_t>(armBase + i * 12);
        const int16_t v = signExtend12(static_cast<uint16_t>(readBitsLE(data, off, 12)));
        state.armJoints[static_cast<size_t>(i)] = clampSigned12(v);
    }
}

bool CustomController::isConnected() const
{
    return !(disconnectTimeout.isStopped() || disconnectTimeout.isExpired());
}

}  // namespace src::communicators::custom_controller