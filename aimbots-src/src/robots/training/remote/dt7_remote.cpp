#include "dt7_remote.hpp"

#ifdef TARGET_TRAINING

#include "tap/architecture/clock.hpp"

#include "drivers.hpp"

// MCUViewer watch variables. Add these to the watch window to see your parser working.
uint32_t framesParsedDisplay = 0;  // should climb by ~140 every second with the remote on
uint32_t badFramesDisplay = 0;     // should stay at (or very near) 0
float rightVerticalDisplay = 0.0f;

namespace src::Training {

using tap::communication::serial::Uart;

void Dt7Remote::initialize() {
    drivers->uart.init<Uart::UartPort::Uart3, 100000, Uart::Parity::Even>();
}

void Dt7Remote::read() {
    uint8_t byte;
    while (drivers->uart.read(Uart::UartPort::Uart3, &byte)) {
        uint32_t now = tap::arch::clock::getTimeMilliseconds();


        if (now - lastByteTimeMs > FRAME_GAP_MS) {
            bytesReceived = 0;
        }
        lastByteTimeMs = now;

        rxBuffer[bytesReceived] = byte;
        bytesReceived++;

        if (bytesReceived == FRAME_LENGTH) {
            if (parseFrame()) {
                drivers->commandMapper.handleKeyStateChange(0, SwitchState::UNKNOWN, rightSwitch, false, false);
            }
            bytesReceived = 0;
        }
    }


    uint32_t now = tap::arch::clock::getTimeMilliseconds();
    if (connected && (now - lastGoodFrameMs > DISCONNECT_TIMEOUT_MS)) {
        connected = false;
        resetValues();
        drivers->commandMapper.handleKeyStateChange(0, SwitchState::UNKNOWN, SwitchState::UNKNOWN, false, false);
    }
}

bool Dt7Remote::parseFrame() {
    uint16_t ch0 = (rxBuffer[0] | (rxBuffer[1] << 8)) & 0x7FF;
    uint16_t ch1 = ((rxBuffer[1] >> 3) | (rxBuffer[2] << 5)) & 0x7FF;
    uint8_t s2 = (rxBuffer[5] >> 4) & 0b11;

    constexpr uint16_t CHANNEL_MIN = CHANNEL_CENTER - CHANNEL_MAX_OFFSET;
    constexpr uint16_t CHANNEL_MAX = CHANNEL_CENTER + CHANNEL_MAX_OFFSET;
    if (ch0 < CHANNEL_MIN || ch0 > CHANNEL_MAX || ch1 < CHANNEL_MIN || ch1 > CHANNEL_MAX || s2 == 0) {
        badFramesDisplay++;
        return false;
    }

    rightHorizontal = static_cast<int16_t>(ch0) - static_cast<int16_t>(CHANNEL_CENTER);
    rightVertical = static_cast<int16_t>(ch1) - static_cast<int16_t>(CHANNEL_CENTER);
    rightSwitch = static_cast<SwitchState>(s2);

    lastGoodFrameMs = tap::arch::clock::getTimeMilliseconds();
    connected = true;

    framesParsedDisplay++;
    rightVerticalDisplay = getRightVertical();
    return true;
}

void Dt7Remote::resetValues() {
    rightHorizontal = 0;
    rightVertical = 0;
    rightSwitch = SwitchState::UNKNOWN;
}

}  // namespace src::Training

#endif  // TARGET_TRAINING
