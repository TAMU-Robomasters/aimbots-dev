#include "dt7_remote.hpp"

#ifdef TARGET_TRAINING

#include "tap/architecture/clock.hpp"

#include "drivers.hpp"

// MCUViewer watch variables. Add these to the watch window to see your parser working.
uint32_t framesParsedDisplay = 0;  // should climb by ~70 every second with the remote on
uint32_t badFramesDisplay = 0;     // should stay at (or very near) 0
float rightVerticalDisplay = 0.0f;

namespace src::Training {

using tap::communication::serial::Uart;

void Dt7Remote::initialize() {
    // TODO(week2): start the UART the DR16 receiver is plugged into (the board's DBUS port is Uart3).
    drivers->uart.init<Uart::UartPort::Uart3, 100000, Uart::Parity::Even>();
    // Find the baud rate and parity in the protocol document, then fill in the ??? below:
    //     drivers->uart.init<Uart::UartPort::Uart3, ???, Uart::Parity::???>();
}

void Dt7Remote::read() {
    // TODO(week2): turn the stream of bytes into frames.
    //
    // 1. Read bytes one at a time until the UART is empty:
    //        uint8_t byte;
    //        while (drivers->uart.read(Uart::UartPort::Uart3, &byte)) { ... }
    uint32_t now = tap::arch::clock::getTimeMilliseconds();
    uint8_t byte;
    uint32_t lastByteMs = 0;
    while (drivers->uart.read(Uart::UartPort::Uart3, &byte)) {
         if (now - lastByteMs > FRAME_GAP_MS) {
            bytesReceived = 0;   // new frame starts now
        }
        lastByteMs = now;

        // store byte
        if (bytesReceived < FRAME_LENGTH) {
            rxBuffer[bytesReceived++] = byte;
        }

        // 3. Full frame received
        if (bytesReceived == FRAME_LENGTH) {
            bool good = parseFrame();
            bytesReceived = 0;

            if (good) {
                // notify command mapper
                drivers->commandMapper.handleKeyStateChange(
                    0,
                    SwitchState::UNKNOWN,
                    rightSwitch,
                    false,
                    false
                );
            }
        }
    }
    // 2. Frame sync: if more than FRAME_GAP_MS passed since the previous byte, this byte is the
    //    START of a new frame, so throw away any partial frame (bytesReceived = 0).
    //    Time: tap::arch::clock::getTimeMilliseconds() (returns uint32_t).
    //
    // 3. Store the byte in rxBuffer. When you have FRAME_LENGTH bytes, call parseFrame(), then start
    //    over. If the frame was good, tell the command mapper about the new switch position:
    //        drivers->commandMapper.handleKeyStateChange(0, SwitchState::UNKNOWN, rightSwitch, false, false);
    //    (no keyboard keys, left switch unknown, no mouse buttons). That's what makes your
    //    HoldCommandMappings work!
    //
    // 4. Disconnect: if connected and no good frame for DISCONNECT_TIMEOUT_MS, set connected = false,
    //    call resetValues(), and tell the command mapper everything is UNKNOWN so commands stop.
    if (connected && (now - lastGoodFrameMs > DISCONNECT_TIMEOUT_MS)) {
        connected = false;
        resetValues();

        drivers->commandMapper.handleKeyStateChange(
            0,
            SwitchState::UNKNOWN,
            SwitchState::UNKNOWN,
            false,
            false
        );
    }
}   

bool Dt7Remote::parseFrame() {
    // TODO(week2): decode ch0 (right horizontal), ch1 (right vertical) and s1 (right switch)
    // from rxBuffer. Use the bit-layout table in dbus_protocol_en.md.
    //
    // Then validate BEFORE you store anything:
    //   - each channel must be within CHANNEL_CENTER +- CHANNEL_MAX_OFFSET
    //   - the switch must be 1, 2 or 3
    // Bad frame -> badFramesDisplay++, return false, keep the old values.
    // Good frame -> store (channel - CHANNEL_CENTER) in rightHorizontal / rightVertical,
    //               static_cast the switch into SwitchState, framesParsedDisplay++,
    //               update lastGoodFrameMs and connected, return true.
    // Decode ch0 (bits 0–10)
    uint16_t ch0 = (rxBuffer[0] | (rxBuffer[1] << 8)) & 0x7FF;

    // Decode ch1 (bits 11–21)
    uint16_t ch1 = ((rxBuffer[1] >> 3) | (rxBuffer[2] << 5)) & 0x7FF;

    // Decode s1 (bits 44–45)
    uint8_t s1 = (rxBuffer[5] >> 4) & 0x03;

    // Validate
    if (ch0 < CHANNEL_MIN || ch0 > CHANNEL_MAX ||
        ch1 < CHANNEL_MIN || ch1 > CHANNEL_MAX ||
        s1 == 0) {
        badFramesDisplay++;
        return false;
    }

    // Store values
    rightHorizontal = ch0 - CHANNEL_CENTER;
    rightVertical   = ch1 - CHANNEL_CENTER;
    rightSwitch     = static_cast<SwitchState>(s1);

    // MCUViewer
    framesParsedDisplay++;
    rightVerticalDisplay = rightVertical / float(CHANNEL_MAX_OFFSET);

    // mark good frame
    lastGoodFrameMs = tap::arch::clock::getTimeMilliseconds();
    connected = true;

    return true;
}

void Dt7Remote::resetValues() {
    rightHorizontal = 0;
    rightVertical = 0;
    rightSwitch = SwitchState::UNKNOWN;
}

}  // namespace src::Training

#endif  // TARGET_TRAINING
