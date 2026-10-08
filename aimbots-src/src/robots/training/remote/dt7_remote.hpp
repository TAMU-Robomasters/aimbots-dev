#pragma once

#ifdef TARGET_TRAINING

#include <cstddef>
#include <cstdint>

#include "tap/communication/serial/remote.hpp"
#include "tap/util_macros.hpp"

// Forward declaration instead of #include "drivers.hpp": drivers.hpp will include THIS file, so
// including it back would be a loop. The .cpp is free to include drivers.hpp.
namespace src {
class Drivers;
}

namespace src::Training {

/**
 * @brief WEEK 2: your own parser for the DT7 remote / DR16 receiver (DBUS protocol).
 *
 * You only need the RIGHT stick (ch0, ch1) and the RIGHT switch (s1).
 * Fill in the TODOs in dt7_remote.cpp. See training/week2/README.md and dbus_protocol_en.md.
 *
 * Named Dt7Remote (not Remote) so it can't clash with taproot's tap::communication::serial::Remote.
 */
class Dt7Remote {
   public:
    // We reuse taproot's switch enum because the command mapper expects it. Its values
    // (UP = 1, DOWN = 2, MID = 3) are the same numbers the receiver sends.
    using SwitchState = tap::communication::serial::Remote::SwitchState;

    explicit Dt7Remote(src::Drivers* drivers) : drivers(drivers) {}
    DISALLOW_COPY_AND_ASSIGN(Dt7Remote)

    /** Starts the UART the receiver is plugged into. Called once from main.cpp. */
    void initialize();

    /** Reads every byte waiting in the UART and parses complete frames. Called every main loop. */
    void read();

    bool isConnected() const { return connected; }

    /** Right stick, -1 (full left) to +1 (full right). */
    float getRightHorizontal() const { return rightHorizontal / static_cast<float>(CHANNEL_MAX_OFFSET); }

    /** Right stick, -1 (full down) to +1 (full up). */
    float getRightVertical() const { return rightVertical / static_cast<float>(CHANNEL_MAX_OFFSET); }

    SwitchState getRightSwitch() const { return rightSwitch; }

   private:
    // Protocol constants. Check every one of these against the protocol document.
    static constexpr size_t FRAME_LENGTH = 18;            // bytes per DBUS frame
    static constexpr uint16_t CHANNEL_CENTER = 1024;      // raw value with the stick centered
    static constexpr uint16_t CHANNEL_MAX_OFFSET = 660;   // raw values go from 1024 - 660 to 1024 + 660
    static constexpr uint32_t FRAME_GAP_MS = 3;           // silence longer than this = next byte starts a new frame
    static constexpr uint32_t DISCONNECT_TIMEOUT_MS = 100;  // no good frame for this long = disconnected

    /**
     * Decodes rxBuffer (one complete frame) into the fields below.
     * @return true if the frame was valid, false if it was rejected.
     */
    bool parseFrame();

    /** Back to "nothing pressed": sticks centered, switch UNKNOWN. */
    void resetValues();

    src::Drivers* drivers;

    uint8_t rxBuffer[FRAME_LENGTH]{};
    size_t bytesReceived = 0;       // how many bytes of the current frame we have so far
    uint32_t lastByteTimeMs = 0;    // when the last byte arrived (for frame sync)
    uint32_t lastGoodFrameMs = 0;   // when the last VALID frame was parsed (for disconnect)
    bool connected = false;

    // Decoded values, as offsets from center: -660..660.
    int16_t rightHorizontal = 0;
    int16_t rightVertical = 0;
    SwitchState rightSwitch = SwitchState::UNKNOWN;
};

}  // namespace src::Training

#endif  // TARGET_TRAINING
