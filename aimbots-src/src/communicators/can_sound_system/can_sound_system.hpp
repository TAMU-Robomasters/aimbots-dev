#ifndef CAN_SOUND_SYSTEM_HPP_
#define CAN_SOUND_SYSTEM_HPP_

#include <cstdint>

#include "tap/communication/can/can_bus.hpp"

namespace tap
{
class Drivers;
}

namespace src::communicators::can_sound_system
{
/**
 * CAN command sender for the ESP32 audio player and amp
 *
 * Payload:
 *   byte 0: sound ID
 *   byte 1: sequence number, incremented every command
 *   byte 2: volume, 0-30, or KEEP_VOLUME/0xFF to keep current volume
 *   byte 3: EQ, 0-5, or KEEP_EQ/0xFF to keep current EQ mode
 *   byte 4: flags, bit0 = force replay
 *
 */
class CanSoundSystem
{
public:
    static constexpr tap::can::CanBus BUS = tap::can::CanBus::CAN_BUS2;

    static constexpr uint16_t CAN_ID_AUDIO_CMD = 0x352;
    static constexpr uint16_t CAN_ID_AUDIO_STATUS_RESERVED = 0x353;

    static constexpr uint8_t PAYLOAD_LEN = 5;

    static constexpr uint8_t KEEP_VOLUME = 0xFF;
    static constexpr uint8_t KEEP_EQ = 0xFF;

    static constexpr uint8_t FLAG_FORCE_REPLAY = 0x01;

    enum SoundId : uint8_t
    {
        SOUND_STOP = 0,

        SOUND_PLAYER_START = 1,
        SOUND_STARTUP_SOUND = 2,
        SOUND_IMU_HEATING = 3,
        SOUND_SHOOT = 4,
        SOUND_SPIN = 5,
        SOUND_WIN = 6,
        SOUND_LOSE = 7,
        SOUND_CV_TARGET = 8,
        SOUND_TAUNT1 = 9,
        SOUND_TAUNT2 = 10,
        SOUND_TAUNT3 = 11,
        SOUND_SONG1 = 12,
        SOUND_SONG2 = 13,
        SOUND_SONG3 = 14,
        SOUND_SFX1 = 15,
        SOUND_SFX2 = 16,
        SOUND_SFX3 = 17,
    };

    enum EqSetting : uint8_t
    {
        EQ_NORMAL = 0,
        EQ_POP = 1,
        EQ_ROCK = 2,
        EQ_JAZZ = 3,
        EQ_CLASSIC = 4,
        EQ_BASS = 5,
    };

    explicit CanSoundSystem(tap::Drivers* drivers);

    void initialize();

    void read() {}

    bool play(
        SoundId soundId,
        uint8_t volume = KEEP_VOLUME,
        uint8_t eq = KEEP_EQ,
        bool forceReplay = false);

    bool play(
        uint8_t soundId,
        uint8_t volume = KEEP_VOLUME,
        uint8_t eq = KEEP_EQ,
        bool forceReplay = false);

    bool stop(bool forceReplay = false);

    uint32_t getTxCount() const { return txCount; }
    uint32_t getTxFailCount() const { return txFailCount; }
    uint32_t getInvalidCommandCount() const { return invalidCommandCount; }
    uint32_t getLastCommandTimeMs() const { return lastCommandTimeMs; }
    uint8_t getLastSoundId() const { return lastSoundId; }
    uint8_t getLastSequence() const { return lastSequence; }

private:
    bool sendCommand(uint8_t soundId, uint8_t volume, uint8_t eq, uint8_t flags);
    uint8_t nextSequence();
    static bool isValidSoundId(uint8_t soundId);
    static bool isValidVolumeOrKeep(uint8_t volume);
    static bool isValidEqOrKeep(uint8_t eq);

private:
    tap::Drivers* drivers;

    uint8_t sequenceCounter = 0;
    uint8_t lastSequence = 0;
    uint8_t lastSoundId = SOUND_STOP;

    uint32_t txCount = 0;
    uint32_t txFailCount = 0;
    uint32_t invalidCommandCount = 0;
    uint32_t lastCommandTimeMs = 0;
};

}  // namespace src::communicators::can_audio_player

#endif  // CAN_SOUND_SYSTEM_HPP_
