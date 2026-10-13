#include "can_sound_system.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "modm/architecture/interface/can_message.hpp"

namespace src::communicators::can_sound_system
{
using tap::arch::clock::getTimeMilliseconds;

CanSoundSystem::CanSoundSystem(tap::Drivers* drivers)
    : drivers(drivers)
{
}

void CanSoundSystem::initialize()
{
    sequenceCounter = 0;
    lastSequence = 0;
    lastSoundId = SOUND_STOP;

    txCount = 0;
    txFailCount = 0;
    invalidCommandCount = 0;
    lastCommandTimeMs = 0;
}

bool CanSoundSystem::play(
    SoundId soundId,
    uint8_t volume,
    uint8_t eq,
    bool forceReplay)
{
    return play(static_cast<uint8_t>(soundId), volume, eq, forceReplay);
}

bool CanSoundSystem::play(
    uint8_t soundId,
    uint8_t volume,
    uint8_t eq,
    bool forceReplay)
{
    if (!isValidSoundId(soundId) ||
        !isValidVolumeOrKeep(volume) ||
        !isValidEqOrKeep(eq))
    {
        invalidCommandCount++;
        return false;
    }

    const uint8_t flags = forceReplay ? FLAG_FORCE_REPLAY : 0;
    return sendCommand(soundId, volume, eq, flags);
}

bool CanSoundSystem::stop(bool forceReplay)
{
    const uint8_t flags = forceReplay ? FLAG_FORCE_REPLAY : 0;
    return sendCommand(SOUND_STOP, KEEP_VOLUME, KEEP_EQ, flags);
}

bool CanSoundSystem::sendCommand(uint8_t soundId, uint8_t volume, uint8_t eq, uint8_t flags)
{
    modm::can::Message msg(CAN_ID_AUDIO_CMD, PAYLOAD_LEN);
    msg.setExtended(false);
    msg.setRemoteTransmitRequest(false);

    const uint8_t seq = nextSequence();

    msg.data[0] = soundId;
    msg.data[1] = seq;
    msg.data[2] = volume;
    msg.data[3] = eq;
    msg.data[4] = flags;

    if (drivers == nullptr || !drivers->can.isReadyToSend(BUS))
    {
        txFailCount++;
        return false;
    }

    (void)drivers->can.sendMessage(BUS, msg);

    txCount++;
    lastSequence = seq;
    lastSoundId = soundId;
    lastCommandTimeMs = getTimeMilliseconds();

    return true;
}

uint8_t CanSoundSystem::nextSequence()
{
    const uint8_t seq = sequenceCounter;
    sequenceCounter++;
    return seq;
}

bool CanSoundSystem::isValidSoundId(uint8_t soundId)
{
    return soundId <= SOUND_SFX3;
}

bool CanSoundSystem::isValidVolumeOrKeep(uint8_t volume)
{
    return volume == KEEP_VOLUME || volume <= 30;
}

bool CanSoundSystem::isValidEqOrKeep(uint8_t eq)
{
    return eq == KEEP_EQ || eq <= EQ_BASS;
}

}  // namespace src::communicators::can_audio_player
