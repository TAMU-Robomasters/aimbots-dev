#include "can_sound_command.hpp"

#include "subsystems/chassis/control/chassis.hpp"

namespace src::Chassis {

volatile uint32_t canSoundCommandConstructorCount = 0;
volatile uint32_t canSoundCommandInitializeCount = 0;
volatile uint32_t canSoundCommandExecuteCount = 0;
volatile uint32_t canSoundCommandEndCount = 0;
volatile uint32_t canSoundCommandTrySendCount = 0;
volatile uint8_t canSoundCommandLastSoundId = 0;
volatile bool canSoundCommandLastSendSucceeded = false;
volatile bool canSoundCommandHadChassisRequirement = false;

CanSoundCommand::CanSoundCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    SoundId soundId,
    uint8_t volume,
    uint8_t eq,
    bool forceReplay,
    bool finishImmediately)
    : CanSoundCommand(
          drivers,
          chassis,
          static_cast<uint8_t>(soundId),
          volume,
          eq,
          forceReplay,
          finishImmediately)
{
}

CanSoundCommand::CanSoundCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    uint8_t soundId,
    uint8_t volume,
    uint8_t eq,
    bool forceReplay,
    bool finishImmediately)
    : drivers(drivers),
      chassis(chassis),
      soundId(soundId),
      volume(volume),
      eq(eq),
      forceReplay(forceReplay),
      finishImmediately(finishImmediately)
{
    canSoundCommandConstructorCount++;

    if (chassis != nullptr) {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
        canSoundCommandHadChassisRequirement = true;
    } else {
        canSoundCommandHadChassisRequirement = false;
    }
}

void CanSoundCommand::initialize()
{
    canSoundCommandInitializeCount++;

    attemptedThisSchedule = false;
    lastSendSucceeded = false;
    trySendOnce();
}

void CanSoundCommand::execute()
{
    canSoundCommandExecuteCount++;
}

void CanSoundCommand::end(bool interrupted)
{
    static_cast<void>(interrupted);
    canSoundCommandEndCount++;

    attemptedThisSchedule = false;
}

bool CanSoundCommand::isReady()
{
    return true;
}

bool CanSoundCommand::isFinished() const
{
    return finishImmediately && attemptedThisSchedule;
}

void CanSoundCommand::setSoundId(SoundId soundId)
{
    this->soundId = static_cast<uint8_t>(soundId);
}

void CanSoundCommand::setSoundId(uint8_t soundId)
{
    this->soundId = soundId;
}

void CanSoundCommand::setStopCommand()
{
    soundId = SOUND_STOP;
}

void CanSoundCommand::setVolume(uint8_t volume)
{
    this->volume = volume;
}

void CanSoundCommand::setEq(uint8_t eq)
{
    this->eq = eq;
}

void CanSoundCommand::setForceReplay(bool forceReplay)
{
    this->forceReplay = forceReplay;
}

void CanSoundCommand::setFinishImmediately(bool finishImmediately)
{
    this->finishImmediately = finishImmediately;
}

void CanSoundCommand::trySendOnce()
{
    if (attemptedThisSchedule) {
        return;
    }

    attemptedThisSchedule = true;
    canSoundCommandTrySendCount++;
    canSoundCommandLastSoundId = soundId;

    if (drivers == nullptr) {
        lastSendSucceeded = false;
        canSoundCommandLastSendSucceeded = false;
        return;
    }

    if (soundId == SOUND_STOP) {
        lastSendSucceeded = drivers->canSoundSystem.stop(forceReplay);
    } else {
        lastSendSucceeded = drivers->canSoundSystem.play(soundId, volume, eq, forceReplay);
    }

    canSoundCommandLastSendSucceeded = lastSendSucceeded;
}

}  // namespace src::Chassis
