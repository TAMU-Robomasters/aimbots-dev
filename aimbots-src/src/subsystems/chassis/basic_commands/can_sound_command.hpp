#pragma once

#include <cstdint>

#include "tap/control/command.hpp"

#include "drivers.hpp"
#include "communicators/can_sound_system/can_sound_system.hpp"

namespace src::Chassis {

class ChassisSubsystem;

class CanSoundCommand : public TapCommand {
public:
    using CanSoundSystem = src::communicators::can_sound_system::CanSoundSystem;
    using SoundId = CanSoundSystem::SoundId;
    using EqSetting = CanSoundSystem::EqSetting;

    static constexpr uint8_t SOUND_STOP = CanSoundSystem::SOUND_STOP;
    static constexpr uint8_t KEEP_VOLUME = CanSoundSystem::KEEP_VOLUME;
    static constexpr uint8_t KEEP_EQ = CanSoundSystem::KEEP_EQ;

    CanSoundCommand(
        src::Drivers* drivers,
        ChassisSubsystem* chassis,
        SoundId soundId,
        uint8_t volume = KEEP_VOLUME,
        uint8_t eq = KEEP_EQ,
        bool forceReplay = false,
        bool finishImmediately = false);

    CanSoundCommand(
        src::Drivers* drivers,
        ChassisSubsystem* chassis,
        uint8_t soundId,
        uint8_t volume = KEEP_VOLUME,
        uint8_t eq = KEEP_EQ,
        bool forceReplay = false,
        bool finishImmediately = false);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "CAN Sound Command TEMP Chassis Requirement"; }

    void setSoundId(SoundId soundId);
    void setSoundId(uint8_t soundId);
    void setStopCommand();

    void setVolume(uint8_t volume);
    void setEq(uint8_t eq);
    void setForceReplay(bool forceReplay);
    void setFinishImmediately(bool finishImmediately);

    uint8_t getSoundId() const { return soundId; }
    uint8_t getVolume() const { return volume; }
    uint8_t getEq() const { return eq; }
    bool getForceReplay() const { return forceReplay; }
    bool didLastSendSucceed() const { return lastSendSucceeded; }

private:
    void trySendOnce();

    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    uint8_t soundId;
    uint8_t volume;
    uint8_t eq;
    bool forceReplay;
    bool finishImmediately;

    bool attemptedThisSchedule = false;
    bool lastSendSucceeded = false;
};

extern volatile uint32_t canSoundCommandConstructorCount;
extern volatile uint32_t canSoundCommandInitializeCount;
extern volatile uint32_t canSoundCommandExecuteCount;
extern volatile uint32_t canSoundCommandEndCount;
extern volatile uint32_t canSoundCommandTrySendCount;
extern volatile uint8_t canSoundCommandLastSoundId;
extern volatile bool canSoundCommandLastSendSucceeded;
extern volatile bool canSoundCommandHadChassisRequirement;

}  // namespace src::Chassis
