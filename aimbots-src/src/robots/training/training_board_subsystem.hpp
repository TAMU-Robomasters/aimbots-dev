#pragma once

#ifdef TARGET_TRAINING

#include "tap/control/subsystem.hpp"

namespace src::Training {

/**
 * @brief Stand-in subsystem for the training dev board. You do NOT need to edit this file.
 *
 * Taproot's CommandScheduler refuses to run a command that requires no subsystems
 * (see CommandScheduler::addCommand in taproot/src/tap/control/command_scheduler.cpp),
 * so every command needs at least one subsystem to "own". This empty subsystem is that owner.
 *
 * On a real robot a subsystem wraps hardware that only one command should control at a
 * time -- almost always motors (see ChassisSubsystem, GimbalSubsystem). Wrapping a single
 * LED like this would be overkill; we only do it so you can learn the framework first.
 */
class TrainingBoardSubsystem : public tap::control::Subsystem {
   public:
    TrainingBoardSubsystem(tap::Drivers* drivers) : tap::control::Subsystem(drivers) {}

    void initialize() override {}
    void refresh() override {}

    const char* getName() const override { return "Training Board"; }
};

}  // namespace src::Training

#endif  // TARGET_TRAINING
