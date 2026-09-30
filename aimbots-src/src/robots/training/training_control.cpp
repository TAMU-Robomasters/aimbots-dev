#include "utils/tools/robot_specific_defines.hpp"

#ifdef TARGET_TRAINING

#include "drivers.hpp"
#include "drivers_singleton.hpp"
//
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//
#include "robots/training/commands/blink_led_command.hpp"
#include "robots/training/training_board_subsystem.hpp"

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;
using namespace tap::communication::serial;
using namespace src::Training;

namespace TrainingControl {

// Define subsystems here ------------------------------------------------
TrainingBoardSubsystem board(drivers());

// Define commands here ---------------------------------------------------
// TODO(week1): create your BlinkLedCommand here.

// Define command mappings here -------------------------------------------
// TODO(week1): map your command to the RIGHT switch in the UP position.
// A mapping ties a remote state to a list of commands. The three kinds you'll see most:
//
//   HoldCommandMapping    -- command runs while the state is held, ends when it isn't
//   ToggleCommandMapping  -- first match starts the command, next match stops it
//   PressCommandMapping   -- command starts on match and runs until it finishes itself
//
// Example (from testbench_control.cpp):
//   HoldCommandMapping leftSwitchUp(
//       drivers(),
//       {&gimbalChaseCommand},
//       RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) { drivers->commandScheduler.registerSubsystem(&board); }

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() { board.initialize(); }

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {}

// Set commands scheduled on startup
void startupCommands(src::Drivers *) {}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *) {
    // TODO(week1): add your mapping, e.g. drivers->commandMapper.addMap(&yourMapping);
}

}  // namespace TrainingControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    TrainingControl::initializeSubsystems();
    TrainingControl::registerSubsystems(drivers);
    TrainingControl::setDefaultCommands(drivers);
    TrainingControl::startupCommands(drivers);
    TrainingControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif  // TARGET_TRAINING
