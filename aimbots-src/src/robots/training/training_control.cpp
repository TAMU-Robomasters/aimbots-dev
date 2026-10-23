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
//
#include "robots/training/commands/buzzer_command.hpp"
#include "robots/training/training_config.hpp"
//
#include "robots/training/commands/velocity_pid_tuning_command.hpp"
#include "robots/training/motor/training_motor_subsystem.hpp"
#include "robots/training/motor/velocity_motor_pid.hpp"
#include "robots/training/commands/velocity_control_command.hpp"
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
TrainingMotorSubsystem motorSubsystem(drivers());
static constexpr SmoothPIDConfig VELOCITY_PID_CONFIG = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 1000000.0f,
    .maxOutput = 25000.0f,
};

VelocityMotorPid velocityPid(
    drivers(),
    &motorSubsystem,
    VELOCITY_PID_CONFIG);

static constexpr VelocityTuningConfig TUNING_CONFIG = {
    .amplitudeRpm = 60.0f,
    .frequencyHz = 0.5f
};
// Define commands here ---------------------------------------------------
// TODO(week1): create your BlinkLedCommand here.
BlinkLedCommand blinkLedCommand(drivers(), &board);
BuzzerCommand buzzerCommand(drivers(), &board);

VelocityPidTuningCommand velocityPidTuningCommand(
    drivers(),
    &motorSubsystem,
    &velocityPid,
    TUNING_CONFIG);

VelocityControlCommand velocityControlCommand(
    drivers(),
    &motorSubsystem,
    &velocityPid);
// Define command mappings here -------------------------------------------
HoldCommandMapping rightSwitchUp(
    drivers(),
    {&velocityPidTuningCommand},
    RemoteMapState(
        Remote::Switch::RIGHT_SWITCH,
        Remote::SwitchState::UP));
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&velocityControlCommand},
    RemoteMapState(
        Remote::Switch::RIGHT_SWITCH,
        Remote::SwitchState::DOWN));
// TODO(week1): map your command to the RIGHT switch in the UP position.
// A mapping ties a remote state to a list of commands. The three kinds you'll see most:
//
//   HoldCommandMapping    -- command runs while the state is held, ends when it isn't
//   ToggleCommandMapping  -- each time the state starts matching it flips: start, stop, start...
//   PressCommandMapping   -- command starts on match and runs until it finishes itself
//
// Example (from testbench_control.cpp):
//   HoldCommandMapping leftSwitchUp(
//       drivers(),
//       {&gimbalChaseCommand},
//       RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&board);
    drivers->commandScheduler.registerSubsystem(&motorSubsystem);
}
// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    board.initialize();
    motorSubsystem.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {}

// Set commands scheduled on startup
void startupCommands(src::Drivers *) {}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers * drivers) {
    // TODO(week1): add your mapping, e.g. drivers->commandMapper.addMap(&yourMapping);
    drivers->commandMapper.addMap(&rightSwitchUp);
    //week 2
    drivers ->commandMapper.addMap(&rightSwitchDown);
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
