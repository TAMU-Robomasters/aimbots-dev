#ifdef TARGET_ENGINEER

#include "utils/common_types.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"
//
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_manual_drive_command.hpp"
//
#include "subsystems/gimbal/gimbal.hpp"
//
#include "subsystems/slide/slide.hpp"
#include "subsystems/slide/slide_control_command.hpp"
#include "subsystems/slide/slide_go_to_command.hpp"
#include "subsystems/slide/slide_hold_command.hpp"
//
#include "subsystems/wrist/wrist.hpp"
#include "subsystems/wrist/wrist_control_command.hpp"
#include "subsystems/wrist/wrist_move_command.hpp"
//
#include "subsystems/grabber/suction_command.hpp"

using namespace src::Chassis;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;
using namespace src::Slide;
using namespace src::Wrist;
using namespace src::Grabber;
using namespace src::Gimbal;

namespace EngineerControl {

// Define subsystems here -------`-----------------------------------------
ChassisSubsystem chassis(drivers());
GimbalSubsystem gimbal(drivers());
SlideSubsystem slide(drivers());
WristSubsystem wrist(drivers());
GrabberSubsystem grabber(drivers());

// Define commands here ---------------------------------------------------
ChassisManualDriveCommand chassisManualDriveCommand(drivers(), &chassis);
//
SlideGoToCommand goToTestLocation(drivers(), &slide, 0, 0.5f);
SlideGoToCommand goHome(drivers(), &slide, 0, 0);
SlideControlCommand slideControlCommand(drivers(), &slide);
SlideHoldCommand slideHoldCommand(drivers(), &slide);
//
WristMoveCommand wristHomeCommand(drivers(), &wrist, 0.0f, 0.0f, 0.0f);
WristControlCommand wristControlCommand(drivers(), &wrist);

Suction_Command suctionCommand(drivers(), &grabber);
// Define command mappings here -------------------------------------------
// HoldCommandMapping rightSwitchDown(
//     drivers(),
//     {&chassisManualDriveCommand},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

// HoldCommandMapping rightSwitchMid(
//     drivers(),
//     {&slideControlCommand},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

HoldCommandMapping leftSwitchUp(
    drivers(),
    {&slideControlCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

HoldCommandMapping leftSwitchMid(
    drivers(),
    {&chassisManualDriveCommand, &slideHoldCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

HoldCommandMapping rightSwitchDown(
    drivers(),
    {&goHome},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

HoldCommandMapping rightSwitchMid(
    drivers(),
    {&goToTestLocation},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&slide);
    drivers->commandScheduler.registerSubsystem(&wrist);
    drivers->commandScheduler.registerSubsystem(&grabber);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
    slide.initialize();
    wrist.initialize();
    grabber.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    // no default commands should be set
    // wrist.setDefaultCommand(&wristHomeCommand);
}

// Set commands scheduled on startup
void startupCommands(src::Drivers *) {
    // no startup commands should be set
    // yet...
    // TODO: Possibly add some sort of hardware test command
    //       that will move all the parts so we
    //       can make sure they're fully operational.
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    // drivers->commandMapper.addMap(&leftSwitchUp);
    // drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchDown);

    // drivers->commandMapper.addMap(&leftSwitchDown);
}

}  // namespace EngineerControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    EngineerControl::initializeSubsystems();
    EngineerControl::registerSubsystems(drivers);
    EngineerControl::setDefaultCommands(drivers);
    EngineerControl::startupCommands(drivers);
    EngineerControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif  // TARGET_ENGINEER