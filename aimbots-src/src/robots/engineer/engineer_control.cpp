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

#include "subsystems/slide/slide.hpp"
#include "subsystems/slide/slide_go_to_command.hpp"
//
#include "subsystems/wrist/wrist.hpp"
#include "subsystems/wrist/wrist_move_command.hpp"
//
#include "subsystems/grabber/grabber.hpp"
#include "subsystems/grabber/suction_command.hpp"

using namespace src::Chassis;
using namespace src::Gimbal;
using namespace src::Wrist;
using namespace src::Grabber;

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

namespace EngineerControl {

// Define subsystems here -------`-----------------------------------------
ChassisSubsystem chassis(drivers());
GimbalSubsystem gimbal(drivers());
SlideSubsystem slide(drivers());
GrabberSubsystem grabber(drivers());
WristSubsystem wrist(drivers());

// Define commands here ---------------------------------------------------
ChassisManualDriveCommand chassisManualDriveCommand(drivers(), &chassis);
SlideGoToCommand goToTestLocation(drivers(), &slide, 0, 800);
SlideGoToCommand goHome(drivers(), &slide, 0, 0);

// Define commands here ---------------------------------------------------
WristMoveCommand wristHomeCommand(drivers(), &wrist, 0.0f, 0.0f, 0.0f);
WristMoveCommand wristMoveCommand(drivers(), &wrist, PI/2, .0f, 0.0f);

Suction_Command suctionCommand(drivers(), &grabber);
// Define command mappings here -------------------------------------------
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&chassisManualDriveCommand, &wristHomeCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
    
HoldCommandMapping rightSwitchUp(
    drivers(),
    {&suctionCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

/* THESE MAPPINGS ARE TEMPORARY */

HoldCommandMapping leftSwitchMid(
    drivers(),
    {&goToTestLocation, &wristMoveCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

HoldCommandMapping leftSwitchDown(
    drivers(),
    {&goHome},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN));

/* END OF TEMPORARY MAPPINGS */
    
// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&wrist);
    drivers->commandScheduler.registerSubsystem(&slide);
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
    wrist.setDefaultCommand(&wristHomeCommand);
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
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&leftSwitchDown);
    drivers->commandMapper.addMap(&rightSwitchUp);
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
