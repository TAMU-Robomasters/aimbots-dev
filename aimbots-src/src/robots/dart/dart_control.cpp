#ifdef TARGET_DART

#include "utils/tools/common_types.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"

//
#include "informants/kinematics/robot_frames.hpp"
#include "utils/ballistics/ballistics_solver.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
//
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//
#include "subsystems/chassis/control/chassis.hpp"
//
#include "subsystems/gimbal/control/gimbal.hpp"
//
#include "subsystems/hopper/basic_commands/close_hopper_command.hpp"
#include "subsystems/hopper/basic_commands/open_hopper_command.hpp"
#include "subsystems/hopper/complex_commands/toggle_hopper_command.hpp"
#include "subsystems/hopper/control/hopper.hpp"
//
#include "subsystems/slide/basic_commands/slide_control_command.hpp"
#include "subsystems/slide/basic_commands/slide_go_to_command.hpp"
#include "subsystems/slide/basic_commands/slide_hold_command.hpp"
#include "subsystems/slide/control/slide.hpp"

using namespace src::Chassis;
using namespace src::Gimbal;
using namespace src::Slide;
using namespace src::Hopper;

// For reference, all possible keyboard inputs:
// W,S,A,D,SHIFT,CTRL,Q,E,R,F,G,Z,X,C,V,B
/*  Dart Control Scheme:

    Chassis -----------------------------------------------------------

    Gimbal ------------------------------------------------------------


*/

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

namespace DartControl {

// Define subsystems here ------------------------------------------------
HopperSubsystem leftServo(drivers());
HopperSubsystem rightServo(drivers());
SlideSubsystem slide(drivers());

// Robot Specific Controllers ------------------------------------------------

// Ballistics Solver -------------------------------------------------------

// Define behavior configs here --------------------------------------------

// Define commands here ---------------------------------------------------

// Define command mappings here -------------------------------------------
// HoldCommandMapping leftSwitchMid(
//     drivers(),  // gimbalFieldRelativeControlCommand
//     {&gimbalToggleAimCommand},
//     RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&leftServo);
    drivers->commandScheduler.registerSubsystem(&rightServo);
    drivers->commandScheduler.registerSubsystem(&slide);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    leftServo.initialize();
    rightServo.initialize();
    slide.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    // feeder.setDefaultCommand(&stopFeederCommand);
}

// Set commands scheduled on startup
void startupCommands(src::Drivers *drivers) {
    // no startup commands should be set
    // yet...
    // TODO: Possibly add some sort of hardware test command
    //       that will move all the parts so we
    //       can make sure they're fully operational.
    // drivers->refSerial.attachRobotToRobotMessageHandler(SENTRY_RESPONSE_MESSAGE_ID, &responseHandler);
    // drivers->commandScheduler.addCommand(&clientDisplayCommand);
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    // drivers->commandMapper.addMap(&leftSwitchUp);
}

}  // namespace DartControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    DartControl::initializeSubsystems();
    DartControl::registerSubsystems(drivers);
    DartControl::setDefaultCommands(drivers);
    DartControl::startupCommands(drivers);
    DartControl::registerIOMappings(drivers);
}
}  // namespace src::Control

// temp

#endif  // TARGET_AERIAL
