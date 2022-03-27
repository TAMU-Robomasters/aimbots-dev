#ifdef TARGET_STANDARD

#include "drivers.hpp"
#include "drivers_singleton.hpp"
#include "utils/common_types.hpp"
//
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_drive_command.hpp"
//
#include "subsystems/feeder/feeder.hpp"
#include "subsystems/feeder/run_feeder_command.hpp"
#include "subsystems/feeder/stop_feeder_command.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/shooter_command.hpp"
#include "subsystems/shooter/shooter_default_command.hpp"

using namespace src::Chassis;
using namespace src::Feeder;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;

namespace StandardControl {

// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
FeederSubsystem feeder(drivers());
src::Shooter::ShooterSubsystem shooter(drivers());

// Define commands here ---------------------------------------------------
ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);
RunFeederCommand runFeederCommand(drivers(), &feeder);
StopFeederCommand stopFeederCommand(drivers(), &feeder);
src::Shooter::ShooterCommand shooterCommand(drivers(), &shooter);
src::Shooter::ShooterDefaultCommand shooterDefaultCommand(drivers(), &shooter);

// Define command mappings here -------------------------------------------
HoldCommandMapping leftSwitchUp(  //you MUST map commands to run them at all (we think)
    drivers(),
    {&chassisDriveCommand, &shooterCommand},
    RemoteMapState(tap::communication::serial::Remote::Switch::LEFT_SWITCH, tap::communication::serial::Remote::SwitchState::UP));

HoldCommandMapping rightSwitchUp(
    drivers(),
    {&runFeederCommand},
    RemoteMapState(tap::communication::serial::Remote::Switch::RIGHT_SWITCH, tap::communication::serial::Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&shooter);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
    feeder.initialize();
    shooter.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    feeder.setDefaultCommand(&stopFeederCommand);
    // no default commands should be set
    //shooter.setDefaultCommand(&shooterDefaultCommand);
}

// Set commands scheduled on startup
void startupCommands(src::Drivers *) {
    // drivers->commandScheduler.addCommand(&runFeederCommand);
    //  no startup commands should be set
    //  yet...
    //  TODO: Possibly add some sort of hardware test command
    //        that will move all the standard's parts so we
    //        can make sure they're fully operational.
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchUp);
}

}  // namespace StandardControl

namespace src::Control {
    // Initialize subsystems ---------------------------------------------------
    void initializeSubsystemCommands(src::Drivers * drivers) {
        StandardControl::initializeSubsystems();
        StandardControl::registerSubsystems(drivers);
        StandardControl::setDefaultCommands(drivers);
        StandardControl::startupCommands(drivers);
        StandardControl::registerIOMappings(drivers);
    }
}  // namespace src::Control

#endif  //TARGET_STANDARD