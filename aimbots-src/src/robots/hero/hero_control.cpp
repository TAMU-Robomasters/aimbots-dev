#ifdef TARGET_HERO

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
#include "subsystems/chassis/chassis_manual_drive_command.hpp"
//
#include "subsystems/shooter/run_shooter_command.hpp"
#include "subsystems/shooter/shooter.hpp"

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

namespace HeroControl {

// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
src::Shooter::ShooterSubsystem shooter(drivers());

// Define commands here ---------------------------------------------------
ChassisManualDriveCommand chassisManualDriveCommand(drivers(), &chassis);
src::Shooter::RunRunShooterCommand runShooterCommand(drivers(), &shooter);

// Define command mappings here -------------------------------------------
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&chassisManualDriveCommand, &runShooterCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&shooter);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
    shooter.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    // no default commands should be set
}

// Set commands scheduled on startup
void startupCommands(src::Drivers *drivers) {
    // no startup commands should be set
    // yet...
    // TODO: Possibly add some sort of hardware test command
    //       that will move all the parts so we
    //       can make sure they're fully operational.
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(&leftSwitchUp);
}

}  // namespace HeroControl

namespace src::Control {
    // Initialize subsystems ---------------------------------------------------
    void initializeSubsystemCommands(src::Drivers * drivers) {
        HeroControl::initializeSubsystems();
        HeroControl::registerSubsystems(drivers);
        HeroControl::setDefaultCommands(drivers);
        HeroControl::startupCommands(drivers);
        HeroControl::registerIOMappings(drivers);
    }
}  // namespace src::Control

#endif  // TARGET_HERO