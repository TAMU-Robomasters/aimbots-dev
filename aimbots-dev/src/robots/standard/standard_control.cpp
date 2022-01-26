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

using namespace src::Chassis;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

namespace StandardControl {
// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());

// Define commands here ---------------------------------------------------
ChassisDriveCommand chassisDriveCommand(drivers(), &chassis);

// Define command mappings here -------------------------------------------

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    chassis.setDefaultCommand(&chassisDriveCommand);
}

// Set commands scheduled on startup
void startupCommands(src::Drivers *drivers) {
    drivers->commandScheduler.addCommand(&chassisDriveCommand);
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *) {
}

}  // namespace StandardControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    StandardControl::initializeSubsystems();
    StandardControl::registerSubsystems(drivers);
    StandardControl::setDefaultCommands(drivers);
    StandardControl::startupCommands(drivers);
    StandardControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif