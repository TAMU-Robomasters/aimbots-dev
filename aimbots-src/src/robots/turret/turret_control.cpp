#ifdef TARGET_TURRET

#include "utils/tools/common_types.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"

namespace TurretControl {

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *) {}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {}

// Set commands scheduled on startup
void startupCommands(src::Drivers *) {
    // no startup commands should be set
    // yet...
    // TODO: Possibly add some sort of hardware test command
    //       that will move all the parts so we
    //       can make sure they're fully operational.
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *) {}

}  // namespace TurretControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    TurretControl::initializeSubsystems();
    TurretControl::registerSubsystems(drivers);
    TurretControl::setDefaultCommands(drivers);
    TurretControl::startupCommands(drivers);
    TurretControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif