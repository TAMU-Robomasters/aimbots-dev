#include "subsystems/chassis/basic_commands/chassis_ignore_gimbal_command.hpp"
#include "utils/tools/robot_specific_defines.hpp"

#if defined(ALL_DARTS)

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

//

//

//

//


src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;
using namespace tap::communication::serial;

namespace DartControl {

// This is technically a command flag, but it needs to be defined before the refHelper


// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {

}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {

}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {

}

// Set commands scheduled on startup
void startupCommands(src::Drivers *drivers) {
    UNUSED(drivers);

}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {

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

#endif  // TARGET_HERO
