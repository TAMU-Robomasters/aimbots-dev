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


using namespace src::Chassis;
using namespace src::Feeder;
using namespace src::Gimbal;
using namespace src::Shooter;
using namespace src::Hopper;
// using namespace src::Communication;
// using namespace src::RobotStates;
using namespace src::Utils::ClientDisplay;


// For reference, all possible keyboard inputs:
// W,S,A,D,SHIFT,CTRL,Q,E,R,F,G,Z,X,C,V,B
/*  DART Control Scheme:

    Chassis -----------------------------------------------------------
    Toggle Chassis Drive Mode (Field Relative <-> Toyko Drift): F
    Quick 90-deg Turn Gimbal Yaw (Left): Q
    Quick 90-deg Turn Gimbal Yaw (Right): E

    Manually Choose Tokyo Direction (Left): F+Q
    Manually Choose Tokyo Direction (Right): F+E

    Decrease Chassis Ground Speed (60%): Shift
    Decrease Chassis Ground Speed (25%): Ctrl

    Gimbal ------------------------------------------------------------
    Aim Using CV: Right Mouse Button

    Shooter -----------------------------------------------------------

    Feeder ------------------------------------------------------------
    Full Auto Shooting: Left Mouse Button

    Hopper ------------------------------------------------------------
    Toggle Hopper Position: C

    Barrel Manager ----------------------------------------------------
    Manually Switch Barrel: R
    Recalibrate: Hold G for 1 second

    UI ----------------------------------------------------------------


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

// This is technically a command flag, but it needs to be defined before the barrel manager subsystem


// Define subsystems here ------------------------------------------------




// Define behavior configs here --------------------------------------------




// GimbalPatrolConfig patrolConfig = {
//     .pitchPatrolAmplitude = modm::toRadian(11.0f),
//     .pitchPatrolFrequency = 1.5f * M_PI,
//     .pitchPatrolOffset = -modm::toRadian(11.0f),
// };

// Define commands here ---------------------------------------------------

//GimbalPatrolCommand gimbalPatrolCommand(drivers(), &gimbal, &gimbalFieldRelativeController, patrolConfig);
//GimbalControlCommand gimbalControlCommand(drivers(), &gimbal, &gimbalChassisRelativeController);


// FullAutoFeederCommand runFeederCommand(drivers(), &feeder, &refHelper, FEEDER_DEFAULT_RPM, 3000.0f, UNJAM_TIMER_MS);
// FullAutoFeederCommand runFeederCommandFromMouse(drivers(), &feeder, &refHelper, FEEDER_DEFAULT_RPM, 3000.0f, UNJAM_TIMER_MS);
// Raise the acceptable threshold on the feeder to let it trust the barrel manager will prevent overheat

// StopFeederCommand stopFeederCommand(drivers(), &feeder);



// Runs shooter with feeder and closes hopper
// HoldRepeatCommandMapping rightSwitchUp(
//     drivers(),
//     {&runFeederCommand, &runShooterWithFeederCommand},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
//     true);

// HoldCommandMapping leftClickMouse(
//     drivers(),
//     {&runFeederCommandFromMouse},
//     RemoteMapState(RemoteMapState::MouseButton::LEFT));

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

}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {

}

}  // namespace DartControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {

}  // namespace src::Control
}
// temp

#endif  // TARGET_DART

