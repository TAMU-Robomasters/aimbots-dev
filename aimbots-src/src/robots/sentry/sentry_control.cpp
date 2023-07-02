#ifdef TARGET_SENTRY

#include "informants/transformers/robot_frames.hpp"
#include "utils/ballistics_solver.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

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
#include "subsystems/chassis/chassis_toggle_drive_command.hpp"
#include "subsystems/chassis/chassis_tokyo_command.hpp"
//
#include "subsystems/feeder/burst_feeder_command.hpp"
#include "subsystems/feeder/feeder.hpp"
#include "subsystems/feeder/full_auto_feeder_command.hpp"

#include "subsystems/feeder/stop_feeder_command.hpp"
//
#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/controllers/gimbal_field_relative_controller.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "subsystems/gimbal/gimbal_chase_command.hpp"
#include "subsystems/gimbal/gimbal_control_command.hpp"
#include "subsystems/gimbal/gimbal_field_relative_control_command.hpp"
#include "subsystems/gimbal/gimbal_toggle_aiming_command.hpp"
//
#include "subsystems/shooter/brake_shooter_command.hpp"
#include "subsystems/shooter/run_shooter_command.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/stop_shooter_command.hpp"
#include "subsystems/shooter/stop_shooter_comprised_command.hpp"
//

using namespace src::Chassis;
using namespace src::Feeder;
using namespace src::Gimbal;
using namespace src::Shooter;

using namespace src::Control;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;

namespace SentryControl {

// This is technically a command flag, but it needs to be defined before the refHelper
BarrelID currentBarrel = BARREL_IDS[0];

src::Utils::RefereeHelperTurreted refHelper(drivers(), currentBarrel);

//src::Chassis::ChassisMatchStates chassisMatchState = src::Chassis::ChassisMatchStates::NONE;
//src::Control::FeederMatchStates feederMatchState = src::Control::FeederMatchStates::ANNOYED;

// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
FeederSubsystem feeder(drivers());
GimbalSubsystem gimbal(drivers());
ShooterSubsystem shooter(drivers(), &refHelper);

// Robot Specific Controllers ------------------------------------------------
GimbalChassisRelativeController gimbalController(&gimbal);
GimbalFieldRelativeController gimbalFieldRelativeController(drivers(), &gimbal);

// Ballistics Solver
src::Utils::Ballistics::BallisticsSolver ballisticsSolver(drivers()/*, &refHelper*/);

// Match Controllers ------------------------------------------------
//SentryMatchFiringControlCommand matchFiringControlCommand(drivers(), &feeder, &shooter, &refHelper, chassisMatchState);
//SentryMatchChassisControlCommand matchChassisControlCommand(drivers(), &chassis, chassisMatchState);
/*SentryMatchGimbalControlCommand matchGimbalControlCommand(
    drivers(),
    &gimbal,
    &gimbalController,
    &refHelper,
    currentBarrel,
    &ballisticsSolver,
    500.0f);*/

SnapSymmetryConfig defaultSnapConfig = {
    .numSnapPositions = CHASSIS_SNAP_POSITIONS,
    .snapAngle = modm::toRadian(0.0f),
};

TokyoConfig defaultTokyoConfig = {
    .translationalSpeedMultiplier = 0.6f,
    .translationThresholdToDecreaseRotationSpeed = 0.5f,
    .rotationalSpeedFractionOfMax = 0.75f,
    .rotationalSpeedMultiplierWhenTranslating = 0.7f,
    .rotationalSpeedIncrement = 50.0f,
};

SpinRandomizerConfig randomizerConfig = {
    .minSpinRateModifier = 0.75f,
    .maxSpinRateModifier = 1.0f,
    .minSpinRateModifierDuration = 500,
    .maxSpinRateModifierDuration = 3000,
};


// Define commands here ---------------------------------------------------
ChassisManualDriveCommand chassisManualDriveCommand(drivers(), &chassis);
ChassisToggleDriveCommand chassisToggleDriveCommand(drivers(), &chassis, &gimbal);
ChassisTokyoCommand chassisTokyoCommand(drivers(), 
    &chassis, 
    &gimbal, 
    defaultTokyoConfig,
    0,
    false,
    randomizerConfig);
//ChassisRailEvadeCommand chassisRailEvadeCommand(drivers(), &chassis, 25.0f);  // Likely to be changed to different evasion

GimbalControlCommand gimbalControlCommand(drivers(), &gimbal, &gimbalController);
/*GimbalPatrolCommand gimbalPatrolCommand(
    drivers(),
    &gimbal,
    &gimbalController,
    PITCH_PATROL_AMPLITUDE,
    PITCH_PATROL_FREQUENCY,
    PITCH_PATROL_OFFSET,
    PITCH_OFFSET_ANGLE);*/  // TODO: Add constants to the sentry file and place them here
GimbalFieldRelativeControlCommand gimbalFieldRelativeControlCommand(drivers(), &gimbal, &gimbalFieldRelativeController);
GimbalFieldRelativeControlCommand gimbalFieldRelativeControlCommand2(drivers(), &gimbal, &gimbalFieldRelativeController);

// pass chassisRelative controller to gimbalChaseCommand on sentry, pass fieldRelative for other robots
GimbalChaseCommand gimbalChaseCommand(
    drivers(),
    &gimbal,
    &gimbalFieldRelativeController,
    &refHelper,
    &ballisticsSolver,
    SHOOTER_SPEED_MATRIX[0][0]);
GimbalChaseCommand gimbalChaseCommand2(
    drivers(),
    &gimbal,
    &gimbalFieldRelativeController,
    &refHelper,
    &ballisticsSolver,
    SHOOTER_SPEED_MATRIX[0][0]);

FullAutoFeederCommand runFeederCommand(drivers(), &feeder, &refHelper, FEEDER_DEFAULT_RPM, UNJAM_TIMER_MS, -1500);
StopFeederCommand stopFeederCommand(drivers(), &feeder);

RunShooterCommand runShooterCommand(drivers(), &shooter, &refHelper);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter, &refHelper);
StopShooterComprisedCommand stopShooterComprisedCommand(drivers(), &shooter);

// Define command mappings here -------------------------------------------

// Enables both chassis and gimbal manual control
HoldCommandMapping leftSwitchMid(
    drivers(),
    {/*&chassisToggleDriveCommand,*/ &gimbalFieldRelativeControlCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

HoldCommandMapping leftSwitchUp(
    drivers(),
    {/*&chassisTokyoCommand,*/ &gimbalFieldRelativeControlCommand2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Runs shooter only
HoldCommandMapping rightSwitchMid(
    drivers(),
    {&runShooterCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder
HoldCommandMapping rightSwitchUp(
    drivers(),
    {&runFeederCommand, &runShooterWithFeederCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&gimbal);
    drivers->commandScheduler.registerSubsystem(&shooter);

    drivers->kinematicInformant.registerSubsystems(&gimbal, &chassis);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
    feeder.initialize();
    gimbal.initialize();
    shooter.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    shooter.setDefaultCommand(&stopShooterComprisedCommand);
    feeder.setDefaultCommand(&stopFeederCommand);
    // gimbal.setDefaultCommand(&gimbalControlCommand);
    // chassis.setDefaultCommand(&chassisRailBounceCommand);
    // gimbal.setDefaultCommand(&gimbalChaseCommand);
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
    // drivers->commandMapper.addMap(&leftSwitchMid);
    // drivers->commandMapper.addMap(&leftSwitchUp);

    drivers->commandMapper.addMap(&rightSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchUp);
}

}  // namespace SentryControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    SentryControl::initializeSubsystems();
    SentryControl::registerSubsystems(drivers);
    SentryControl::setDefaultCommands(drivers);
    SentryControl::startupCommands(drivers);
    SentryControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif  // TARGET_SENTRY