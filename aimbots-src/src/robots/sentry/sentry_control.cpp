#include "utils/tools/robot_specific_defines.hpp"

#if defined(ALL_SENTRIES)

#include "informants/kinematics/robot_frames.hpp"
#include "utils/ballistics/ballistics_solver.hpp"
#include "utils/tools/common_types.hpp"

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
#include "informants/imu/calibrate_imu_command.hpp"
//
#include "subsystems/chassis/basic_commands/chassis_manual_drive_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_tokyo_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_toggle_drive_command.hpp"
#include "subsystems/chassis/control/chassis.hpp"
//
#include "subsystems/feeder/basic_commands/dual_barrel_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/full_auto_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/stop_feeder_command.hpp"
#include "subsystems/feeder/complex_commands/sentry_match_firing_control_command.hpp"
#include "subsystems/feeder/control/feeder.hpp"
//
#include "subsystems/gimbal/basic_commands/gimbal_chase_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_field_relative_control_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_patrol_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_toggle_aiming_command.hpp"
#include "subsystems/gimbal/complex_commands/sentry_match_gimbal_control_command.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "subsystems/gimbal/control/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"
//
#include "subsystems/shooter/basic_commands/brake_shooter_command.hpp"
#include "subsystems/shooter/basic_commands/run_shooter_command.hpp"
#include "subsystems/shooter/basic_commands/stop_shooter_command.hpp"
#include "subsystems/shooter/complex_commands/stop_shooter_comprised_command.hpp"
#include "subsystems/shooter/control/shooter.hpp"
//
#include "subsystems/hopper/basic_commands/close_hopper_command.hpp"
#include "subsystems/hopper/basic_commands/open_hopper_command.hpp"
#include "subsystems/hopper/complex_commands/toggle_hopper_command.hpp"
#include "subsystems/hopper/control/hopper.hpp"

using namespace src::Chassis;
using namespace src::Feeder;
using namespace src::Gimbal;
using namespace src::Shooter;
// using namespace src::Communication;
using namespace src::Control;
using namespace src::Hopper;

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

src::Utils::RefereeHelperTurreted refHelper(drivers(), currentBarrel, 30);

ChassisMatchStates chassisMatchState = src::Chassis::ChassisMatchStates::START;
// src::Control::FeederMatchStates feederMatchState = src::Control::FeederMatchStates::ANNOYED;

// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
FeederSubsystem feeder(drivers());
GimbalSubsystem gimbal(drivers());
// CommunicationResponseSubsytem response(*drivers());
ShooterSubsystem shooter(drivers(), &refHelper);
HopperSubsystem hopper(drivers());

// Informant Controllers
src::Informants::IMUCalibrateCommand imuCalibrateCommand(drivers(), &chassis, &gimbal);

// Robot Specific Controllers ------------------------------------------------
GimbalChassisRelativeController gimbalController(&gimbal);
GimbalFieldRelativeController gimbalFieldRelativeController(drivers(), &gimbal);

// Ballistics Solver
src::Utils::Ballistics::BallisticsSolver ballisticsSolver(drivers(), BARREL_POSITION_FROM_GIMBAL_ORIGIN);

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

GimbalPatrolConfig patrolConfig = {
    .pitchPatrolAmplitude = modm::toRadian(4.0f),
    .pitchPatrolFrequency = 1.0f * M_PI,
    .pitchPatrolOffset = -modm::toRadian(2.0f),
};

// Match Controllers ------------------------------------------------
SentryMatchFiringControlCommand matchFiringControlCommand(
    drivers(),
    &feeder,
    &shooter,
    &refHelper,
    &ballisticsSolver,
    &gimbalFieldRelativeController,
    chassisMatchState);

SentryMatchChassisControlCommand matchChassisControlCommand(
    drivers(),
    &chassis,
    &gimbal,
    chassisMatchState,
    &refHelper,
    defaultSnapConfig,
    defaultTokyoConfig,
    false,
    randomizerConfig);

SentryMatchGimbalControlCommand matchGimbalControlCommand(
    drivers(),
    &gimbal,
    &gimbalFieldRelativeController,
    &refHelper,
    &ballisticsSolver,
    patrolConfig,
    chassisMatchState,
    500);

// SentryMatchChassisControlCommand sentryMatchChassisControlCommand(drivers(),
//     &chassis, ChassisMatchStates::PATROL,&refHelper, defaultSnapConfig, defaultTokyoConfig, false, randomizerConfig);

// Define commands here ---------------------------------------------------
ChassisManualDriveCommand chassisManualDriveCommand(drivers(), &chassis);
ChassisToggleDriveCommand chassisToggleDriveCommand(
    drivers(),
    &chassis,
    &gimbal,
    defaultSnapConfig,
    defaultTokyoConfig,
    false,
    randomizerConfig);
ChassisTokyoCommand chassisTokyoCommand(drivers(), &chassis, &gimbal, defaultTokyoConfig, 0, false, randomizerConfig);
ChassisAutoNavCommand chassisAutoNavCommand(drivers(), &chassis, defaultLinearConfig, defaultRotationConfig);

GimbalPatrolCommand gimbalPatrolCommand(drivers(), &gimbal, &gimbalFieldRelativeController, patrolConfig, chassisMatchState);
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
// only for when dirving with remote

GimbalToggleAimCommand gimbalToggleAimCommand(
    drivers(),
    &gimbal,
    &gimbalFieldRelativeController,
    &refHelper,
    &ballisticsSolver,
    SHOOTER_SPEED_MATRIX[0][0]);

FullAutoFeederCommand runFeederCommand(drivers(), &feeder, &refHelper, 1, UNJAM_TIMER_MS);
FullAutoFeederCommand runFeederCommandFromMouse(drivers(), &feeder, &refHelper, 1, UNJAM_TIMER_MS);

DualBarrelFeederCommand dualBarrelsFeederCommand(drivers(), &feeder, &refHelper, BARREL_IDS, 1, UNJAM_TIMER_MS);

DualBarrelFeederCommand dualBarrelsFeederCommandFromMouse(drivers(), &feeder, &refHelper, BARREL_IDS, 1, UNJAM_TIMER_MS);

StopFeederCommand stopFeederCommand(drivers(), &feeder);

RunShooterCommand runShooterCommand(drivers(), &shooter, &refHelper);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter, &refHelper);
StopShooterComprisedCommand stopShooterComprisedCommand(drivers(), &shooter);

OpenHopperCommand openHopperCommand(drivers(), &hopper, HOPPER_OPEN_ANGLE);
OpenHopperCommand openHopperCommand2(drivers(), &hopper, HOPPER_OPEN_ANGLE);
CloseHopperCommand closeHopperCommand(drivers(), &hopper, HOPPER_CLOSED_ANGLE);
CloseHopperCommand closeHopperCommand2(drivers(), &hopper, HOPPER_CLOSED_ANGLE);
ToggleHopperCommand toggleHopperCommand(drivers(), &hopper, HOPPER_CLOSED_ANGLE, HOPPER_OPEN_ANGLE);

// CommunicationResponseHandler responseHandler(*drivers());

// Define command mappings here -------------------------------------------

// MANUAL ROBOT CONTROL (NON-SERVER USE) ----------------------------------
// // Enables both chassis and gimbal manual control
// HoldCommandMapping leftSwitchMid(
//     drivers(),  // gimbalFieldRelativeControlCommand
//     {&chassisToggleDriveCommand, &gimbalToggleAimCommand /*&gimbalChaseCommand*/},
//     RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// // Enables both chassis and gimbal control and closes hopper
// HoldCommandMapping leftSwitchUp(
//     drivers(),
//     {&chassisTokyoCommand, &gimbalChaseCommand2},
//     RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// // HoldCommandMapping rightSwitchDown(
// //     drivers(),
// //     {&openHopperCommand},
// //     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

// // Runs shooter only and closes hopper
// HoldCommandMapping rightSwitchMid(
//     drivers(),
//     {&runShooterCommand},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// // Runs shooter with feeder and closes hopper
// HoldRepeatCommandMapping rightSwitchUp(
//     drivers(),
//     {&runFeederCommand, &runShooterWithFeederCommand},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
//     true);

// Autonomous Match Control Switch Mapping -----------------------------
HoldCommandMapping leftSwitchMid(
    drivers(),
    {/*&imuCalibrateCommand,*/ &chassisToggleDriveCommand, &gimbalFieldRelativeControlCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

HoldCommandMapping leftSwitchUp(
    drivers(),
    {/*&chassisTokyoCommand,*/ &matchChassisControlCommand, &matchGimbalControlCommand, &matchFiringControlCommand
     /*&gimbalChaseCommand*/},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Runs shooter only
HoldCommandMapping rightSwitchMid(
    drivers(),
    {&runShooterCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder
HoldCommandMapping rightSwitchUp(
    drivers(),
    {&dualBarrelsFeederCommand, &runShooterWithFeederCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&gimbal);
    drivers->commandScheduler.registerSubsystem(&shooter);
    // drivers->commandScheduler.registerSubsystem(&response);

    drivers->kinematicInformant.registerSubsystems(&gimbal, &chassis);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
    feeder.initialize();
    gimbal.initialize();
    shooter.initialize();
    // response.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    shooter.setDefaultCommand(&stopShooterComprisedCommand);
    feeder.setDefaultCommand(&stopFeederCommand);
    // gimbal.setDefaultCommand(&gimbalControlCommand);
    // chassis.setDefaultCommand(&sentryMatchChassisControlCommand);
    // gimbal.setDefaultCommand(&gimbalChaseCommand);
}

// Set commands scheduled on startup
void startupCommands(src::Drivers *drivers) {
    // drivers->refSerial.attachRobotToRobotMessageHandler(SENTRY_RESPONSE_MESSAGE_ID, &responseHandler);

    // no startup commands should be set
    // yet...
    // TODO: Possibly add some sort of hardware test command
    //       that will move all the parts so we
    //       can make sure they're fully operational.
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&leftSwitchUp);

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