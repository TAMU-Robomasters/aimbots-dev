#include "utils/tools/robot_specific_defines.hpp"

#if defined(ALL_SENTRIES)

// Sentry-only constant headers (these define BARREL_IDS etc.) must stay inside the
// ALL_SENTRIES guard — leaking them at file scope clashes with other robots' constants
// when their targets are compiled.
#include "robots/sentry/constants/sentry_feeder_constants.hpp"
#include "robots/sentry/constants/sentry_shooter_constants.hpp"
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
#include "tap/control/governor/governor_with_fallback_command.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//
#include "utils/ref_system/game_started_governor.hpp"
//
#include "informants/imu/calibrate_imu_command.hpp"
//
#include "subsystems/chassis/basic_commands/chassis_manual_drive_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_tokyo_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_tokyo_master_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_auto_nav_velocity_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_auto_nav_tokyo_velocity_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_auto_tokyo_power_limited_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_toggle_drive_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_toggle_drive_custom_controller_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_toggle_drive_ignore_gimbal_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_auto_nav_command.hpp"
#include "subsystems/chassis/control/chassis.hpp"
//
#include "subsystems/feeder/basic_commands/dual_barrel_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/full_auto_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/stop_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/feeder_velocity_pid_tunning.hpp"
#include "subsystems/feeder/complex_commands/feeder_limit_command.hpp"
#include "subsystems/feeder/complex_commands/feeder_shot_timing_command.hpp"
#include "subsystems/feeder/complex_commands/autoaim_feeder_command.hpp"
#include "subsystems/feeder/control/feeder.hpp"
//
#include "subsystems/gimbal/basic_commands/gimbal_chase_command.hpp"
#include "subsystems/gimbal/basic_commands/gimbal_position_PID_tunning_command.hpp"
#include "subsystems/gimbal/basic_commands/gimbal_velocity_PID_tunning_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_field_relative_control_command.hpp"
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
    .translationalSpeedMultiplier = 1.0f,
    .translationThresholdToDecreaseRotationSpeed = 0.25f,
    .rotationalSpeedFractionOfMax = 0.8f,
    .rotationalSpeedMultiplierWhenTranslating = 0.5,
    .rotationalSpeedIncrement = 20.0f,
};

SpinRandomizerConfig randomizerConfig = {
    // fr sin spin settings change min/max SpinRateModifier changes sin wave amp range
    .minSpinRateModifier = 0.5f, 
    .maxSpinRateModifier = 0.7f, 
    .minSpinRateModifierDuration = 500,
    .maxSpinRateModifierDuration = 3000,
};

GimbalPatrolConfig patrolConfig = {
    .pitchPatrolAmplitude = modm::toRadian(15.0f),
    .pitchPatrolFrequency = 5.0f,
    .pitchPatrolOffset = -modm::toRadian(10.0f),
    .yawPatrolAngularVelocityDegreesPerSec = 80.0f,
};

GimbalVelocityTunningConfig gimbalYawVelocityTunningConfig = {
    .velocityAmplitudeDegreesPerSec = 60.0f,
    .frequencyHz = .2f,
};

GimbalVelocityTunningConfig gimbalPitchVelocityTunningConfig = {
    .velocityAmplitudeDegreesPerSec = 30.0f,
    .frequencyHz = .2f,
};

GimbalPositionTunningConfig gimbalYawPositionTunningConfig = {
    .positionAmplitudeDegrees = 30.0f,
    .frequencyHz = 0.5f,
};

GimbalPositionTunningConfig gimbalPitchPositionTunningConfig = {
    .positionAmplitudeDegrees = 20.0f,
    .frequencyHz = 0.5f,
};

FeederVelocityTunningConfig feederVelocityTunningConfig = {
    .VelocityAmplitudeRPM = 60.0f,
    .frequencyHz = 1.0f,
};

// Define commands here ---------------------------------------------------

// ?: Chat do we still need this what is this for
// SentryMatchChassisControlCommand matchChassisControlCommand(
//     drivers(),
//     &chassis,
//     &gimbal,
//     chassisMatchState,
//     &refHelper,
//     defaultSnapConfig,
//     defaultTokyoConfig,
//     false,
//     randomizerConfig);

SentryMatchGimbalControlCommand matchGimbalControlCommand(
    drivers(),
    &gimbal,
    &gimbalFieldRelativeController,
    &refHelper,
    &ballisticsSolver,
    patrolConfig,
    chassisMatchState,
    500,
    SHOOTER_SPEED_MATRIX[0][0]);

// SentryMatchChassisControlCommand sentryMatchChassisControlCommand(drivers(),
//     &chassis, ChassisMatchStates::PATROL,&refHelper, defaultSnapConfig, defaultTokyoConfig, false, randomizerConfig);

// Define commands here ---------------------------------------------------
ChassisManualDriveCommand chassisManualDriveCommand(drivers(), &chassis);

//chassis follow gimbal toggle command
ChassisToggleDriveCommand chassisToggleDriveCommand(
    drivers(),
    &chassis,
    &gimbal,
    defaultSnapConfig,
    defaultTokyoConfig,
    false,
    randomizerConfig);

//chassis ignore gimbal toggle command
ChassisToggleDriveIgnoreGimbalCommand chassisToggleDriveIgnoreGimbalCommand(
    drivers(),
    &chassis,
    &gimbal,
    defaultTokyoConfig,
    false,
    randomizerConfig);
ChassisToggleDriveIgnoreGimbalCommand chassisToggleDriveIgnoreGimbalCommand2(
    drivers(),
    &chassis,
    &gimbal,
    defaultTokyoConfig,
    false,
    randomizerConfig);

// chassis custom-controller toggle drive (ignore-gimbal + tokyo master, power limited)
ChassisToggleDriveCustomControllerCommand chassisToggleDriveCustomControllerCommand(
    drivers(),
    &chassis,
    &gimbal,
    defaultTokyoConfig,
    true,
    randomizerConfig,
    6500.0f,
    10000.0f);

ChassisTokyoCommand chassisTokyoCommand(drivers(), &chassis, &gimbal, defaultTokyoConfig, 0, true, randomizerConfig);
// Disabled: its constructor builds the A* visibility graph (soft-float doubles + heap) during static init,
// which hangs boot before main(). Sentry navigates with Nav2 on the Jetson instead.
// ChassisAutoNavCommand chassisAutoNavCommand(drivers(), &chassis, defaultLinearConfig, defaultRotationConfig);
// Drives the chassis from nav2's turret-relative velocity command over the Jetson link.
// Not yet mapped to a switch — wire into a HoldCommandMapping when ready.
ChassisAutoNavVelocityCommand chassisAutoNavVelocityCommand(drivers(), &chassis, &gimbal);
// "Spin-to-win" version of the above: nav2 velocity translation + continuous tokyo spin.
// Not yet mapped to a switch — wire into a HoldCommandMapping when ready.
ChassisAutoNavTokyoVelocityCommand chassisAutoNavTokyoVelocityCommand(
    drivers(),
    &chassis,
    &gimbal,
    defaultTokyoConfig,
    0,
    true,
    randomizerConfig);

// Auto (nav2-driven) tokyo via the master command: translation comes from the Jetson's
// field-relative velocity, operator/custom-controller input is ignored (isAuto=true).
ChassisTokyoMasterCommand nav2TokyoMasterCommand(
    drivers(),
    &chassis,
    &gimbal,
    defaultTokyoConfig,
    0,                                  // spinDirectionOverride (0 = random)
    true,                               // randomizeSpinRate
    randomizerConfig,
    ChassisTokyoMasterMode::NORMAL,     // mode
    0.0f,                               // joystick2OverrideVelocity (ignored in auto)
    5000.0f,                            // maxWheelSpeed
    true);                              // isAuto

// Manual (operator-driven) tokyo via the master command: translation comes from the operator,
// used before the match starts (isAuto=false).
ChassisTokyoMasterCommand manualTokyoMasterCommand(
    drivers(),
    &chassis,
    &gimbal,
    defaultTokyoConfig,
    0,                                  // spinDirectionOverride (0 = random)
    true,                               // randomizeSpinRate
    randomizerConfig,
    ChassisTokyoMasterMode::NORMAL,     // mode
    0.0f,                               // joystick2OverrideVelocity
    5000.0f,                            // maxWheelSpeed
    false);                             // isAuto

// Auto tokyo with ESP power limiting: wraps a Jetson-driven (isAuto=true) tokyo master and
// dynamically reduces the wheel-speed ceiling from the ESP power sensor, same logic as
// ChassisToggleDriveCustomControllerCommand but with all operator/custom-controller input removed.
ChassisAutoTokyoPowerLimitedCommand autoTokyoPowerLimitedCommand(
    drivers(),
    &chassis,
    &gimbal,
    defaultTokyoConfig,
    true,                               // randomizeSpinRate
    randomizerConfig,
    15000000.0f);                           // maxWheelSpeed (pre-power-limit ceiling)

// Gate the chassis on the referee game stage: pre-game runs the fallback; once the ref system
// reports IN_GAME it hands off to the power-limited Jetson-driven auto tokyo. Both must require
// only the chassis subsystem (GovernorWithFallbackCommand asserts identical requirement sets).
// The trailing `true` force-ends the fallback the instant the game starts so the handoff is
// immediate (requires the HoldRepeatCommandMapping on leftSwitchUp to re-add this command).
// src::Utils::GameStartedGovernor gameStartedGovernor(&refHelper);
// governor::GovernorWithFallbackCommand<1> gameGatedTokyoCommand(
//     {&chassis},
//     autoTokyoPowerLimitedCommand,   // governors ready (game started) -> power-limited auto tokyo
//     chassisToggleDriveIgnoreGimbalCommand2, // fallback (pre-game) -> manual tokyo
//     {&gameStartedGovernor},
//     true);

// GimbalPatrolCommand gimbalPatrolCommand(drivers(), &gimbal, &gimbalFieldRelativeController, patrolConfig, chassisMatchState);
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

GimbalVelocityTunningCommand gimbalVelocityTunningCommand(
    drivers(), 
    &gimbal,     
    &gimbalFieldRelativeController, 
    gimbalYawVelocityTunningConfig,
    gimbalPitchVelocityTunningConfig);

GimbalPositionTunningCommand gimbalPositionTunningCommand(
    drivers(), 
    &gimbal,     
    &gimbalFieldRelativeController, 
    gimbalYawPositionTunningConfig,
    gimbalPitchPositionTunningConfig);

GimbalToggleAimCommand gimbalToggleAimCommand(
    drivers(),
    &gimbal,
    &gimbalFieldRelativeController,
    &refHelper,
    &ballisticsSolver,
    SHOOTER_SPEED_MATRIX[0][0]);

FullAutoFeederCommand runFeederCommand(drivers(), &feeder, &refHelper, 1, UNJAM_TIMER_MS);
FullAutoFeederCommand runFeederCommandFromMouse(drivers(), &feeder, &refHelper, 1, UNJAM_TIMER_MS);
FeederLimitCommand feederLimitCommand(drivers(), &feeder, &refHelper, UNJAM_TIMER_MS);
FeederShotTimingCommand feederShotTimingCommand(drivers(), &feeder, &refHelper, UNJAM_TIMER_MS);
AutoAimFeederCommand autoAimFeederCommand(drivers(), &feeder, &refHelper, BARREL_IDS, 1, UNJAM_TIMER_MS);
// Separate instance for the right-mouse mapping so it doesn't share the feeder subsystem
// requirement with rightSwitchUp's autoAimFeederCommand (holding both would interrupt each other).
AutoAimFeederCommand autoAimFeederCommandFromMouse(drivers(), &feeder, &refHelper, BARREL_IDS, 1, UNJAM_TIMER_MS);

DualBarrelFeederCommand dualBarrelsFeederCommand(drivers(), &feeder, &refHelper, BARREL_IDS, 1, UNJAM_TIMER_MS);

DualBarrelFeederCommand dualBarrelsFeederCommandFromMouse(drivers(), &feeder, &refHelper, BARREL_IDS, 1, UNJAM_TIMER_MS);

StopFeederCommand stopFeederCommand(drivers(), &feeder);

FeederVelocityTunningCommand feederVelocityTunningCommand(
    drivers(), 
    &feeder,     
    feederVelocityTunningConfig); 

RunShooterCommand runShooterCommand(drivers(), &shooter, &refHelper);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter, &refHelper);
// Separate shooter instance for the right-mouse auto-feeder mapping (see autoAimFeederCommandFromMouse).
RunShooterCommand runShooterFromMouseCommand(drivers(), &shooter, &refHelper);
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
    // Manual driving: custom-controller toggle drive + manual gimbal aiming.
    // {&chassisToggleDriveCustomControllerCommand, &gimbalFieldRelativeControlCommand},
   {&chassisToggleDriveIgnoreGimbalCommand2, &gimbalFieldRelativeControlCommand2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// HoldRepeat (not Hold): gameGatedTokyoCommand self-finishes the instant the game starts (its
// stopFallbackCommandIfGovernorsReady=true) so the GovernorWithFallbackCommand can be re-added and
// re-run isReady(), handing off manual->auto tokyo. A plain HoldCommandMapping only adds once on
// entering UP and would NOT re-add after the self-finish, leaving the chassis dead until you toggle
// the switch off UP and back. -1 (default) = reschedule forever while held; true = end when released.
HoldRepeatCommandMapping leftSwitchUp(
    drivers(),
   // {&chassisToggleDriveIgnoreGimbalCommand2, &gimbalChaseCommand2},
   // {&chassisToggleDriveIgnoreGimbalCommand2, &matchGimbalControlCommand},
   // {&chassisAutoNavVelocityCommand, &gimbalChaseCommand2},
    // {&chassisAutoNavVelocityCommand, &gimbalFieldRelativeControlCommand2},
    // {&chassisAutoNavTokyoVelocityCommand, &gimbalFieldRelativeControlCommand2},
    // {&chassisAutoNavTokyoVelocityCommand, &gimbalChaseCommand2},
    // {&feederVelocityTunningCommand},
    //{/*&imuCalibrateCommand,*/ &chassisTokyoCommand, &gimbalFieldRelativeControlCommand},
    // {&gimbalVelocityTunningCommand},
    // {&gimbalPositionTunningCommand},
    // {&chassisTokyoCommand, &gimbalChaseCommand2},
     // {&nav2TokyoMasterCommand, &gimbalChaseCommand2},
     // {&nav2TokyoMasterCommand, &gimbalFieldRelativeControlCommand2},
     // {&nav2TokyoMasterCommand, &matchGimbalControlCommand},
     // Pre-game: manual tokyo. Once the ref system reports IN_GAME: nav2 auto tokyo.
     {&autoTokyoPowerLimitedCommand, &gimbalFieldRelativeControlCommand},
     // {&gameGatedTokyoCommand, &matchGimbalControlCommand},
     // {&nav2TokyoMasterCommand},
    // {/*&chassisTokyoCommand,*/ &matchChassisControlCommand, &matchGimbalControlCommand, &matchFiringControlCommand
    // {&chassisAutoNavCommand, &gimbalToggleAimCommand /*&gimbalChaseCommand*/},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
    true);  // endCommandsWhenNotHeld: stop both commands when the switch leaves UP

// Runs shooter only
HoldCommandMapping rightSwitchMid(
    drivers(),
    // {&feederShotTimingCommand, &runShooterCommand},
    {&autoAimFeederCommand, &runShooterCommand}, 
     // {&runShooterCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Auto feeder (CV/auto-aim gated) + flywheel
HoldCommandMapping rightSwitchUp(
    drivers(),
    {&dualBarrelsFeederCommand, &runShooterWithFeederCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

HoldCommandMapping leftClickMouse(
    drivers(),
    {&runFeederCommandFromMouse},
    RemoteMapState(RemoteMapState::MouseButton::LEFT));

// Right mouse: auto feeder (CV/auto-aim gated) + flywheel
HoldCommandMapping rightClickMouse(
    drivers(),
    {&autoAimFeederCommandFromMouse, &runShooterFromMouseCommand},
    RemoteMapState(RemoteMapState::MouseButton::RIGHT));

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
    drivers->commandMapper.addMap(&leftClickMouse);
    drivers->commandMapper.addMap(&rightClickMouse);

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

#endif  // ALL_SENTRIES
