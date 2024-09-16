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
#include "subsystems/chassis/basic_commands/chassis_manual_drive_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_tokyo_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_auto_nav_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_auto_nav_tokyo_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_toggle_drive_command.hpp"
#include "subsystems/chassis/control/chassis.hpp"
//
#include "subsystems/feeder/basic_commands/dual_barrel_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/full_auto_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/stop_feeder_command.hpp"
#include "subsystems/feeder/control/feeder.hpp"
//
#include "subsystems/gimbal/basic_commands/gimbal_chase_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_field_relative_control_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_patrol_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_toggle_aiming_command.hpp"
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
//
#include "subsystems/barrel_manager/barrel_manager.hpp"
#include "subsystems/barrel_manager/barrel_swap_command.hpp"
//
// #include "informants/communication/communication_response_handler.hpp"
// #include "informants/communication/communication_response_subsytem.hpp"
//
#include "subsystems/display/basic_commands/client_display_command.hpp"
#include "subsystems/display/control/client_display_subsystem.hpp"
//

using namespace src::Chassis;
using namespace src::Feeder;
using namespace src::Gimbal;
using namespace src::Shooter;

// For reference, all possible keyboard inputs:
// W,S,A,D,SHIFT,CTRL,Q,E,R,F,G,Z,X,C,V,B
/*  AERIAL Control Scheme:

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

namespace AerialControl {

// This is technically a command flag, but it needs to be defined before the barrel manager subsystem
BarrelID currentBarrel = BARREL_IDS[0];

src::Utils::RefereeHelperTurreted refHelper(drivers(), currentBarrel, 30);

// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
FeederSubsystem feeder(drivers());
GimbalSubsystem gimbal(drivers());
ShooterSubsystem shooter(drivers(), &refHelper);

// CommunicationResponseSubsytem response(*drivers());

// Robot Specific Controllers ------------------------------------------------
GimbalChassisRelativeController gimbalChassisRelativeController(&gimbal);
GimbalFieldRelativeController gimbalFieldRelativeController(drivers(), &gimbal);

// Ballistics Solver -------------------------------------------------------
src::Utils::Ballistics::BallisticsSolver ballisticsSolver(drivers(), BARREL_POSITION_FROM_GIMBAL_ORIGIN);

// Define behavior configs here --------------------------------------------

SnapSymmetryConfig defaultSnapConfig = {
    .numSnapPositions = 2,
    .snapAngle = modm::toRadian(0.0f),
};

TokyoConfig defaultTokyoConfig = {
    .translationalSpeedMultiplier = 0.6f,
    .translationThresholdToDecreaseRotationSpeed = 0.5f,
    .rotationalSpeedFractionOfMax = 0.75f,
    .rotationalSpeedMultiplierWhenTranslating = 0.7f,
    .rotationalSpeedIncrement = 30.0f,
};

SpinRandomizerConfig randomizerConfig = {
    .minSpinRateModifier = 0.75f,
    .maxSpinRateModifier = 1.0f,
    .minSpinRateModifierDuration = 500,
    .maxSpinRateModifierDuration = 3000,
};

GimbalPatrolConfig patrolConfig = {
    .pitchPatrolAmplitude = modm::toRadian(11.0f),
    .pitchPatrolFrequency = 1.5f * M_PI,
    .pitchPatrolOffset = -modm::toRadian(11.0f),
};

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

ChassisAutoNavCommand chassisAutoNavCommand(
    drivers(),
    &chassis,
    defaultLinearConfig,
    defaultRotationConfig,
    defaultSnapConfig);

ChassisAutoNavTokyoCommand chassisAutoNavTokyoCommand(
    drivers(),
    &chassis,
    defaultLinearConfig,
    defaultTokyoConfig,
    false,
    randomizerConfig);

GimbalPatrolCommand gimbalPatrolCommand(drivers(), &gimbal, &gimbalFieldRelativeController, patrolConfig);
GimbalControlCommand gimbalControlCommand(drivers(), &gimbal, &gimbalChassisRelativeController);
GimbalFieldRelativeControlCommand gimbalFieldRelativeControlCommand(drivers(), &gimbal, &gimbalFieldRelativeController);
GimbalFieldRelativeControlCommand gimbalFieldRelativeControlCommand2(drivers(), &gimbal, &gimbalFieldRelativeController);
GimbalChaseCommand gimbalChaseCommand(
    drivers(),
    &gimbal,
    &gimbalChassisRelativeController,
    &refHelper,
    &ballisticsSolver,
    SHOOTER_SPEED_MATRIX[0][0]);
GimbalChaseCommand gimbalChaseCommand2(
    drivers(),
    &gimbal,
    &gimbalChassisRelativeController,
    &refHelper,
    &ballisticsSolver,
    SHOOTER_SPEED_MATRIX[0][0]);
GimbalToggleAimCommand gimbalToggleAimCommand(
    drivers(),
    &gimbal,
    &gimbalChassisRelativeController,
    &refHelper,
    &ballisticsSolver,
    SHOOTER_SPEED_MATRIX[0][0]);

FullAutoFeederCommand runFeederCommand(drivers(), &feeder, &refHelper, FEEDER_DEFAULT_RPM, 3000.0f, UNJAM_TIMER_MS);
FullAutoFeederCommand runFeederCommandFromMouse(drivers(), &feeder, &refHelper, FEEDER_DEFAULT_RPM, 3000.0f, UNJAM_TIMER_MS);
// Raise the acceptable threshold on the feeder to let it trust the barrel manager will prevent overheat

StopFeederCommand stopFeederCommand(drivers(), &feeder);

RunShooterCommand runShooterCommand(drivers(), &shooter, &refHelper);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter, &refHelper);
StopShooterComprisedCommand stopShooterComprisedCommand(drivers(), &shooter);

// Define command mappings here -------------------------------------------
HoldCommandMapping leftSwitchMid(
    drivers(),  // gimbalFieldRelativeControlCommand
    {&gimbalToggleAimCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// Enables both chassis and gimbal control and closes hopper
HoldCommandMapping leftSwitchUp(
    drivers(),  // gimbalFieldRelativeControlCommand2
    {&gimbalChaseCommand2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// HoldCommandMapping rightSwitchDown(
//     drivers(),
//     {&},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

// Runs shooter only and closes hopper
HoldCommandMapping rightSwitchMid(
    drivers(),
    {&runShooterCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder and closes hopper
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&runFeederCommand, &runShooterWithFeederCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);

HoldCommandMapping leftClickMouse(
    drivers(),
    {&runFeederCommandFromMouse},
    RemoteMapState(RemoteMapState::MouseButton::LEFT));

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
    feeder.setDefaultCommand(&stopFeederCommand);
    shooter.setDefaultCommand(&stopShooterComprisedCommand);
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
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchMid);
    // drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&leftClickMouse);
}

}  // namespace AerialControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    AerialControl::initializeSubsystems();
    AerialControl::registerSubsystems(drivers);
    AerialControl::setDefaultCommands(drivers);
    AerialControl::startupCommands(drivers);
    AerialControl::registerIOMappings(drivers);
}
}  // namespace src::Control

// temp

#endif  // TARGET_AERIAL
