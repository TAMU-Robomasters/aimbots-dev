#ifdef TARGET_CVTESTBENCH

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
#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/chassis/basic_commands/chassis_manual_drive_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_shakira_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_toggle_drive_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_tokyo_command.hpp"
//
#include "subsystems/feeder/basic_commands/full_auto_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/stop_feeder_command.hpp"
#include "subsystems/feeder/control/feeder.hpp"
//
#include "subsystems/gimbal/basic_commands/gimbal_chase_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_field_relative_control_command.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "subsystems/gimbal/control/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"
//
#include "subsystems/shooter/basic_commands/brake_shooter_command.hpp"
#include "subsystems/shooter/basic_commands/run_shooter_command.hpp"
#include "subsystems/shooter/control/shooter.hpp"
#include "subsystems/shooter/basic_commands/stop_shooter_command.hpp"
#include "subsystems/shooter/complex_commands/stop_shooter_comprised_command.hpp"
//
#include "subsystems/hopper/basic_commands/close_hopper_command.hpp"
#include "subsystems/hopper/basic_commands/open_hopper_command.hpp"
#include "subsystems/hopper/complex_commands/toggle_hopper_command.hpp"
#include "subsystems/hopper/control/hopper.cpp"
//
#include "subsystems/gui/gui_display.hpp"
#include "subsystems/gui/gui_display_command.hpp"
//
#include "subsystems/barrel_manager/barrel_manager.hpp"
#include "subsystems/barrel_manager/barrel_swap_command.hpp"

using namespace src::Chassis;
using namespace src::Feeder;
using namespace src::Gimbal;
using namespace src::Shooter;
using namespace src::Hopper;
using namespace src::GUI;

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

namespace CVTestbenchControl {

BarrelID currentBarrel = BarrelID::TURRET_17MM_1;

src::Utils::RefereeHelperTurreted refHelper(drivers(), currentBarrel);

// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
FeederSubsystem feeder(drivers());
GimbalSubsystem gimbal(drivers());
ShooterSubsystem shooter(drivers(), &refHelper);
HopperSubsystem hopper(
    drivers(),
    HOPPER_PIN,
    HOPPER_MAX_PWM,
    HOPPER_MIN_PWM,
    HOPPER_PWM_RAMP_SPEED,
    HOPPER_MIN_ANGLE,
    HOPPER_MAX_ANGLE,
    HOPPER_MIN_ACTION_DELAY);

// Robot Specific Controllers ------------------------------------------------
GimbalChassisRelativeController gimbalChassisRelativeController(&gimbal);
GimbalFieldRelativeController gimbalFieldRelativeController(drivers(), &gimbal);

// Ballistics Solver -------------------------------------------------------
src::Utils::Ballistics::BallisticsSolver ballisticsSolver(drivers(), BARREL_POSITION_FROM_GIMBAL_ORIGIN);

// Configs -----------------------------------------------------------------
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
ChassisTokyoCommand chassisTokyoCommand(drivers(), &chassis, &gimbal, defaultTokyoConfig, 0, false, randomizerConfig);
// ChassisShakiraCommand chassisShakiraCommand(
//     drivers(),
//     &chassis,
//     &gimbal,
//     &ROTATION_POSITION_PID_CONFIG,
//     &ballisticsSolver,
//     &refHelper,
//     4,
//     modm::toRadian(45.0f),
//     3000);

GimbalControlCommand gimbalControlCommand(drivers(), &gimbal, &gimbalChassisRelativeController);
GimbalFieldRelativeControlCommand gimbalFieldRelativeControlCommand(drivers(), &gimbal, &gimbalFieldRelativeController);
GimbalFieldRelativeControlCommand gimbalFieldRelativeControlCommand2(drivers(), &gimbal, &gimbalFieldRelativeController);
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
FullAutoFeederCommand runFeederCommand(drivers(), &feeder, &refHelper, FEEDER_DEFAULT_RPM, 0.80f);
FullAutoFeederCommand runFeederCommandFromMouse(drivers(), &feeder, &refHelper, FEEDER_DEFAULT_RPM, 0.80f);
StopFeederCommand stopFeederCommand(drivers(), &feeder);

RunShooterCommand runShooterCommand(drivers(), &shooter, &refHelper);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter, &refHelper);
StopShooterComprisedCommand stopShooterComprisedCommand(drivers(), &shooter);

OpenHopperCommand openHopperCommand(drivers(), &hopper, HOPPER_OPEN_ANGLE);
OpenHopperCommand openHopperCommand2(drivers(), &hopper, HOPPER_OPEN_ANGLE);
CloseHopperCommand closeHopperCommand(drivers(), &hopper, HOPPER_CLOSED_ANGLE);
CloseHopperCommand closeHopperCommand2(drivers(), &hopper, HOPPER_CLOSED_ANGLE);
ToggleHopperCommand toggleHopperCommand(drivers(), &hopper, HOPPER_CLOSED_ANGLE, HOPPER_OPEN_ANGLE);

// Define command mappings here -------------------------------------------
HoldCommandMapping leftSwitchMid(
    drivers(),  // gimbalFieldRelativeControlCommand
    {/*&chassisToggleDriveCommand,*/ &gimbalChaseCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// Enables both chassis and gimbal control and closes hopper
HoldCommandMapping leftSwitchUp(
    drivers(),  // gimbalFieldRelativeControlCommand2
    {/*&chassisTokyoCommand,*/ &gimbalFieldRelativeControlCommand2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// opens hopper
HoldCommandMapping rightSwitchDown(
    drivers(),
    {&openHopperCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

// Runs shooter only and closes hopper
HoldCommandMapping rightSwitchMid(
    drivers(),
    {&runShooterCommand, &closeHopperCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder and closes hopper
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&runFeederCommand, &runShooterWithFeederCommand, &closeHopperCommand2},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);

HoldCommandMapping leftClickMouse(
    drivers(),
    {&runFeederCommandFromMouse},
    RemoteMapState(RemoteMapState::MouseButton::LEFT));

// HoldCommandMapping rightClickMouse(
//     drivers(),
//     {&},
//     RemoteMapState(RemoteMapState::MouseButton::LEFT));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&gimbal);
    drivers->commandScheduler.registerSubsystem(&shooter);
    drivers->commandScheduler.registerSubsystem(&hopper);

    drivers->kinematicInformant.registerSubsystems(&gimbal, &chassis);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
    feeder.initialize();
    gimbal.initialize();
    shooter.initialize();
    hopper.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    feeder.setDefaultCommand(&stopFeederCommand);
    shooter.setDefaultCommand(&stopShooterComprisedCommand);
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
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&leftClickMouse);
}

}  // namespace CVTestbenchControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    CVTestbenchControl::initializeSubsystems();
    CVTestbenchControl::registerSubsystems(drivers);
    CVTestbenchControl::setDefaultCommands(drivers);
    CVTestbenchControl::startupCommands(drivers);
    CVTestbenchControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif  // TARGET_STANDARD
