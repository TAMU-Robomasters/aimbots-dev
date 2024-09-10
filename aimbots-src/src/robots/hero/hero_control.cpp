#include "utils/tools/robot_specific_defines.hpp"

#if defined(ALL_HEROES)

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
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_manual_drive_command.hpp"
#include "subsystems/chassis/chassis_toggle_drive_command.hpp"
#include "subsystems/chassis/chassis_tokyo_command.hpp"
//
#include "subsystems/feeder/basic_commands/full_auto_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/stop_feeder_command.hpp"
#include "subsystems/feeder/complex_commands/feeder_limit_command.hpp"
#include "subsystems/feeder/control/feeder.hpp"
//
#include "subsystems/gimbal/basic_commands/gimbal_chase_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_field_relative_control_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_toggle_aiming_command.hpp"
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
#include "informants/communication/communication_response_handler.hpp"
#include "informants/communication/communication_response_subsytem.hpp"
//
#include "subsystems/display/basic_commands/client_display_command.hpp"
#include "subsystems/display/control/client_display_subsystem.hpp"
//

using namespace src::Chassis;
using namespace src::Feeder;
using namespace src::Gimbal;
using namespace src::Shooter;
// using namespace src::Communication;
// using namespace src::RobotStates;
using namespace src::Utils::ClientDisplay;

// For reference, all possible keyboard inputs:
// W,S,A,D,SHIFT,CTRL,Q,E,R,F,G,Z,X,C,V,B
/*  Standard Control Scheme:

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
    Force Reload: B

    UI ----------------------------------------------------------------
    Force Update: Ctrl + B

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

namespace HeroControl {

// This is technically a command flag, but it needs to be defined before the refHelper
BarrelID currentBarrel = BARREL_IDS[0];

src::Utils::RefereeHelperTurreted refHelper(drivers(), currentBarrel, 0);

// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
FeederSubsystem feeder(drivers());
GimbalSubsystem gimbal(drivers());
ClientDisplaySubsystem clientDisplay(drivers());
ShooterSubsystem shooter(drivers(), &refHelper);

// Robot Specific Controllers ------------------------------------------------
GimbalChassisRelativeController gimbalChassisRelativeController(&gimbal);
GimbalFieldRelativeController gimbalFieldRelativeController(drivers(), &gimbal);

// Ballistics Solver ---------------------------------------------------------
src::Utils::Ballistics::BallisticsSolver ballisticsSolver(drivers(), BARREL_POSITION_FROM_GIMBAL_ORIGIN);

// Define behavior configs here -----------------------------------------------

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
ChassisFollowGimbalCommand chassisFollowGimbal(drivers(), &chassis, &gimbal);

ChassisToggleDriveCommand chassisToggleDriveCommand(
    drivers(),
    &chassis,
    &gimbal,
    defaultSnapConfig,
    defaultTokyoConfig,
    false,
    randomizerConfig);
ChassisTokyoCommand chassisTokyoCommand(drivers(), &chassis, &gimbal, defaultTokyoConfig);

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
GimbalToggleAimCommand gimbalToggleAimCommand(
    drivers(),
    &gimbal,
    &gimbalFieldRelativeController,
    &refHelper,
    &ballisticsSolver,
    SHOOTER_SPEED_MATRIX[0][0]);
GimbalToggleAimCommand gimbalToggleAimCommand2(
    drivers(),
    &gimbal,
    &gimbalFieldRelativeController,
    &refHelper,
    &ballisticsSolver,
    SHOOTER_SPEED_MATRIX[0][0]);

FullAutoFeederCommand runFeederCommand(drivers(), &feeder, &refHelper, 0, UNJAM_TIMER_MS);
FullAutoFeederCommand runFeederCommandFromMouse(drivers(), &feeder, &refHelper, 0, UNJAM_TIMER_MS);
StopFeederCommand stopFeederCommand(drivers(), &feeder);
FeederLimitCommand feederLimitCommand(drivers(), &feeder, &refHelper, UNJAM_TIMER_MS);

RunShooterCommand runShooterCommand(drivers(), &shooter, &refHelper);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter, &refHelper);
StopShooterComprisedCommand stopShooterComprisedCommand(drivers(), &shooter);

ClientDisplayCommand clientDisplayCommand(
    *drivers(),
    drivers()->commandScheduler,
    clientDisplay
    // chassis
);

// Define command mappings here -------------------------------------------
// Enables normal drive and gimbal field relative control. Enables CV toggle
HoldCommandMapping leftSwitchMid(
    drivers(),
    {&chassisToggleDriveCommand, &gimbalToggleAimCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// Enables Tokyo and Gimbal Field Relative Control. Also enables CV toggle
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&chassisTokyoCommand, &gimbalToggleAimCommand2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// HoldCommandMapping rightSwitchDown(
//     drivers(),
//     {&feederlimitcommand},
//     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

HoldCommandMapping rightSwitchMid(
    drivers(),
    {/*&feederlimitcommand,*/ &runShooterCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder and closes hopper
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&runFeederCommand, &runShooterWithFeederCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);

PressCommandMapping bCtrlPressed(drivers(), {&clientDisplayCommand}, RemoteMapState({Remote::Key::CTRL, Remote::Key::B}));

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
    // drivers->commandScheduler.registerSubsystem(&response);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);

    drivers->kinematicInformant.registerSubsystems(&gimbal, &chassis);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
    feeder.initialize();
    gimbal.initialize();
    shooter.initialize();
    // response.initialize();
    clientDisplay.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    feeder.setDefaultCommand(&stopFeederCommand);
    // feeder.setDefaultCommand(&feederLimitCommand);
    shooter.setDefaultCommand(&stopShooterComprisedCommand);
}

// Set commands scheduled on startup
void startupCommands(src::Drivers *drivers) {
    UNUSED(drivers);
    // drivers->refSerial.attachRobotToRobotMessageHandler(SENTRY_RESPONSE_MESSAGE_ID, &responseHandler);
    drivers->commandScheduler.addCommand(&clientDisplayCommand);

    // test
    //  no startup commands should be set
    //  yet...
    //  TODO: Possibly add some sort of hardware test command
    //        that will move all the parts so we
    //        can make sure they're fully operational.
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchMid);
    drivers->commandMapper.addMap(&bCtrlPressed);
    drivers->commandMapper.addMap(&leftClickMouse);
}
}  // namespace HeroControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    HeroControl::initializeSubsystems();
    HeroControl::registerSubsystems(drivers);
    HeroControl::setDefaultCommands(drivers);
    HeroControl::startupCommands(drivers);
    HeroControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif  // TARGET_HERO
