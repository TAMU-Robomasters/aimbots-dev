#ifdef TARGET_STANDARD

#include "utils/common_types.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"

//
#include "informants/transformers/robot_frames.hpp"
#include "utils/ballistics_solver.hpp"
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
#include "subsystems/chassis/chassis_auto_nav_command.hpp"
#include "subsystems/chassis/chassis_auto_nav_tokyo_command.hpp"
#include "subsystems/chassis/chassis_manual_drive_command.hpp"
#include "subsystems/chassis/chassis_toggle_drive_command.hpp"
#include "subsystems/chassis/chassis_tokyo_command.hpp"
//
#include "subsystems/feeder/barrel_swapping_feeder_command.hpp"
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
#include "subsystems/gimbal/sentry_commands/gimbal_patrol_command.hpp"
//
#include "subsystems/shooter/brake_shooter_command.hpp"
#include "subsystems/shooter/run_shooter_command.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/stop_shooter_command.hpp"
#include "subsystems/shooter/stop_shooter_comprised_command.hpp"
//
#include "subsystems/hopper/close_hopper_command.hpp"
#include "subsystems/hopper/hopper.hpp"
#include "subsystems/hopper/open_hopper_command.hpp"
#include "subsystems/hopper/toggle_hopper_command.hpp"
//
#include "subsystems/barrel_manager/barrel_manager.hpp"
#include "subsystems/barrel_manager/barrel_swap_command.hpp"
//
// #include "informants/communication/communication_response_handler.hpp"
// #include "informants/communication/communication_response_subsytem.hpp"
//
#include "utils/display/client_display_command.hpp"
#include "utils/display/client_display_subsystem.hpp"
//

using namespace src::Chassis;
using namespace src::Feeder;
using namespace src::Gimbal;
using namespace src::Shooter;
using namespace src::Hopper;
using namespace src::BarrelManager;
// using namespace src::Communication;
// using namespace src::RobotStates;
using namespace src::Utils::ClientDisplay;
using namespace src::BarrelManager;

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

namespace StandardControl {

// This is technically a command flag, but it needs to be defined before the barrel manager subsystem
BarrelID currentBarrel = BARREL_IDS[0];

src::Utils::RefereeHelperTurreted refHelper(drivers(), currentBarrel, 30);

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

BarrelManagerSubsystem barrelManager(
    drivers(),
    HARD_STOP_OFFSET,
    BARREL_SWAP_DISTANCE_MM,
    BARRELS_ALIGNED_TOLERANCE,
    LEAD_SCREW_TICKS_PER_MM,
    LEAD_SCREW_CURRENT_SPIKE_TORQUE,
    LEAD_SCREW_CALI_OUTPUT,
    BARREL_SWAP_POSITION_PID_CONFIG,
    BARREL_IDS,
    currentBarrel);

// Command Flags ----------------------------
bool barrelMovingFlag = true;
bool barrelCaliDoneFlag = false;

// CommunicationResponseSubsytem response(*drivers());
ClientDisplaySubsystem clientDisplay(drivers());

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

FullAutoFeederCommand runFeederCommand(drivers(), &feeder, &refHelper, FEEDER_DEFAULT_RPM, 3000.0f, UNJAM_TIMER_MS);
FullAutoFeederCommand runFeederCommandFromMouse(drivers(), &feeder, &refHelper, FEEDER_DEFAULT_RPM, 3000.0f, UNJAM_TIMER_MS);
// Raise the acceptable threshold on the feeder to let it trust the barrel manager will prevent overheat
BarrelSwappingFeederCommand runDoubleBarrelFeederCommand(
    drivers(),
    &feeder,
    &refHelper,
    barrelMovingFlag,
    FEEDER_DEFAULT_RPM,
    3000.0f,
    UNJAM_TIMER_MS);
BarrelSwappingFeederCommand runDoubleBarrelFeederCommandFromMouse(
    drivers(),
    &feeder,
    &refHelper,
    barrelMovingFlag,
    FEEDER_DEFAULT_RPM,
    3000.0f,
    UNJAM_TIMER_MS);
StopFeederCommand stopFeederCommand(drivers(), &feeder);

RunShooterCommand runShooterCommand(drivers(), &shooter, &refHelper);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter, &refHelper);
StopShooterComprisedCommand stopShooterComprisedCommand(drivers(), &shooter);

BarrelSwapCommand barrelSwapperCommand(drivers(), &barrelManager, &refHelper, barrelMovingFlag, barrelCaliDoneFlag);

OpenHopperCommand openHopperCommand(drivers(), &hopper, HOPPER_OPEN_ANGLE);
OpenHopperCommand openHopperCommand2(drivers(), &hopper, HOPPER_OPEN_ANGLE);
CloseHopperCommand closeHopperCommand(drivers(), &hopper, HOPPER_CLOSED_ANGLE);
CloseHopperCommand closeHopperCommand2(drivers(), &hopper, HOPPER_CLOSED_ANGLE);
ToggleHopperCommand toggleHopperCommand(drivers(), &hopper, HOPPER_CLOSED_ANGLE, HOPPER_OPEN_ANGLE);

// CommunicationResponseHandler responseHandler(*drivers());

// client display
ClientDisplayCommand clientDisplayCommand(*drivers(), drivers()->commandScheduler, clientDisplay, &hopper, chassis);

// Define command mappings here -------------------------------------------
HoldCommandMapping leftSwitchMid(
    drivers(),  // gimbalFieldRelativeControlCommand
    {&chassisToggleDriveCommand, &gimbalToggleAimCommand /*&gimbalChaseCommand*/},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// Enables both chassis and gimbal control and closes hopper
HoldCommandMapping leftSwitchUp(
    drivers(),  // gimbalFieldRelativeControlCommand2
    {&chassisTokyoCommand, /*&chassisAutoNavTokyoCommand,*/ &gimbalChaseCommand2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

HoldCommandMapping rightSwitchDown(
    drivers(),
    {&openHopperCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

// Runs shooter only and closes hopper
HoldCommandMapping rightSwitchMid(
    drivers(),
    {&runShooterCommand, &toggleHopperCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder and closes hopper
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&runDoubleBarrelFeederCommand, &runShooterWithFeederCommand, &closeHopperCommand2},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);

HoldCommandMapping leftClickMouse(
    drivers(),
    {&runDoubleBarrelFeederCommandFromMouse},
    RemoteMapState(RemoteMapState::MouseButton::LEFT));

// The user can press b+ctrl when the remote right switch is in the down position to restart the
// client display command. This is necessary since we don't know when the robot is connected to the
// server and thus don't know when to start sending the initial HUD graphics.
PressCommandMapping bCtrlPressed(drivers(), {&clientDisplayCommand}, RemoteMapState({Remote::Key::B}));

// This is the command for starting up the GUI.  Uncomment once subsystem does something more useful.
/*PressCommandMapping ctrlC(
    drivers(),
    {&guiDisplayCommand},
    RemoteMapState({Remote::Key::CTRL, Remote::Key::C}));*/

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&gimbal);
    drivers->commandScheduler.registerSubsystem(&shooter);
    drivers->commandScheduler.registerSubsystem(&hopper);
    drivers->commandScheduler.registerSubsystem(&barrelManager);
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
    hopper.initialize();
    barrelManager.initialize();
    // response.initialize();
    clientDisplay.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    feeder.setDefaultCommand(&stopFeederCommand);
    shooter.setDefaultCommand(&stopShooterComprisedCommand);
    barrelManager.setDefaultCommand(&barrelSwapperCommand);
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
    // drivers->commandMapper.addMap(&leftSwitchUp);
    // drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchDown);
    // drivers->commandMapper.addMap(&leftClickMouse);
    drivers->commandMapper.addMap(&bCtrlPressed);
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

// temp

#endif  // TARGET_STANDARD
