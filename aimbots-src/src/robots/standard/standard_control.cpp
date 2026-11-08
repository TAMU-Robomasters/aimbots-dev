#include "utils/tools/robot_specific_defines.hpp"

#if defined(ALL_STANDARDS)

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
#include "tap/control/remote_map_state.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//
#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/chassis/complex_commands/chassis_auto_nav_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_auto_nav_tokyo_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_manual_drive_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_toggle_drive_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_tokyo_command.hpp"
//#include "subsystems/chassis/basic_commands/chassis_shakira_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_toggle_drive_custom_controller_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_tokyo_master_command.hpp"
//
#include "subsystems/feeder/basic_commands/dual_barrel_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/full_auto_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/stop_feeder_command.hpp"
#include "subsystems/feeder/control/feeder.hpp"
//
#include "subsystems/gimbal/basic_commands/gimbal_chase_command.hpp"
#include "subsystems/gimbal/basic_commands/gimbal_control_command.hpp"
#include "subsystems/gimbal/basic_commands/gimbal_position_PID_tunning_command.hpp"
#include "subsystems/gimbal/basic_commands/gimbal_velocity_PID_tunning_command.hpp"
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
#include "subsystems/hopper/basic_commands/close_hopper_command.hpp"
#include "subsystems/hopper/basic_commands/open_hopper_command.hpp"
#include "subsystems/hopper/complex_commands/toggle_hopper_command.hpp"
#include "subsystems/hopper/control/hopper.hpp"
//
//#include "subsystems/barrel_manager/barrel_manager.hpp"
//#include "subsystems/barrel_manager/barrel_swap_command.hpp"
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
using namespace src::Hopper;
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

    Decrease Chassis Ground Speed (60%): Shift
    Decrease Chassis Ground Speed (25%): Ctrl

    Gimbal ------------------------------------------------------------
    Aim Using CV: Right Mouse Button

    Shooter -----------------------------------------------------------

    Feeder ------------------------------------------------------------
    Full Auto Shooting: Left Mouse Button

    Hopper ------------------------------------------------------------
    Toggle Hopper Position: C

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
HopperSubsystem hopper(drivers());
ClientDisplaySubsystem clientDisplay(drivers());

// Command Flags ----------------------------
bool barrelMovingFlag = true;
bool barrelCaliDoneFlag = false;

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
    .translationalSpeedMultiplier = 0.75f,
    .translationThresholdToDecreaseRotationSpeed = 0.5f,
    .rotationalSpeedFractionOfMax = 1.0f,  // 0.75
    .rotationalSpeedMultiplierWhenTranslating = 0.75f,
    .rotationalSpeedIncrement = 30.0f,
};

SpinRandomizerConfig randomizerConfig = {
    .minSpinRateModifier = 0.75f,
    .maxSpinRateModifier = 1.0f,
    .minSpinRateModifierDuration = 500,
    .maxSpinRateModifierDuration = 3000,
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

ChassisToggleDriveCustomControllerCommand chassisToggleDriveCustomControllerCommand(
    drivers(),
    &chassis,
    &gimbal,
    defaultTokyoConfig,
    true,
    randomizerConfig,
    5000.0f,
    10000.0f);

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
    SHOOTER_SPEED_MATRIX[0][0],
    modm::toRadian(30.0f));

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

FullAutoFeederCommand runFeederCommand(drivers(), &feeder, &refHelper, 0, UNJAM_TIMER_MS);
FullAutoFeederCommand runFeederCommandFromMouse(drivers(), &feeder, &refHelper, 0, UNJAM_TIMER_MS);

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

// client display
ClientDisplayCommand clientDisplayCommand(*drivers(), drivers()->commandScheduler, clientDisplay);

// Define command mappings here -------------------------------------------
// Each mapping keeps a pointer to its RemoteMapState, so the states must outlive the mappings.
RemoteMapState leftSwitchUpState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
RemoteMapState leftSwitchMidState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID);
RemoteMapState rightSwitchUpState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP);
RemoteMapState rightSwitchMidState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID);
RemoteMapState rightSwitchDownState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN);
RemoteMapState leftClickMouseState(RemoteMapState::MouseButton::LEFT);
RemoteMapState bCtrlPressedState({Remote::Key::CTRL, Remote::Key::B});

// develop's CommandMapper owns its mappings (addMap takes a std::unique_ptr), so they are
// moved into it in registerIOMappings().
auto leftSwitchMid = std::make_unique<HoldCommandMapping>(
    drivers(),  // gimbalFieldRelativeControlCommand
    std::vector<Command *>{&chassisToggleDriveCustomControllerCommand, &gimbalFieldRelativeControlCommand},
    &leftSwitchMidState);

// Enables both chassis and gimbal control and closes hopper
auto leftSwitchUp = std::make_unique<HoldCommandMapping>(
    drivers(),  // gimbalFieldRelativeControlCommand2
    std::vector<Command *>{&chassisTokyoCommand, &gimbalChaseCommand2},
    //{&gimbalVelocityTunningCommand},
    //{&gimbalPositionTunningCommand},
    &leftSwitchUpState);

auto rightSwitchDown = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&openHopperCommand},
    &rightSwitchDownState);

// Runs shooter only and closes hopper
auto rightSwitchMid = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&runShooterCommand, &toggleHopperCommand},
    &rightSwitchMidState);

// Runs shooter with feeder and closes hopper
auto rightSwitchUp = std::make_unique<HoldRepeatCommandMapping>(
    drivers(),
    std::vector<Command *>{&runFeederCommand, &runShooterWithFeederCommand, &closeHopperCommand2},
    &rightSwitchUpState,
    true);

auto leftClickMouse = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&runFeederCommandFromMouse},
    &leftClickMouseState);

// The user can press b+ctrl when the remote right switch is in the down position to restart the
// client display command. This is necessary since we don't know when the robot is connected to the
// server and thus don't know when to start sending the initial HUD graphics.
auto bCtrlPressed = std::make_unique<PressCommandMapping>(
    drivers(),
    std::vector<Command *>{&clientDisplayCommand},
    &bCtrlPressedState);

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&gimbal);
    drivers->commandScheduler.registerSubsystem(&shooter);
    drivers->commandScheduler.registerSubsystem(&hopper);
    // drivers->commandScheduler.registerSubsystem(&barrelManager);
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
    // response.initialize();
    clientDisplay.initialize();
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
 //   drivers->commandScheduler.addCommand(&clientDisplayCommand);
    drivers->commandScheduler.addCommand(&clientDisplayCommand);
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    // Same order as before. (main added bCtrlPressed twice; a mapping can only be owned once now.)
    drivers->commandMapper.addMap(std::move(leftSwitchUp));
    drivers->commandMapper.addMap(std::move(leftSwitchMid));
    drivers->commandMapper.addMap(std::move(rightSwitchUp));
    drivers->commandMapper.addMap(std::move(rightSwitchMid));
    drivers->commandMapper.addMap(std::move(rightSwitchDown));
    drivers->commandMapper.addMap(std::move(leftClickMouse));
    drivers->commandMapper.addMap(std::move(bCtrlPressed));
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

#endif  // TARGET_STANDARD
