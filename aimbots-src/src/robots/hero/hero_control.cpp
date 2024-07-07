#ifdef TARGET_HERO

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
#include "subsystems/chassis/chassis_manual_drive_command.hpp"
#include "subsystems/chassis/chassis_toggle_drive_command.hpp"
#include "subsystems/chassis/chassis_tokyo_command.hpp"
//
#include "subsystems/feeder/auto_agitator_indexer_command.hpp"
#include "subsystems/feeder/feeder.hpp"
#include "subsystems/feeder/full_auto_feeder_command.hpp"
#include "subsystems/feeder/stop_feeder_command.hpp"
//
#include "subsystems/indexer/burst_indexer_command.hpp"
#include "subsystems/indexer/full_auto_indexer_command.hpp"
#include "subsystems/indexer/indexer.hpp"
#include "subsystems/indexer/stop_indexer_command.hpp"
//
#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/controllers/gimbal_field_relative_controller.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "subsystems/gimbal/gimbal_control_command.hpp"
#include "subsystems/gimbal/gimbal_field_relative_control_command.hpp"
//
#include "subsystems/shooter/brake_shooter_command.hpp"
#include "subsystems/shooter/run_shooter_command.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/stop_shooter_command.hpp"
#include "subsystems/shooter/stop_shooter_comprised_command.hpp"
//
#include "informants/communication/communication_response_handler.hpp"
#include "informants/communication/communication_response_subsytem.hpp"
//
 #include "utils/display/client_display_command.hpp"
 #include "utils/display/client_display_subsystem.hpp"
//

using namespace src::Chassis;
using namespace src::Feeder;
using namespace src::Indexer;
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

    Feeder/Indexer ------------------------------------------------------------
    Full Auto Shooting: Left Mouse Button
    Force Reload: B

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

namespace HeroControl {

// This is technically a command flag, but it needs to be defined before the refHelper
BarrelID currentBarrel = BARREL_IDS[0];

src::Utils::RefereeHelperTurreted refHelper(drivers(), currentBarrel, 0);

// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
FeederSubsystem feeder(drivers());
IndexerSubsystem indexer(drivers(), INDEXER_ID, INDEX_BUS, INDEXER_DIRECTION, INDEXER_VELOCITY_PID_CONFIG);
GimbalSubsystem gimbal(drivers());
ClientDisplaySubsystem clientDisplay(drivers());
ShooterSubsystem shooter(drivers(), &refHelper);

// Robot Specific Controllers ------------------------------------------------
GimbalChassisRelativeController gimbalChassisRelativeController(&gimbal);
GimbalFieldRelativeController gimbalFieldRelativeController(drivers(), &gimbal);


//Define behavior configs here -----------------------------------------------

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

ChassisToggleDriveCommand chassisToggleDriveCommand(drivers(), 
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

ClientDisplayCommand clientDisplayCommand(
    *drivers(),
    drivers()->commandScheduler,
    clientDisplay,
    chassis
);

FullAutoFeederCommand runFeederCommand(drivers(), &feeder, &refHelper, FEEDER_DEFAULT_RPM, 3000.0f, 1, UNJAM_TIMER_MS);
FullAutoFeederCommand runFeederCommandFromMouse(drivers(), &feeder, &refHelper, FEEDER_DEFAULT_RPM, 3000.0f, 1, UNJAM_TIMER_MS);
StopFeederCommand stopFeederCommand(drivers(), &feeder);

FullAutoIndexerCommand runIndexerCommand(drivers(), &indexer, &refHelper, INDEXER_DEFAULT_RPM, 0.50f);
FullAutoIndexerCommand runIndexerCommandFromMouse(drivers(), &indexer, &refHelper, INDEXER_DEFAULT_RPM, 0.50f);
StopIndexerCommand stopIndexerCommand(drivers(), &indexer);

AutoAgitatorIndexerCommand feederIndexerCommand(
    drivers(),
    &feeder,
    &indexer,
    &refHelper,
    FEEDER_DEFAULT_RPM,
    INDEXER_DEFAULT_RPM,
    0.5f,
    UNJAM_TIMER_MS,
    1);

RunShooterCommand runShooterCommand(drivers(), &shooter, &refHelper);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter, &refHelper);
StopShooterComprisedCommand stopShooterComprisedCommand(drivers(), &shooter);

// Define command mappings here -------------------------------------------
HoldCommandMapping leftSwitchMid(
    drivers(),
    {&chassisToggleDriveCommand, &gimbalFieldRelativeControlCommand, &feederIndexerCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// Enables both chassis and gimbal control and closes hopper
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&chassisTokyoCommand, &gimbalFieldRelativeControlCommand2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));


HoldCommandMapping rightSwitchMid(
    drivers(),
    {&runShooterCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder and closes hopper
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&runFeederCommand, &runIndexerCommand, &runShooterWithFeederCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);

// PressCommandMapping bCtrlPressed(drivers(), {&clientDisplayCommand}, RemoteMapState({Remote::Key::B}));

HoldCommandMapping leftClickMouse(
    drivers(),
    {&runFeederCommandFromMouse, &runIndexerCommandFromMouse},
    RemoteMapState(RemoteMapState::MouseButton::LEFT));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&gimbal);
    drivers->commandScheduler.registerSubsystem(&shooter);
    // drivers->commandScheduler.registerSubsystem(&response);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
    drivers->commandScheduler.registerSubsystem(&indexer);

    drivers->kinematicInformant.registerSubsystems(&gimbal, &chassis);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
    feeder.initialize();
    indexer.initialize();
    gimbal.initialize();
    shooter.initialize();
    // response.initialize();
    clientDisplay.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    feeder.setDefaultCommand(&stopFeederCommand);
    indexer.setDefaultCommand(&stopIndexerCommand);
    shooter.setDefaultCommand(&stopShooterComprisedCommand);
}

// Set commands scheduled on startup
void startupCommands(src::Drivers *drivers) {
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
    // drivers->commandMapper.addMap(&bCtrlPressed);
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
