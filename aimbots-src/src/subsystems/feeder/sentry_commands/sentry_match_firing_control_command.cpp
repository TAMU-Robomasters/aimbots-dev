#include "utils/robot_specific_inc.hpp"
#ifdef TARGET_SENTRY

#include "sentry_match_firing_control_command.hpp"

namespace src::Control {

static constexpr int BASE_BURST_LENGTH = 3;
static constexpr int ANNOYED_BURST_LENGTH = 10;

static constexpr float MAX_FEEDER_SPEED = 500.0f;
static constexpr float MIN_FEEDER_SPEED = 70.0f;

float percentageToSpeed(float percentage) { return (MAX_FEEDER_SPEED - MIN_FEEDER_SPEED) * percentage + MIN_FEEDER_SPEED; }

SentryMatchFiringControlCommand::SentryMatchFiringControlCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    ShooterSubsystem* shooter,
    src::Utils::RefereeHelperTurreted* refHelper,
    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver,
    src::Gimbal::GimbalControllerInterface* fieldRelativeGimbalController,
    src::Chassis::ChassisMatchStates& chassisState)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      feeder(feeder),
      shooter(shooter),
      refHelper(refHelper),
      ballisticsSolver(ballisticsSolver),
      fieldRelativeGimbalController(fieldRelativeGimbalController),
      chassisState(chassisState),
      stopFeederCommand(drivers, feeder),
      burstFeederCommand(drivers, feeder, refHelper, BASE_BURST_LENGTH),
      fullAutoFeederCommand(drivers, feeder, refHelper, FEEDER_DEFAULT_RPM, -1500, UNJAM_TIMER_MS),
      stopShooterCommand(drivers, shooter),
      runShooterCommand(drivers, shooter, refHelper)  //
{
    this->comprisedCommandScheduler.registerSubsystem(feeder);
    this->comprisedCommandScheduler.registerSubsystem(shooter);
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void SentryMatchFiringControlCommand::initialize() {
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &stopFeederCommand);
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &runShooterCommand);
    shootMinimumTime.restart(0);
}

void SentryMatchFiringControlCommand::execute() {
    // if (1) {
    if (drivers->cvCommunicator.isJetsonOnline()) {
        // Assuming the cvState is found, we need to determine if the turret is within an armor panel's distance
        bool isErrorCloseEnoughToShoot = false;
        float targetDepth = drivers->cvCommunicator.getLastValidMessage().targetY;

        if (drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::Vision::CVState::FOUND) {
            float yawTargetGimbal = fieldRelativeGimbalController->getTargetYaw(AngleUnit::Radians);
            tap::algorithms::ContiguousFloat yawCurrentGimbal =
                drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsContiguousFloat();

            float pitchTargetGimbal = fieldRelativeGimbalController->getTargetPitch(AngleUnit::Radians);
            tap::algorithms::ContiguousFloat pitchCurrentGimbal =
                drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsContiguousFloat();

            isErrorCloseEnoughToShoot = ballisticsSolver->withinAimingTolerance(
                yawCurrentGimbal.difference(yawTargetGimbal),
                pitchCurrentGimbal.difference(pitchTargetGimbal),
                targetDepth);
        }

        if ((chassisState != src::Chassis::ChassisMatchStates::EVADE && isErrorCloseEnoughToShoot) || !shootMinimumTime.isExpired()) {
            auto botData = drivers->refSerial.getRobotData();
            float healthPercentage = static_cast<float>(botData.currentHp) / static_cast<float>(botData.maxHp);

            float feederSpeed = MAX_FEEDER_SPEED;
            float healthPressure = limitVal((1.0f - healthPercentage), 0.0f, 1.0f);  // inverts health percentage

            if (targetDepth <= 3.0f) {
                feederSpeed = MAX_FEEDER_SPEED;
            } else if (targetDepth <= 4.0f) {
                feederSpeed = percentageToSpeed(0.75f + healthPressure);
            } else if (targetDepth <= 5.0f) {
                feederSpeed = percentageToSpeed(0.4f + healthPressure);
            } else {
                feederSpeed = percentageToSpeed(0.0f + healthPressure);
            }

            // healthBasedFeederSpeed = (healthPercentage < 0.35f) ? MAX_FEEDER_SPEED : healthBasedFeederSpeed;
            // healthBasedFeederSpeed = (healthPercentage < 0.35f) ? MAX_FEEDER_SPEED : MIN_FEEDER_SPEED;
            feederSpeed = limitVal(feederSpeed, MIN_FEEDER_SPEED, MAX_FEEDER_SPEED);
            fullAutoFeederCommand.setSpeed(feederSpeed);

            scheduleIfNotScheduled(this->comprisedCommandScheduler, &fullAutoFeederCommand);
            shootMinimumTime.restart(500);
            // }
        } else {
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &stopFeederCommand);
            fullAutoFeederCommand.setSpeed(0.0f);
        }
    } else {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &stopFeederCommand);
    }

    this->comprisedCommandScheduler.run();
}

void SentryMatchFiringControlCommand::end(bool interrupted) {
    // descheduleIfScheduled(this->comprisedCommandScheduler, &burstFeederCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &fullAutoFeederCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &stopFeederCommand, interrupted);
}

bool SentryMatchFiringControlCommand::isReady() { return true; }

bool SentryMatchFiringControlCommand::isFinished() const { return false; }

}  // namespace src::Control

#endif