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
    src::Utils::RefereeHelper* refHelper,
    src::Chassis::ChassisMatchStates& chassisState)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      feeder(feeder),
      shooter(shooter),
      refHelper(refHelper),
      chassisState(chassisState),
      stopFeederCommand(drivers, feeder),
      burstFeederCommand(drivers, feeder, refHelper, BASE_BURST_LENGTH),
      fullAutoFeederCommand(drivers, feeder, refHelper),
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
}

void SentryMatchFiringControlCommand::execute() {
    // descheduleIfScheduled(this->comprisedCommandScheduler, &fullAutoFeederCommand, true);

    // if (1) {
    if (drivers->cvCommunicator.isJetsonOnline()) {
        if (chassisState != src::Chassis::ChassisMatchStates::EVADE &&
            drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::Vision::CVState::FIRE) {
            auto botData = drivers->refSerial.getRobotData();
            float healthPercentage = static_cast<float>(botData.currentHp) / static_cast<float>(botData.maxHp);
            float targetDepth = drivers->cvCommunicator.getLastValidMessage().targetZ;  //TODO: Replace this this the appropriate kinematic informant function

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
            // }
        } else {
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &stopFeederCommand);
        }
    } else {
        // descheduleIfScheduled(this->comprisedCommandScheduler, &burstFeederCommand, true);
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