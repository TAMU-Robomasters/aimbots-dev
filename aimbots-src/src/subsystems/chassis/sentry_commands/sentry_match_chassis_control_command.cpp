#include "sentry_match_chassis_control_command.hpp"

namespace src::Chassis {

static constexpr float MAX_SAFE_DPS = 20.0f;
static constexpr uint32_t EVADE_DURATION_MS = 4000;

SentryMatchChassisControlCommand::SentryMatchChassisControlCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    ChassisMatchStates& chassisState,
    src::Utils::RefereeHelperTurreted* refHelper)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      chassis(chassis),
      chassisState(chassisState),
      refHelper(refHelper),
      chassisAutoNavCommand(drivers, chassis, defaultLinearConfig, defaultRotationConfig, defaultSnapConfig),
      ChassisAutoNavTokyoCommand(
          drivers,
          chassis,
          defaultLinearConfig,
          defaultTokyoConfig,
          false,
          randomizerConfig),  // velocity ramp value
      evadeTimeout(EVADE_DURATION_MS) {
    addSubsystemRequirement(chassis);
    this->comprisedCommandScheduler.registerSubsystem(chassis);
}

void SentryMatchChassisControlCommand::initialize() {
    chassis->setDefaultCommand(dynamic_cast<TapCommand*>(&chassisAutoNavCommand));
    chassisState = ChassisMatchStates::PATROL;
    chassisAutoNavCommand.setTargetLocation(locationStart);
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &chassisAutoNavCommand);
}

float dpsDisplay = 0.0f;
float currPatrolTarget = 0;  // 0 is A, 1 is B

void SentryMatchChassisControlCommand::execute() {
    if (refHelper.getGameStage < 2 || refHelper.getGameStage > 4) {
        descheduleIfScheduled(this->comprisedCommandScheduler, &chassisAutoNavCommand, interrupted);
    } else {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &chassisAutoNavCommand);
        // auto gameData = drivers->refSerial.getGameData();

        float recievedDPS = refHelper.getReceivedDPS();
        // auto gameState = gameData.gameStage;

        dpsDisplay = recievedDPS;

        if (receivedDps >= MAX_SAFE_DPS && chassisState != ChassisMatchStates::EVADE) {
            chassisState = ChassisMatchStates::EVADE;
            evadeTimeout.restart(EVADE_DURATION_MS);
        } else if (refHelper.getCurrHealthPercentage < .3 && chassisState != ChassisMatchStates::HEAL) {
            chassisState = ChassisMatchStates::HEAL;
        } else if (chassisState != ChassisMatchStates::PATROL) {
            chassisState = ChassisMatchStates::PATROL;
        }
        switch (chassisState) {
            case ChassisMatchStates::EVADE:
                if (!evadeTimeout.expired) {
                    scheduleIfNotScheduled(this->comprisedCommandScheduler, &sentryChassisAutoNavEvadeCommand);
                } else {
                    descheduleIfScheduled(this->comprisedCommandScheduler, &sentryChassisAutoNavEvadeCommand, interrupted);
                    chassisState = ChassisMatchStates::NONE;
                }
                // schedule evade command if not scheduled & not timed out
                // if timed out, remove from scheduler and change state to none

            case ChassisMatchStates::HEAL:
                chassisAutoNavCommand.setTargetLocation(locationHeal);
                currPatrolTarget = 1;

            case ChassisMatchStates::PATROL:  // Do we want to stop patrolling if we see someone and start shooting?
                if (chassisAutoNavCommand.isSettled) {
                    if (currPatrolTarget == 0) {
                        currPatrolTarget = 1;
                        chassisAutoNavCommand.setTargetLocation(locationB);
                    } else if (currPatrolTarget == 1) {
                        currPatrolTarget = 0;
                        chassisAutoNavCommand.setTargetLocation(locationA);
                    }
                } else {
                    if (currPatrolTarget == 0) {
                        chassisAutoNavCommand.setTargetLocation(locationA);
                    } else if (currPatrolTarget == 1) {
                        chassisAutoNavCommand.setTargetLocation(locationB);
                    }
                }
        }
    }

    this->comprisedCommandScheduler.run();
}

void SentryMatchChassisControlCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &patrolCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &evadeCommand, interrupted);
    chassisState = NONE;
}

bool SentryMatchChassisControlCommand::isReady() { return true; }

bool SentryMatchChassisControlCommand::isFinished() const { return false; }

}  // namespace src::Chassis