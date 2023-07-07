#include "sentry_match_chassis_control_command.hpp"

namespace src::Chassis {

static constexpr float MAX_SAFE_DPS = 20.0f;
static constexpr uint32_t EVADE_DURATION_MS = 4000;

SentryMatchChassisControlCommand::SentryMatchChassisControlCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    ChassisMatchStates& chassisState,
    src::Utils::RefereeHelperTurreted* refHelper,
    const SnapSymmetryConfig& snapSymmetryConfig,
    const TokyoConfig& tokyoConfig,
    bool randomizeSpinRate,
    const SpinRandomizerConfig& randomizerConfig)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      chassis(chassis),
      chassisState(chassisState),
      refHelper(refHelper),
      autoNavCommand(drivers, chassis, defaultLinearConfig, defaultRotationConfig, snapSymmetryConfig),
      autoNavTokyoCommand(
          drivers,
          chassis,
          defaultLinearConfig,
          tokyoConfig,
          randomizeSpinRate,
          randomizerConfig),  // velocity ramp value
      evadeTimeout(EVADE_DURATION_MS) {
    addSubsystemRequirement(chassis);
    this->comprisedCommandScheduler.registerSubsystem(chassis);
}

void SentryMatchChassisControlCommand::initialize() {
    //chassis->setDefaultCommand(dynamic_cast<TapCommand*>(&chassisAutoNavCommand));
    chassisState = ChassisMatchStates::PATROL;

    modm::Location2D<float> locationStart({-5.300f, -0.176f}, modm::toRadian(0.0f));
    modm::Location2D<float> locationA(TARGET_A, modm::toRadian(0.0f));
    modm::Location2D<float> locationB(TARGET_B, modm::toRadian(0.0f));
    modm::Location2D<float> locationHeal(TARGET_HEAL, modm::toRadian(0.0f));

    autoNavCommand.setTargetLocation(locationStart);
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &autoNavCommand);
}

float dpsDisplay = 0.0f;
float currPatrolTarget = 0;  // 0 is A, 1 is B

void SentryMatchChassisControlCommand::execute() {
    //TODO: This is cringe, ask Sid for help fixing later
    modm::Location2D<float> locationA(TARGET_A, modm::toRadian(0.0f));
    modm::Location2D<float> locationB(TARGET_B, modm::toRadian(0.0f));
    modm::Location2D<float> locationHeal(TARGET_HEAL, modm::toRadian(0.0f));


    if (static_cast<int>(refHelper->getGameStage()) < 2 || static_cast<int>(refHelper->getGameStage()) > 4) {
        descheduleIfScheduled(this->comprisedCommandScheduler, &autoNavCommand, true);
    } else {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &autoNavCommand);
        // auto gameData = drivers->refSerial.getGameData();

        float receivedDPS = refHelper->getReceivedDPS();
        // auto gameState = gameData.gameStage;

        dpsDisplay = receivedDPS;

        if (receivedDPS >= MAX_SAFE_DPS && chassisState != ChassisMatchStates::EVADE) {
            chassisState = ChassisMatchStates::EVADE;
            evadeTimeout.restart(EVADE_DURATION_MS);
        } else if (refHelper->getCurrHealthPercentage() < .3 && chassisState != ChassisMatchStates::HEAL) {
            chassisState = ChassisMatchStates::HEAL;
        } else if (chassisState != ChassisMatchStates::PATROL) {
            chassisState = ChassisMatchStates::PATROL;
        }
        switch (chassisState) {
            case ChassisMatchStates::EVADE:
                if (!evadeTimeout.isExpired()) {
                    //scheduleIfNotScheduled(this->comprisedCommandScheduler, &autoNavEvadeCommand);
                } else {
                    //descheduleIfScheduled(this->comprisedCommandScheduler, &sentryChassisAutoNavEvadeCommand, true);
                    chassisState = ChassisMatchStates::NONE;
                }
                // schedule evade command if not scheduled & not timed out
                // if timed out, remove from scheduler and change state to none

            case ChassisMatchStates::HEAL:
                autoNavCommand.setTargetLocation(locationHeal);
                currPatrolTarget = 1;

            case ChassisMatchStates::PATROL:  // Do we want to stop patrolling if we see someone and start shooting?
                if (autoNavCommand.isSettled()) {
                    if (currPatrolTarget == 0) {
                        currPatrolTarget = 1;
                        autoNavCommand.setTargetLocation(locationB);
                    } else if (currPatrolTarget == 1) {
                        currPatrolTarget = 0;
                        autoNavCommand.setTargetLocation(locationA);
                    }
                } else {
                    if (currPatrolTarget == 0) {
                        autoNavCommand.setTargetLocation(locationA);
                    } else if (currPatrolTarget == 1) {
                        autoNavCommand.setTargetLocation(locationB);
                    }
                }
        }
    }

    this->comprisedCommandScheduler.run();
}

void SentryMatchChassisControlCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &autoNavCommand, interrupted);
    chassisState = NONE;
}

bool SentryMatchChassisControlCommand::isReady() { return true; }

bool SentryMatchChassisControlCommand::isFinished() const { return false; }

}  // namespace src::Chassis