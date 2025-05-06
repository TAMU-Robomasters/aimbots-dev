#include "sentry_match_chassis_control_command.hpp"

#ifdef ALL_SENTRIES
#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

static constexpr float MAX_SAFE_DPS = 20.0f;
static constexpr uint32_t EVADE_DURATION_MS = 4000;

SentryMatchChassisControlCommand::SentryMatchChassisControlCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    src::Gimbal::GimbalSubsystem* gimbal,
    ChassisMatchStates& chassisState,
    src::Utils::RefereeHelperTurreted* refHelper,
    const SnapSymmetryConfig& snapSymmetryConfig,
    const TokyoConfig& tokyoConfig,
    bool randomizeSpinRate,
    const SpinRandomizerConfig& randomizerConfig)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
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
      tokyoCommand(drivers, chassis, gimbal, tokyoConfig, 0, randomizeSpinRate, randomizerConfig),
      evadeTimeout(EVADE_DURATION_MS) {
    addSubsystemRequirement(chassis);
    this->comprisedCommandScheduler.registerSubsystem(chassis);
}

void SentryMatchChassisControlCommand::initialize() {
    // This is a necessary step, do not question it.
    // updateChassisState(ChassisMatchStates::GUARDING);
    // updateChassisState(ChassisMatchStates::START);
    chassisState = ChassisMatchStates::GUARDING;
    // chassisState = ChassisMatchStates::START;
    updateChassisState(ChassisMatchStates::START);
    delayTimer.restart(500);
    lockoutTimer.restart(0);

    // autoNavCommand.setTargetLocation(3.5f, 3.5f);  // 3.5, 3.5
    // startingTimer.restart(700);
    // activeMovement = false;
    /*
     * Reload
     *  Zone
     *
     * +y
     * ^
     * |
     * |
     * |
     *  Our
     *  Base -----> +x
     */
}

float dpsDisplay = 0.0f;

int stateDisplay = 7;
bool activeDisplay = false;
bool activeDisplay2 = false;

void SentryMatchChassisControlCommand::execute() {
    if (refHelper->getGameStage() == GamePeriod::IN_GAME) {  // Remove the true later

        stateDisplay = chassisState;
        activeDisplay = activeMovement;

        matchTimer = MATCH_TIME_LENGTH - drivers->refSerial.getGameData().stageTimeRemaining;

        if (activeMovement) {
            // Do nothing, upkeep only
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &autoNavCommand);
            if (autoNavCommand.isSettled() && delayTimer.isExpired()) {
                activeMovement = false;
                lockoutTimer.restart(STATE_LOCKOUT_TIMES[chassisState]);
                // updateChassisState(chassisState);
            }
        } else {
            delayTimer.restart(500);
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoCommand);
        }

        activeDisplay2 = activeMovement;

        // Logic for which state to use
        // if (drivers->refSerial.getRobotData().currentHp < 400 && updateChassisState(ChassisMatchStates::RESUPPLYING)) {
        //     updateChassisState(ChassisMatchStates::RETREAT);
        // }

        if (lockoutTimer.isExpired()) {
            if (chassisState == ChassisMatchStates::START) {
                // updateChassisState(ChassisMatchStates::AGGRO);
                updateChassisState(ChassisMatchStates::GUARDING);
            } else if (chassisState == ChassisMatchStates::AGGRO && !activeMovement) {
                updateChassisState(ChassisMatchStates::CAPTURE);
            } else if (chassisState == ChassisMatchStates::RETREAT && matchTimer > (60 * 4)) {
                updateChassisState(ChassisMatchStates::RESUPPLYING);
            } else if (
                (chassisState == ChassisMatchStates::RETREAT || chassisState == ChassisMatchStates::RESUPPLYING) &&
                drivers->refSerial.getRobotData().currentHp >= 400) {
                updateChassisState(ChassisMatchStates::CAPTURE);
            }
        }

        if (chassisState != lastChassisState) {
            // Perform setup for the next state

            switch (chassisState) {
                case ChassisMatchStates::START:
                    autoNavCommand.setTargetLocation(3.1f, 3.5f);  // 3.1, 3.5
                    break;
                case ChassisMatchStates::GUARDING:
                    autoNavCommand.setTargetLocation(3.0f, 4.5f);  // 3.0, 4.0
                    break;
                case ChassisMatchStates::CAPTURE:
                    autoNavCommand.setTargetLocation(6.5f, 3.7f);  // 6.4, 3.7
                    break;
                case ChassisMatchStates::AGGRO:
                    autoNavCommand.setTargetLocation(9.0f, 0.5f);  // 9.0, 1.5
                    break;
                case ChassisMatchStates::RETREAT:
                    autoNavCommand.setTargetLocation(1.5f, 6.0f);  // 1.5, 5.0
                    break;
                case ChassisMatchStates::RESUPPLYING:
                    autoNavCommand.setTargetLocation(0.5f, 5.0f);  // 0.5, 5.0
                case ChassisMatchStates::EVADE:
                default:
                    break;
            }

            lastChassisState = chassisState;
        }

    } else {
        descheduleIfScheduled(this->comprisedCommandScheduler, &autoNavCommand, true);
        descheduleIfScheduled(this->comprisedCommandScheduler, &autoNavTokyoCommand, true);
        descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoCommand, true);
    }

    this->comprisedCommandScheduler.run();
}

void SentryMatchChassisControlCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &autoNavCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &autoNavTokyoCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoCommand, interrupted);
    chassisState = ChassisMatchStates::START;
}

bool SentryMatchChassisControlCommand::isReady() { return true; }

bool SentryMatchChassisControlCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif  //#ifdef CHASSIS_COMPATIBLE
#endif