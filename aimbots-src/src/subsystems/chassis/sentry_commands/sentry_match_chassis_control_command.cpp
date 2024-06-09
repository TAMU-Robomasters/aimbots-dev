#include "sentry_match_chassis_control_command.hpp"

#ifdef TARGET_SENTRY
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
    updateChassisState(ChassisMatchStates::GUARDING);
    updateChassisState(ChassisMatchStates::START);
    // chassisState = ChassisMatchStates::START;

    // Because this is set in intialize, only waypointTarget should need to be updated to move the robot
    waypointTarget.setPosition(SENTRY_WAYPOINTS[waypointName::SENTRY_START]);
    // autoNavCommand.setTargetLocation(waypointTarget.getX(), waypointTarget.getY());
    autoNavCommand.setTargetLocation(3.5f, 3.5f);  // 3.5, 3.5
    startingTimer.restart(700);
    activeMovement = false;
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

void SentryMatchChassisControlCommand::execute() {
    if (refHelper->getGameStage() == GamePeriod::IN_GAME || true) {  // Remove the true later
        /*if (chassisState == lastChassisState) {
            switch (chassisState) {
                case ChassisMatchStates::START:
                    scheduleIfNotScheduled(this->comprisedCommandScheduler, &autoNavCommand);
                    autoNavCommand.setTargetLocation(3.5f, 3.5f);  // 3.0, 5.0
                    updateChassisState(ChassisMatchStates::GUARDING);
                    break;
                case ChassisMatchStates::GUARDING:
                    if (aggroTimer.execute()) {
                        scheduleIfNotScheduled(this->comprisedCommandScheduler, &autoNavCommand);
                        updateChassisState(ChassisMatchStates::MOVE_TO_RESUPPLY);
                        autoNavCommand.setTargetLocation(3.5f, 4.0f);  // 0.5, 1
                    }
                    break;
                    // if (drivers->refSerial.getRobotData().turret.bulletsRemaining17 < 30 ||
                    //     drivers->refSerial.getRobotData().currentHp < 200) {
                    //     chassisState = ChassisMatchStates::MOVE_TO_RESUPPLY;
                    // }
                    // break;

                case ChassisMatchStates::RESUPPLYING:
                    // if ((drivers->refSerial.getRobotData().turret.bulletsRemaining17 > 100 ||
                    //      drivers->refSerial.getRobotData().currentHp > 500) ||
                    //     false) {
                    //     chassisState = ChassisMatchStates::MOVE_TO_GUARD;
                    // }
                    break;
                case ChassisMatchStates::CAPTURE:
                    if () {
                        scheduleIfNotScheduled(this->comprisedCommandScheduler, &autoNavCommand);
                        autoNavCommand.setTargetLocation(6.0f, 4.0f);
                        updateChassisState(ChassisMatchStates::GUARDING);
                        break;
                    }

                case ChassisMatchStates::AGGRO:
                    // autoNavCommand.setTargetLocation(3.0f, 5.0f);
                    // chassisState = ChassisMatchStates::GUARDING;
                    break;
                default:
                    break;
            }
        } else if (autoNavCommand.isSettled()) {
            updateChassisState(chassisState);
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoCommand);
            aggroTimer.restart(5000);
        }*/

        matchTimer = MATCH_TIME_LENGTH - drivers->refSerial.getGameData().stageTimeRemaining;

        if (activeMovement) {
            // Do nothing, administration only

            if (autoNavCommand.isSettled()) {
                activeMovement = false;
                updateChassisState(chassisState);
                scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoCommand);
            }
        } else {
            if (chassisState == lastChassisState) {
                // Decide what state to enter
                if (drivers->refSerial.getRobotData().currentHp < 400) {
                    updateChassisState(ChassisMatchStates::RETREAT);
                } else {
                    switch (chassisState) {
                        case ChassisMatchStates::START:
                            if (startingTimer.execute()) {
                                updateChassisState(ChassisMatchStates::AGGRO);
                            }
                            break;
                        case ChassisMatchStates::GUARDING:
                            if (holdPositionTimer.execute()) {
                                updateChassisState(ChassisMatchStates::CAPTURE);
                            }
                            break;
                        case ChassisMatchStates::CAPTURE:
                            if (buffPointTimer.execute()) {
                                updateChassisState(ChassisMatchStates::RETREAT);
                            }
                            break;
                        case ChassisMatchStates::AGGRO:
                            if (aggroTimer.execute()) {
                                updateChassisState(ChassisMatchStates::GUARDING);
                            }
                            break;
                    }
                }
            }
            if (chassisState != lastChassisState) {
                // Perform setup for the next state
                scheduleIfNotScheduled(this->comprisedCommandScheduler, &autoNavCommand);
                switch (chassisState) {
                    case ChassisMatchStates::START:
                        autoNavCommand.setTargetLocation(3.5f, 3.0f);  // 3.0, 5.0
                        startingTimer.restart(700);
                        break;
                    case ChassisMatchStates::GUARDING:
                        autoNavCommand.setTargetLocation(3.5f, 4.0f);
                        holdPositionTimer.restart(5000);
                        break;
                    case ChassisMatchStates::CAPTURE:
                        autoNavCommand.setTargetLocation(6.0f, 4.0f);
                        buffPointTimer.restart(BUFF_POINT_REFRESH_TIME);
                        break;
                    case ChassisMatchStates::AGGRO:
                        autoNavCommand.setTargetLocation(6.0f, 2.5f);  // 9.0, 1.5
                        aggroTimer.restart(5000);
                        break;
                    case ChassisMatchStates::RETREAT:
                        autoNavCommand.setTargetLocation(1.5f, 5.0f);
                        break;
                }

                activeMovement = true;
            }
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