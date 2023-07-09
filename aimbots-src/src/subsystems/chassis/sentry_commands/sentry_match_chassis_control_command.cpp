#include "sentry_match_chassis_control_command.hpp"

#ifdef TARGET_SENTRY

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
    //chassis->setDefaultCommand(dynamic_cast<TapCommand*>(&chassisAutoNavCommand));
    chassisState = ChassisMatchStates::SETUP;

    //Because this is set in intialize, only waypointTarget should need to be updated to move the robot
    waypointTarget.setPosition(SENTRY_WAYPOINTS[waypointName::SENTRY_START]);
    autoNavCommand.setTargetLocation(waypointTarget);
    autoNavTokyoCommand.setTargetLocation(waypointTarget);

    //scheduleIfNotScheduled(this->comprisedCommandScheduler, &autoNavCommand);
}

float dpsDisplay = 0.0f;

void SentryMatchChassisControlCommand::execute() {

    if (refHelper->getGameStage() == GamePeriod::COUNTDOWN || refHelper->getGameStage() == GamePeriod::IN_GAME) {
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoCommand);
        /*matchTimer = MATCH_TIME_LENGTH - drivers->refSerial.getGameData().stageTimeRemaining;

        if (engageTokyo) {
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &autoNavTokyoCommand);
        }
        else {   
            scheduleIfNotScheduled(this->comprisedCommandScheduler, &autoNavCommand);
        }
        
        float receivedDPS = refHelper->getReceivedDPS();

        dpsDisplay = receivedDPS;

        //Logic for state swapping
        if (matchTimer > CENTRAL_BUFF_OPEN && (buffPointTimer.isExpired() || buffPointTimer.isStopped())) {
            updateChassisState(ChassisMatchStates::CAPTURE);
        }
        else {
            updateChassisState(ChassisMatchStates::AGGRO);
        }

        //What to do in each state
        switch (chassisState) {
            case ChassisMatchStates::SETUP:
                //Do nothing currently

            case ChassisMatchStates::CAPTURE:
                if (lastChassisState == ChassisMatchStates::AGGRO) {
                    waypointTarget.setPosition(SENTRY_WAYPOINTS[AGGRO_TO_CAPTURE[currPatrolIndex]]);
                    currentPathLength = AGGRO_TO_CAPTURE.size();
                }

            case ChassisMatchStates::AGGRO:
                if (lastChassisState == ChassisMatchStates::SETUP) {
                    waypointTarget.setPosition(SENTRY_WAYPOINTS[SETUP_TO_AGGRO[currPatrolIndex]]);
                    currentPathLength = SETUP_TO_AGGRO.size();
                }
                if (lastChassisState == ChassisMatchStates::CAPTURE) {
                    waypointTarget.setPosition(SENTRY_WAYPOINTS[CAPTURE_TO_AGGRO[currPatrolIndex]]);
                    currentPathLength = CAPTURE_TO_AGGRO.size();
                }
                
        }
        if (isNavSettled() && currPatrolIndex < (currentPathLength-1)) {
            currPatrolIndex++;
        }*/

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
    chassisState = ChassisMatchStates::SETUP;
}

bool SentryMatchChassisControlCommand::isReady() { return true; }

bool SentryMatchChassisControlCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif