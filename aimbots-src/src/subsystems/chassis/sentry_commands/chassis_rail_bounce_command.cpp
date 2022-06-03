#include "chassis_rail_bounce_command.hpp"

namespace src::Chassis {

ChassisRailBounceCommand::ChassisRailBounceCommand(src::Drivers* drivers, ChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis)
#ifdef TARGET_SENTRY
      ,
      profileConstraints(
          {0.5f,     // max velocity
           1.0f,     // max acceleration
           10.0f}),  // max jerk
      railTargetIndex(1),
      railTraverseProfile(nullptr)
#endif
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));

    static constexpr float railTargetsArray[2] = {leftRailBound, rightRailBound};
    railTargets = Matrix<float, 2, 1>(railTargetsArray);
}

float displacementTargetDisplay = 0.0f;

void ChassisRailBounceCommand::initialize() {
    movementStartTime = tap::arch::clock::getTimeMilliseconds();

    float currRailPosition = drivers->fieldRelativeInformant.getRailRelativeRobotPosition()[0][X];

    float displacementTarget = railTargets[railTargetIndex][X] - currRailPosition;
    displacementTargetDisplay = displacementTarget;

    if (railTraverseProfile != nullptr) {
        delete railTraverseProfile;
    }
    railTraverseProfile = new SCurveMotionProfile(profileConstraints, displacementTarget);
}

float currRailPositionDisplay = 0.0f;
float stepVelocityDisplay = 0.0f;
float expectedMovementTimeDisplay = 0.0f;
float motorTargetRPMDisplay = 0.0f;
float railTargetDisplay = 0.0f;
int railTargetIndexDisplay = 0;

void ChassisRailBounceCommand::execute() {
    float currTime = tap::arch::clock::getTimeMilliseconds();
    float currRailPosition = drivers->fieldRelativeInformant.getRailRelativeRobotPosition()[0][X];
    currRailPositionDisplay = currRailPosition;
    railTargetDisplay = railTargets[railTargetIndex][X];
    railTargetIndexDisplay = railTargetIndex;

    // if (chassisProfile.isSettled(railTargets[railTargetIndex][X] - currRailPosition, 0.03f)) {
    //     railTargetIndex = (railTargetIndex + 1) % railTargets.getNumberOfRows();

    //     if (railTraverseProfile != nullptr) {
    //         delete railTraverseProfile;
    //     }
    //     // railTraverseProfile = new SCurveMotionProfile(profileConstraints, railTargets[railTargetIndex][X] - currRailPosition);
    //     railTraverseProfile = new SCurveMotionProfile(profileConstraints, -2.0f);
    //     movementStartTime = tap::arch::clock::getTimeMilliseconds();
    // }
    expectedMovementTimeDisplay = railTraverseProfile->totalTime();

    auto step = railTraverseProfile->stepAtTime((currTime - movementStartTime) / 1000.0f);
    stepVelocityDisplay = step.velocity;

    float wheelTargetRPM = (step.velocity / (2.0f * M_PI * WHEEL_RADIUS)) * 60.0f;
    float motorTargetRPM = wheelTargetRPM / CHASSIS_GEARBOX_RATIO;
    motorTargetRPMDisplay = -motorTargetRPM;

    // chassis->setTargetRPMs(-motorTargetRPM, 0.0f, 0.0f);
}

void ChassisRailBounceCommand::end(bool) {
    if (railTraverseProfile != nullptr) {
        delete railTraverseProfile;
    }
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisRailBounceCommand::isReady() {
    return true;
}

bool ChassisRailBounceCommand::isFinished() const {
    return false;
}

}  // namespace src::Chassis