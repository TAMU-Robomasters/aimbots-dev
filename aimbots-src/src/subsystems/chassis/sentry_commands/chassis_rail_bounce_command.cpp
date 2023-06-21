#include "chassis_rail_bounce_command.hpp"
#ifdef ULTRASONIC

namespace src::Chassis {

ChassisRailBounceCommand::ChassisRailBounceCommand(src::Drivers* drivers, ChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis),
      profileConstraints(
          {1.0f,     // max velocity
           2.0f,     // max acceleration
           10.0f}),  // max jerk
      railTargetIndex(1),
      railTraverseProfile(nullptr) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));

    static constexpr float railTargetsArray[2] = {leftRailBound, rightRailBound};
    railTargets = Matrix<float, 2, 1>(railTargetsArray);
}

float leftRailBoundDisplay = 0.0f;
float rightRailBoundDisplay = 0.0f;

float displacementTargetDisplay = 0.0f;

void ChassisRailBounceCommand::initialize() {
    leftRailBoundDisplay = leftRailBound;
    rightRailBoundDisplay = rightRailBound;

    float currRailPosition = drivers->fieldRelativeInformant.getRailRelativeRobotPosition()[0][X];

    float displacementTarget = railTargets[railTargetIndex][X] - currRailPosition;

    if (displacementTarget < 0.0f) {
        profilerDirection = -1;
    } else {
        profilerDirection = 1;
    }

    displacementTargetDisplay = displacementTarget * profilerDirection;

    if (railTraverseProfile != nullptr) {
        delete railTraverseProfile;
    }
    railTraverseProfile = new SCurveMotionProfile(profileConstraints, fabs(displacementTarget));

    movementStartTime = tap::arch::clock::getTimeMilliseconds();
}

float currRailPositionDisplay = 0.0f;
float profileVelocityDisplay = 0.0f;
float expectedMovementTimeDisplay = 0.0f;
float motorTargetRPMDisplay = 0.0f;
float railTargetDisplay = 0.0f;
int railTargetIndexDisplay = 0;
float profileDisplacementDisplay = 0.0f;
bool exitConditionDisplay = false;
float movementStartTimeDisplay = 0.0f;
float currTimeDisplay = 0.0f;

void ChassisRailBounceCommand::execute() {
    float currTime = tap::arch::clock::getTimeMilliseconds();
    float currRailPosition = drivers->fieldRelativeInformant.getRailRelativeRobotPosition()[0][X];
    currRailPositionDisplay = currRailPosition;
    railTargetDisplay = railTargets[railTargetIndex][X];
    railTargetIndexDisplay = railTargetIndex;

    exitConditionDisplay = currTime > (railTraverseProfile->totalTime() * 1000) + movementStartTime + 1000;

    expectedMovementTimeDisplay = railTraverseProfile->totalTime();
    movementStartTimeDisplay = movementStartTime;
    currTimeDisplay = currTime;

    if (currTime > (railTraverseProfile->totalTime() * 1000) + movementStartTime + patrolDelay) {
        // if (chassisProfile.isSettled(railTargets[railTargetIndex][X] - currRailPosition, 0.03f)) {

        // increment rail target index and wrap to 0 when it goes past 1
        railTargetIndex = (railTargetIndex + 1) % railTargets.getNumberOfRows();

        // float displacementTarget = railTargets[railTargetIndex][X] - railTargets[(railTargetIndex + 1) % 2][X];
        float displacementTarget = railTargets[railTargetIndex][X] - currRailPosition;

        if (displacementTarget < 0.0f) {
            profilerDirection = -1;
        } else {
            profilerDirection = 1;
        }

        if (railTraverseProfile != nullptr) {
            delete railTraverseProfile;
        }
        railTraverseProfile = new SCurveMotionProfile(profileConstraints, fabs(displacementTarget));
        profileDisplacementDisplay = displacementTarget * profilerDirection;

        movementStartTime = currTime;
    }

    auto step = railTraverseProfile->stepAtTime((currTime - movementStartTime) / 1000.0f);
    profileVelocityDisplay = step.velocity * profilerDirection;

    float wheelTargetRPM = ((step.velocity * profilerDirection) / (2.0f * M_PI * WHEEL_RADIUS)) * 60.0f;
    float motorTargetRPM = -(wheelTargetRPM / CHASSIS_GEARBOX_RATIO);
    motorTargetRPMDisplay = motorTargetRPM;

    chassis->setTargetRPMs(motorTargetRPM, 0.0f, 0.0f);
}

void ChassisRailBounceCommand::end(bool) {
    if (railTraverseProfile != nullptr) {
        delete railTraverseProfile;
    }
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisRailBounceCommand::isReady() { return true; }

bool ChassisRailBounceCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif // ULTRASONIC