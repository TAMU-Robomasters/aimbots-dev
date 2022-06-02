#include "chassis_rail_bounce_command.hpp"

namespace src::Chassis {

    ChassisRailBounceCommand::ChassisRailBounceCommand(src::Drivers * drivers, ChassisSubsystem * chassis)
        : drivers(drivers),
          chassis(chassis)
#ifdef TARGET_SENTRY
          ,
          profileConstraints(
              {0.5f,     // max velocity
               1.0f,     // max acceleration
               10.0f}),  // max jerk
          leftRailBound(leftRailBoundArray),
          rightRailBound(rightRailBoundArray),
          currTraverseTarget(0.0f),
          railTraverseProfile(nullptr)
#endif
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
    }

    void ChassisRailBounceCommand::initialize() {
        movementStartTime = tap::arch::clock::getTimeMilliseconds();

        currTraverseTarget = rightRailBound[0][0];

        float currPosition = /*drivers->fieldRelativeInformant.getRailRelativeRobotPosition()[0][X];*/ 0.0f;
        float displacement = currTraverseTarget - currPosition;

        if (railTraverseProfile != nullptr) {
            delete railTraverseProfile;
        }
        railTraverseProfile = new SCurveMotionProfile(profileConstraints, displacement);
    }

    float currRailPositionDisplay = 0.0f;
    float stepVelocityDisplay = 0.0f;
    float movementTimeDisplay = 0.0f;
    float movementTotalTimeReportedDisplay = 0.0f;
    float currTraverseTargetDisplay = 0.0f;
    float motorTargetRPMDisplay = 0.0f;

    void ChassisRailBounceCommand::execute() {
        float currTime = tap::arch::clock::getTimeMilliseconds();
        float currPosition = drivers->fieldRelativeInformant.getRailRelativeRobotPosition()[0][X];
        currRailPositionDisplay = currPosition;

        float movementTime = currTime - movementStartTime;
        auto step = railTraverseProfile->stepAtTime(movementTime / 1000.0f);

        stepVelocityDisplay = step.velocity;

        float wheelTargetRPM = (step.velocity / (2.0f * M_PI * WHEEL_RADIUS)) * 60.0f;
        float motorTargetRPM = wheelTargetRPM / CHASSIS_GEARBOX_RATIO;
        motorTargetRPMDisplay = motorTargetRPM;

        chassis->setTargetRPMs(motorTargetRPM, 0.0f, 0.0f);
    }

    void ChassisRailBounceCommand::end(bool) {
        chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
    }

    bool ChassisRailBounceCommand::isReady() {
        return true;
    }

    bool ChassisRailBounceCommand::isFinished() const {
        return false;
    }

}  // namespace src::Chassis