#include "utils/robot_specific_inc.hpp"
#ifdef GIMBAL_UNTETHERED

#include "subsystems/chassis/chassis_helper.hpp"

#include "chassis_follow_gimbal_command.hpp"

namespace src::Chassis {

ChassisFollowGimbalCommand::ChassisFollowGimbalCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    src::Gimbal::GimbalSubsystem* gimbal,
    uint8_t numSnapPositions,
    float starterAngle)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
      numSnapPositions(numSnapPositions),
      starterAngle(starterAngle),
      rotationController(ROTATION_POSITION_PID_CONFIG)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisFollowGimbalCommand::initialize() {}

// Debug variables
float yawAngleFromChassisCenterDisplay2 = 0.0f;
float chassisYawDisplay = 0.0f;
float rotationControllerOutputDisplay = 0.0f;
float rotationLimitedMaxTranslationalSpeedDisplay = 0.0f;

bool isChassisScheduled = false;
bool isChassisRunning = false;

bool gimbalOnlineDisplay = false;

float chassisErrorAngleDisplay = 0.0f;

void ChassisFollowGimbalCommand::execute() {
    float desiredX = 0.0f;
    float desiredY = 0.0f;
    float desiredRotation = 0.0f;
    isChassisScheduled = true;
    // gets desired user input from operator interface
    Chassis::Helper::getUserDesiredInput(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

    gimbalOnlineDisplay = gimbal->isOnline();

    if (gimbal->isOnline()) {  // if the gimbal is online, follow the gimbal's yaw
        float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);

        float chassisErrorAngle =
            Helper::findNearestChassisErrorTo(yawAngleFromChassisCenter, numSnapPositions, starterAngle);
        // float chassisErrorAngle = yawAngleFromChassisCenter;

        chassisErrorAngleDisplay = chassisErrorAngle;

        // Find rotation correction power
        rotationController.runController(
            chassisErrorAngle,
            -RADPS_TO_RPM(drivers->kinematicInformant.getChassisIMUAngularVelocity(
                src::Informants::AngularAxis::YAW_AXIS,
                AngleUnit::Radians)));
        // rotationController.runControllerDerivateError(chassisErrorAngle);
        rotationControllerOutputDisplay = rotationController.getOutput();

        // overwrite desired rotation with rotation controller output, range [-1, 1]
        desiredRotation = rotationController.getOutput();

        Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

        tap::algorithms::rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);
    } else {  // if the gimbal is offline, run the normal manual drive command
        Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
    }

    isChassisRunning = true;

    chassis->setTargetRPMs(desiredX, desiredY, desiredRotation);
}

void ChassisFollowGimbalCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
    isChassisScheduled = false;
    isChassisRunning = false;
}

bool ChassisFollowGimbalCommand::isReady() { return true; }

bool ChassisFollowGimbalCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif