#include "utils/robot_specific_inc.hpp"
#ifdef GIMBAL_UNTETHERED
#ifdef CHASSIS_COMPATIBLE

#include "subsystems/chassis/chassis_helper.hpp"

#include "chassis_follow_gimbal_command.hpp"

namespace src::Chassis {

ChassisFollowGimbalCommand::ChassisFollowGimbalCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    src::Gimbal::GimbalSubsystem* gimbal,
    const SnapSymmetryConfig& snapSymmetryConfig)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
      snapSymmetryConfig(snapSymmetryConfig),
      rotationController(ROTATION_POSITION_PID_CONFIG)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisFollowGimbalCommand::initialize() {
    
}

// Debug variables
float yawAngleFromChassisCenterDisplay2 = 0.0f;
float chassisYawDisplay = 0.0f;
float rotationControllerOutputDisplay = 0.0f;
float rotationLimitedMaxTranslationalSpeedDisplay = 0.0f;

bool gimbalOnlineDisplay = false;
float chassisErrorAngleDisplay = 0.0f;

void ChassisFollowGimbalCommand::execute() {
    chassis->setTokyoDrift(false);
    float desiredX = 0.0f;
    float desiredY = 0.0f;
    float desiredRotation = 0.0f;
    // gets desired user input from operator interface
    Chassis::Helper::getUserDesiredInput(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

    gimbalOnlineDisplay = gimbal->isOnline();

    if (gimbal->isOnline()) {  // if the gimbal is online, follow the gimbal's yaw
        float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);

        float chassisErrorAngle = Helper::findNearestChassisErrorTo(yawAngleFromChassisCenter, snapSymmetryConfig);
        yawAngleFromChassisCenter -= modm::toRadian(90);
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
        if (abs(chassisErrorAngle) > modm::toRadian(2.0f)) {
            desiredRotation = rotationController.getOutput();
        } else {
        desiredRotation = 0.0f;
        }

        Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

        tap::algorithms::rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);
    } else {  // if the gimbal is offline, run the normal manual drive command
        Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
    }

    chassis->setTargetRPMs(desiredX, desiredY, desiredRotation);
}

void ChassisFollowGimbalCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisFollowGimbalCommand::isReady() { return true; }

bool ChassisFollowGimbalCommand::isFinished() const { return false; }

}  // namespace src::Chassis
#endif //#ifdef CHASSIS_COMPATIBLE

#endif //#ifdef GIMBAL_UNTETHERED