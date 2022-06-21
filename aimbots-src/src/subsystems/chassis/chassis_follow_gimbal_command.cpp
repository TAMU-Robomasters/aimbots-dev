#include "utils/robot_specific_inc.hpp"
#ifdef TOKYO_COMPATIBLE

#include <subsystems/chassis/chassis_rel_drive.hpp>

#include "chassis_follow_gimbal_command.hpp"

#define RADPS_TO_RPM 9.549297f

namespace src::Chassis {

ChassisFollowGimbalCommand::ChassisFollowGimbalCommand(src::Drivers* drivers, ChassisSubsystem* chassis, src::Gimbal::GimbalSubsystem* gimbal)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
      rotationController(ROTATION_POSITION_PID_CONFIG)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisFollowGimbalCommand::initialize() {
}

float gimbalYawFieldRelativeDisplay = 0.0f;
float gimbalYawAngleDisplay2 = 0.0f;
float chassisYawDisplay = 0.0f;
float rotationControllerOutputDisplay = 0.0f;

void ChassisFollowGimbalCommand::execute() {
    if (gimbal->isOnline()) {
        float gimbalYawAngle = gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Radians);

        float x = 0.0f;
        float y = 0.0f;

        // float rotationControllerError = drivers->fieldRelativeInformant.getChassisYaw() - gimbal->getCurrentFieldRelativeYawAngle(AngleUnit::Radians);
        float rotationControllerError = gimbalYawAngle;

        // chassisYawDisplay = modm::toDegree(drivers->fieldRelativeInformant.getChassisYaw());
        // gimbalYawFieldRelativeDisplay = gimbal->getCurrentFieldRelativeYawAngle(AngleUnit::Degrees);

        // if (fabsf(rotationControllerError) < FOLLOW_GIMBAL_ANGLE_THRESHOLD) {
        //     rotationControllerError = 0.0f;
        // }

        rotationController.runController(rotationControllerError, (RADPS_TO_RPM * drivers->fieldRelativeInformant.getGz()));

        rotationControllerOutputDisplay = rotationController.getOutput();

        Movement::Independent::calculateUserDesiredMovement(drivers, chassis, &x, &y, rotationController.getOutput());

        // x *= TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER;
        // y *= TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER;

        const float maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        // const float translationalSpeedThreshold = maxWheelSpeed * TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER * TOKYO_TRANSLATION_THRESHOLD_TO_DECREASE_ROTATION_SPEED;

        // float rampTarget = maxWheelSpeed * rotationDirection * TOKYO_ROTATIONAL_SPEED_FRACTION_OF_MAX;

        // reduces rotation speed when translation speed is high
        // if (fabsf(x) > translationalSpeedThreshold || fabsf(y) > translationalSpeedThreshold) {
        //     rampTarget *= TOKYO_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING;
        // }

        // rotationSpeedRamp.setTarget(rampTarget);
        // rotationSpeedRamp.update(TOKYO_ROTATIONAL_SPEED_INCREMENT);

        // float r = rotationSpeedRamp.getValue();

        gimbalYawAngleDisplay2 = modm::toDegree(gimbalYawAngle);

        rotateVector(&x, &y, -gimbalYawAngle);

        chassis->setTargetRPMs(x, y, rotationController.getOutput());
    } else {
        Movement::Independent::onExecute(drivers, chassis);
    }
}

void ChassisFollowGimbalCommand::end(bool) {
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisFollowGimbalCommand::isReady() {
    return true;
}

bool ChassisFollowGimbalCommand::isFinished() const {
    return false;
}

}  // namespace src::Chassis

#endif