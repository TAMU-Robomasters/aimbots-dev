
#include "chassis_ignore_gimbal_command.hpp"

#ifdef GIMBAL_UNTETHERED
#ifdef CHASSIS_COMPATIBLE

#include "subsystems/chassis/control/chassis_helper.hpp"

namespace src::Chassis {

ChassisIgnoreGimbalCommand::ChassisIgnoreGimbalCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    src::Gimbal::GimbalSubsystem* gimbal)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
      rotationController(ROTATION_POSITION_PID_CONFIG)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisIgnoreGimbalCommand::initialize() {}

// Debug variables
// float yawAngleFromChassisCenterDisplay2 = 0.0f;
// float chassisYawDisplay = 0.0f;
// float rotationControllerOutputDisplay = 0.0f;
// float rotationLimitedMaxTranslationalSpeedDisplay = 0.0f;

// bool gimbalOnlineDisplay = false;
// float chassisErrorAngleDisplay = 0.0f;

void ChassisIgnoreGimbalCommand::execute() {
    chassis->setTokyoDrift(false);
    float desiredX = 0.0f;
    float desiredY = 0.0f;
    float desiredRotation = 0.0f;
    // gets desired user input from operator interface
    Chassis::Helper::getUserDesiredInput(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

    //gimbalOnlineDisplay = gimbal->isOnline();

    if (gimbal->isOnline()) {
        float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);

        float chassisErrorAngle = yawAngleFromChassisCenter;

        //chassisErrorAngleDisplay = chassisErrorAngle;

        // Find rotation correction power
        rotationController.runController(
            chassisErrorAngle,
            -RADPS_TO_RPM(drivers->kinematicInformant.getChassisIMUAngularVelocity(
                src::Informants::AngularAxis::YAW_AXIS,
                AngleUnit::Radians)));
        // rotationController.runControllerDerivateError(chassisErrorAngle);
        //rotationControllerOutputDisplay = rotationController.getOutput();

        // overwrite desired rotation with rotation controller output, range [-1, 1]
        desiredRotation = rotationController.getOutput();

        Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

        tap::algorithms::rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);
    } else {  // if the gimbal is offline, run the normal manual drive command
        Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
    }

    //chassis does not care about rotation
    chassis->setTargetRPMs(desiredX, desiredY, 0.0f);
}

void ChassisIgnoreGimbalCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisIgnoreGimbalCommand::isReady() { return true; }

bool ChassisIgnoreGimbalCommand::isFinished() const { return false; }

}  // namespace src::Chassis
#endif  //#ifdef CHASSIS_COMPATIBLE

#endif  //#ifdef GIMBAL_UNTETHERED