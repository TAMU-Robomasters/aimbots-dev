#include "chassis_ignore_gimbal_command.hpp"

#ifdef GIMBAL_UNTETHERED
#ifdef CHASSIS_COMPATIBLE

#include <cmath>

#include "subsystems/chassis/control/chassis_helper.hpp"

namespace src::Chassis {

static constexpr float CUSTOM_CONTROLLER_TRANSLATION_DEADBAND = 0.05f;

static inline float applyCustomControllerDeadband(float value) {
    return std::fabs(value) < CUSTOM_CONTROLLER_TRANSLATION_DEADBAND ? 0.0f : value;
}

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

void ChassisIgnoreGimbalCommand::setMaxWheelSpeed(float maxWheelSpeed) {
    this->maxWheelSpeed = maxWheelSpeed > 0.0f ? maxWheelSpeed : DEFAULT_CUSTOM_CONTROLLER_MAX_WHEEL_SPEED;
}

// Debug/watch variables for ozone
float ignoreGimbalDesiredXDisplay = 0.0f;
float ignoreGimbalDesiredYDisplay = 0.0f;
float ignoreGimbalCustomXDisplay = 0.0f;
float ignoreGimbalCustomYDisplay = 0.0f;
float ignoreGimbalMaxWheelSpeedDisplay = 0.0f;
bool ignoreGimbalButton4Display = false;

void ChassisIgnoreGimbalCommand::execute() {
    chassis->setTokyoDrift(false);

    float desiredX = 0.0f;
    float desiredY = 0.0f;
    float desiredRotation = 0.0f;

    // Conventional RC / keyboard input path.
    Chassis::Helper::getUserDesiredInput(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

    // Add custom controller joystick 1 translation on top of the conventional inputs.
    const bool customControllerConnected = drivers->controlOperatorInterface.isCustomControllerConnected();
    const float customX = customControllerConnected
                              ? applyCustomControllerDeadband(drivers->controlOperatorInterface.getCustomControllerChassisXInput())
                              : 0.0f;
    const float customY = customControllerConnected
                              ? applyCustomControllerDeadband(drivers->controlOperatorInterface.getCustomControllerChassisYInput())
                              : 0.0f;

    desiredX = limitVal<float>(desiredX + customX, -1.0f, 1.0f);
    desiredY = limitVal<float>(desiredY + customY, -1.0f, 1.0f);

    // Button 4 is still displayed here for debugging, but the RPM ceiling itself is now fed by
    // ChassisToggleDriveCustomControllerCommand after power limiting is applied.
    const bool button4Held = customControllerConnected && drivers->controlOperatorInterface.customControllerButton4Pressed();

    if (gimbal->isOnline()) {
        float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        float chassisErrorAngle = yawAngleFromChassisCenter;

        // Find rotation correction power. This is preserved from the original command even though the final
        // chassis rotation command is intentionally zeroed below.
        rotationController.runController(
            chassisErrorAngle,
            -RADPS_TO_RPM(drivers->kinematicInformant.getChassisIMUAngularVelocity(
                src::Informants::AngularAxis::YAW_AXIS,
                AngleUnit::Radians)));

        desiredRotation = rotationController.getOutput();

        Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(
            drivers,
            chassis,
            &desiredX,
            &desiredY,
            &desiredRotation,
            maxWheelSpeed);

        tap::algorithms::rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);
    } else {
        Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(
            drivers,
            chassis,
            &desiredX,
            &desiredY,
            &desiredRotation,
            maxWheelSpeed);
    }

    ignoreGimbalDesiredXDisplay = desiredX;
    ignoreGimbalDesiredYDisplay = desiredY;
    ignoreGimbalCustomXDisplay = customX;
    ignoreGimbalCustomYDisplay = customY;
    ignoreGimbalMaxWheelSpeedDisplay = maxWheelSpeed;
    ignoreGimbalButton4Display = button4Held;

    // Chassis ignores rotation in this command.
    chassis->setTargetRPMs(desiredX, desiredY, 0.0f, maxWheelSpeed);
}

void ChassisIgnoreGimbalCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisIgnoreGimbalCommand::isReady() { return true; }

bool ChassisIgnoreGimbalCommand::isFinished() const { return false; }

}  // namespace src::Chassis
#endif  // #ifdef CHASSIS_COMPATIBLE

#endif  // #ifdef GIMBAL_UNTETHERED