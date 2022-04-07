#include "gimbal.hpp"

static inline float wrappedEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

namespace src::Gimbal {

GimbalSubsystem::GimbalSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      yawMotor(drivers,
               YAW_MOTOR_ID,
               GIMBAL_BUS,
               false,
               "Yaw Motor"),
      pitchMotor(drivers,
                 PITCH_MOTOR_ID,
                 GIMBAL_BUS,
                 false,
                 "Pitch Motor"),
      currentYawAngle(0.0f, 0.0f, M_TWOPI),
      currentPitchAngle(0.0f, 0.0f, M_TWOPI),
      targetYawAngle(modm::toRadian(YAW_START_ANGLE)),
      targetPitchAngle(modm::toRadian(PITCH_START_ANGLE)) {}

void GimbalSubsystem::initialize() {
    yawMotor.initialize();
    yawMotor.setDesiredOutput(0);

    pitchMotor.initialize();
    pitchMotor.setDesiredOutput(0);
}

static float currentPitchAngleDebug = 0.0f;
static float currentYawAngleDebug = 0.0f;

void GimbalSubsystem::refresh() {
    if (yawMotor.isMotorOnline()) {
        uint16_t currentYawEncoderPosition = yawMotor.getEncoderWrapped();
        currentYawAngle.setValue(wrappedEncoderValueToRadians(currentYawEncoderPosition));

        currentYawAngleDebug = getCurrentYawAngle(AngleUnit::Degrees);

        // flush whatever our current output is to the motors
        yawMotor.setDesiredOutput(desiredYawMotorOutput);
    }
    currentYawAngleDebug = getCurrentYawAngle(AngleUnit::Degrees);

    if (pitchMotor.isMotorOnline()) {
        uint16_t currentPitchEncoderPosition = pitchMotor.getEncoderWrapped();
        currentPitchAngle.setValue(wrappedEncoderValueToRadians(currentPitchEncoderPosition));

        currentPitchAngleDebug = getCurrentPitchAngle(AngleUnit::Degrees);

        // flush whatever our current output is to the motors
        pitchMotor.setDesiredOutput(desiredPitchMotorOutput);
    }
}

void GimbalSubsystem::setYawMotorOutput(float output) {
    desiredYawMotorOutput = tap::algorithms::limitVal(output, -30000.0f, 30000.0f);
}

void GimbalSubsystem::setPitchMotorOutput(float output) {
    desiredPitchMotorOutput = tap::algorithms::limitVal(output, -30000.0f, 30000.0f);
}

float GimbalSubsystem::getCurrentYawAngleFromCenter(AngleUnit unit) const {
    return tap::algorithms::ContiguousFloat(
               (unit == AngleUnit::Degrees) ? modm::toDegree(currentYawAngle.getValue() - YAW_START_ANGLE) : currentYawAngle.getValue() - YAW_START_ANGLE,
               (unit == AngleUnit::Degrees) ? -180.0f : M_PI,
               (unit == AngleUnit::Degrees) ? 180.0f : M_PI)
        .getValue();
}

float GimbalSubsystem::getCurrentPitchAngleFromCenter(AngleUnit unit) const {
    return tap::algorithms::ContiguousFloat(
               (unit == AngleUnit::Degrees) ? modm::toDegree(currentPitchAngle.getValue() - PITCH_START_ANGLE) : currentPitchAngle.getValue() - PITCH_START_ANGLE,
               (unit == AngleUnit::Degrees) ? -180.0f : M_PI,
               (unit == AngleUnit::Degrees) ? 180.0f : M_PI)
        .getValue();
}

}  // namespace src::Gimbal
