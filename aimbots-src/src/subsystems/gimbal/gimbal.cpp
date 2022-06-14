#include "gimbal.hpp"

static inline float wrappedEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

static inline int64_t radiansToEncoder(float angle) {
    return static_cast<int64_t>(angle * (DJIMotor::ENC_RESOLUTION / M_TWOPI));
}

static inline void updateUnwrappedAngleMeasure(
    int16_t startEncoderValue,
    float startAngle,
    int64_t unwrappedEncoder,
    int16_t& startEncoderOffset,
    int64_t& lastEncoder,
    float& unwrappedAngleMeasure)
{
    if (startEncoderOffset == INT16_MIN) {
        int encDiff = static_cast<int>(radiansToEncoder(modm::toRadian(YAW_START_ANGLE))) - static_cast<int>(unwrappedEncoder);

        if (encDiff < -static_cast<int>(DJIMotor::ENC_RESOLUTION / 2)) {
            startEncoderOffset = -DJIMotor::ENC_RESOLUTION;
        } else if (encDiff > DJIMotor::ENC_RESOLUTION / 2) {
            startEncoderOffset = DJIMotor::ENC_RESOLUTION;
        } else {
            startEncoderOffset = 0;
        }
    }

    if (lastEncoder == unwrappedEncoder) return;

    lastEncoder = unwrappedEncoder;
    unwrappedAngleMeasure = 
        static_cast<float>(unwrappedEncoder
                         - startEncoderValue
                         + startEncoderOffset) *
        M_TWOPI / static_cast<float>(DJIMotor::ENC_RESOLUTION) +
        modm::toRadian(startAngle);
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

void GimbalSubsystem::refresh() {
    int64_t unwrappedYawEncoder = INT64_MIN;
    int64_t unwrappedPitchEncoder = INT64_MIN;

    if (yawMotor.isMotorOnline()) {
        unwrappedYawEncoder = yawMotor.getEncoderUnwrapped();

        // Update subsystem state to stay up-to-date with reality
        uint16_t currentYawEncoderPosition = yawMotor.getEncoderWrapped();
        currentYawAngle.setValue(wrappedEncoderValueToRadians(currentYawEncoderPosition));

        // Flush whatever our current output is to the motors
        yawMotor.setDesiredOutput(desiredYawMotorOutput);
    }

    if (pitchMotor.isMotorOnline()) {
        unwrappedPitchEncoder = pitchMotor.getEncoderUnwrapped();

        // Update subsystem state to stay up-to-date with reality
        uint16_t currentPitchEncoderPosition = pitchMotor.getEncoderWrapped();
        currentPitchAngle.setValue(wrappedEncoderValueToRadians(currentPitchEncoderPosition));

        // Flush whatever our current output is to the motors
        pitchMotor.setDesiredOutput(desiredPitchMotorOutput);
    }

    updateUnwrappedAngleMeasure(
        YAW_START_ENCODER,
        modm::toRadian(YAW_START_ANGLE),
        unwrappedYawEncoder,
        startYawEncoderOffset,
        lastUpdatedYawEncoderValue,
        unwrappedYawAngleMeasurement);
}

void GimbalSubsystem::setYawMotorOutput(float output) {
    // This is limited since the datatype of the parameter to the function
    // `DJIMotor::setDesiredOutput()` is an int16_t. So, to make sure that
    // it isn't overflowed, we limit it within +/- 30,000, which is just
    // the max and min of an int16_t (int16_t has a range of about +/- 32,000).
    desiredYawMotorOutput = tap::algorithms::limitVal(output, -30000.0f, 30000.0f);
}

void GimbalSubsystem::setPitchMotorOutput(float output) {
    // This is limited since the datatype of the parameter to the function
    // `DJIMotor::setDesiredOutput()` is an int16_t. So, to make sure that
    // it isn't overflowed, we limit it within +/- 30,000, which is just
    // the max and min of an int16_t (int16_t has a range of about +/- 32,000).
    desiredPitchMotorOutput = tap::algorithms::limitVal(output, -30000.0f, 30000.0f);
}

float GimbalSubsystem::getCurrentYawAngleFromCenter(AngleUnit unit) const {
    return tap::algorithms::ContiguousFloat(
               (unit == AngleUnit::Degrees) ? modm::toDegree(currentYawAngle.getValue() - YAW_START_ANGLE) : currentYawAngle.getValue() - YAW_START_ANGLE,
               (unit == AngleUnit::Degrees) ? -180.0f : -M_PI,
               (unit == AngleUnit::Degrees) ? 180.0f : M_PI)
        .getValue();
}

float GimbalSubsystem::getCurrentPitchAngleFromCenter(AngleUnit unit) const {
    return tap::algorithms::ContiguousFloat(
               (unit == AngleUnit::Degrees) ? modm::toDegree(currentPitchAngle.getValue() - PITCH_START_ANGLE) : currentPitchAngle.getValue() - PITCH_START_ANGLE,
               (unit == AngleUnit::Degrees) ? -180.0f : -M_PI,
               (unit == AngleUnit::Degrees) ? 180.0f : M_PI)
        .getValue();
}

}  // namespace src::Gimbal
