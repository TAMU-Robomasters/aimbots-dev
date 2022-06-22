#include "gimbal.hpp"

static inline float wrappedEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

namespace src::Gimbal {

GimbalSubsystem::GimbalSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      yawMotor(drivers,
               YAW_MOTOR_ID,
               GIMBAL_BUS,
               YAW_DIRECTION,
               "Yaw Motor"),
      pitchMotor(drivers,
                 PITCH_MOTOR_ID,
                 GIMBAL_BUS,
                 PITCH_DIRECTION,
                 "Pitch Motor"),
      currentFieldRelativeYawAngle(0.0f, 0.0f, M_TWOPI),
      currentChassisRelativeYawAngle(0.0f, 0.0f, M_TWOPI),
      currentChassisRelativePitchAngle(0.0f, 0.0f, M_TWOPI),
      targetChassisRelativeYawAngle(modm::toRadian(YAW_START_ANGLE)),
      targetChassisRelativePitchAngle(modm::toRadian(PITCH_START_ANGLE)) {}

void GimbalSubsystem::initialize() {
    yawMotor.initialize();
    yawMotor.setDesiredOutput(0);

    pitchMotor.initialize();
    pitchMotor.setDesiredOutput(0);
}

float yawChassisRelativeDisplay = 0.0f;
float yawFieldRelativeDisplay = 0.0f;
float pitchChassisRelativeDisplay = 0.0f;

float yawOutputDisplay = 0.0f;

// This is ugly, but I'm just doing this for simplicity
#ifdef TARGET_HERO
static bool isStartYawSet = false;
static int64_t heroStartYawUnwrappedEncoder = 0;
#endif

void GimbalSubsystem::refresh() {
    if (yawMotor.isMotorOnline()) {
        // Update subsystem state to stay up-to-date with reality
        uint16_t currentYawEncoderPosition = yawMotor.getEncoderWrapped();
        currentChassisRelativeYawAngle.setValue(wrappedEncoderValueToRadians(currentYawEncoderPosition));

#ifdef TARGET_HERO
        // This code just assumes that we're starting at our
        // YAW_START_ANGLE when the robot gets turned on, and
        // then we just apply the delta from the starting encoder
        // to that after we apply the gear ratio transformation.

        int64_t unwrappedEncoder = yawMotor.getEncoderUnwrapped();

        if (!isStartYawSet) {
            isStartYawSet = true;
            heroStartYawUnwrappedEncoder = unwrappedEncoder;
        }

        float rawDelta = unwrappedEncoder - heroStartYawUnwrappedEncoder;
        float transformedDelta = rawDelta * GIMBAL_YAW_GEAR_RATIO;
        float angle = modm::toRadian(YAW_START_ANGLE) + (transformedDelta * (M_TWOPI / DJIMotor::ENC_RESOLUTION));
        currentChassisRelativeYawAngle.setValue(angle);
#endif

        // FIXME: Verify that these plus and minus signs work out...
        currentFieldRelativeYawAngle.setValue(currentChassisRelativeYawAngle.getValue() + drivers->fieldRelativeInformant.getChassisYaw() - modm::toRadian(YAW_START_ANGLE));

        // Flush whatever our current output is to the motors
        yawMotor.setDesiredOutput(desiredYawMotorOutput);
        
        ////////////////
        // DEBUG VARS //
        ////////////////
        yawChassisRelativeDisplay = modm::toDegree(currentChassisRelativeYawAngle.getValue());
        yawFieldRelativeDisplay = modm::toDegree(currentFieldRelativeYawAngle.getValue());
        yawOutputDisplay = desiredYawMotorOutput;
    }

    if (pitchMotor.isMotorOnline()) {
        // Update subsystem state to stay up-to-date with reality
        uint16_t currentPitchEncoderPosition = pitchMotor.getEncoderWrapped();
        currentChassisRelativePitchAngle.setValue(wrappedEncoderValueToRadians(currentPitchEncoderPosition));

        pitchChassisRelativeDisplay = modm::toDegree(currentChassisRelativePitchAngle.getValue());

        // Flush whatever our current output is to the motors
        pitchMotor.setDesiredOutput(desiredPitchMotorOutput);
    }
}

void GimbalSubsystem::setYawMotorOutput(float output) {
    // This is limited since the datatype of the parameter to the function
    // `DJIMotor::setDesiredOutput()` is an int16_t. So, to make sure that
    // it isn't overflowed, we limit it within +/- 30,000, which is just
    // the max and min of an int16_t (int16_t has a range of about +/- 32,000).
    desiredYawMotorOutput = tap::algorithms::limitVal(output, -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT);
}

void GimbalSubsystem::setPitchMotorOutput(float output) {
    // This is limited since the datatype of the parameter to the function
    // `DJIMotor::setDesiredOutput()` is an int16_t. So, to make sure that
    // it isn't overflowed, we limit it within +/- 30,000, which is just
    // the max and min of an int16_t (int16_t has a range of about +/- 32,000).
    desiredPitchMotorOutput = tap::algorithms::limitVal(output, -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT);
}

float GimbalSubsystem::getCurrentYawAngleFromChassisCenter(AngleUnit unit) const {
    return tap::algorithms::ContiguousFloat(
               (unit == AngleUnit::Degrees) ? modm::toDegree(currentChassisRelativeYawAngle.getValue() - modm::toRadian(YAW_START_ANGLE)) : (currentChassisRelativeYawAngle.getValue() - modm::toRadian(YAW_START_ANGLE)),
               (unit == AngleUnit::Degrees) ? -180.0f : -M_PI,
               (unit == AngleUnit::Degrees) ? 180.0f : M_PI)
        .getValue();
}

float GimbalSubsystem::getCurrentPitchAngleFromChassisCenter(AngleUnit unit) const {
    return tap::algorithms::ContiguousFloat(
               (unit == AngleUnit::Degrees) ? modm::toDegree(currentChassisRelativePitchAngle.getValue() - modm::toRadian(PITCH_START_ANGLE)) : (currentChassisRelativePitchAngle.getValue() - modm::toRadian(PITCH_START_ANGLE)),
               (unit == AngleUnit::Degrees) ? -180.0f : -M_PI,
               (unit == AngleUnit::Degrees) ? 180.0f : M_PI)
        .getValue();
}

}  // namespace src::Gimbal
