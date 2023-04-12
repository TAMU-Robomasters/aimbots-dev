#include "gimbal.hpp"

#include <drivers.hpp>
#include <utils/common_types.hpp>

static inline float DJIEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

namespace src::Gimbal {

GimbalSubsystem::GimbalSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      yawMotor(drivers, YAW_MOTOR_ID, GIMBAL_BUS, YAW_DIRECTION, "Yaw Motor"),
      pitchMotor(drivers, PITCH_MOTOR_ID, GIMBAL_BUS, PITCH_DIRECTION, "Pitch Motor"),
      currentYawAngle(0.0f, 0.0f, M_TWOPI),
      currentPitchAngle(0.0f, 0.0f, M_TWOPI),
      targetYawAngle(0.0f, 0.0f, M_TWOPI),
      targetPitchAngle(0.0f, 0.0f, M_TWOPI) {}

void GimbalSubsystem::initialize() {
    yawMotor.initialize();
    pitchMotor.initialize();

    yawMotor.setDesiredOutput(0);
    pitchMotor.setDesiredOutput(0);
}

float yawMotorAngleDisplay = 0.0f;
float yawFieldRelativeDisplay = 0.0f;
float pitchMotorAngleDisplay = 0.0f;

float pitchOutputDisplay = 0.0f;
float yawOutputDisplay = 0.0f;

// This is ugly, but I'm just doing this for simplicity
#ifdef TARGET_HERO
static bool isStartYawSet = false;
static int64_t heroStartYawUnwrappedEncoder = 0;
#endif

float currentYawAngleDisplay = 0.0f;
float currentPitchAngleDisplay = 0.0f;

void GimbalSubsystem::refresh() {
    if (yawMotor.isMotorOnline()) {
        // Update subsystem state to stay up-to-date with reality

        uint16_t currentYawEncoderPosition = yawMotor.getEncoderUnwrapped();

        float unwrappedYawAngle =
            ((DJIEncoderValueToRadians(currentYawEncoderPosition) - YAW_OFFSET_ANGLE) * GIMBAL_YAW_GEAR_RATIO);

        currentYawAngle.setValue(unwrappedYawAngle);

#ifdef TARGET_HERO
        // This code just assumes that we're starting at our
        // YAW_OFFSET_ANGLE when the robot gets turned on, and
        // then we just apply the delta from the starting encoder
        // to that after we apply the gear ratio transformation.

        int64_t unwrappedEncoder = yawMotor.getEncoderUnwrapped();

        if (!isStartYawSet) {
            isStartYawSet = true;
            heroStartYawUnwrappedEncoder = unwrappedEncoder;
        }

        float rawDelta = unwrappedEncoder - heroStartYawUnwrappedEncoder;
        float transformedDelta = rawDelta * GIMBAL_YAW_GEAR_RATIO;
        float angle = modm::toRadian(YAW_OFFSET_ANGLE) + (transformedDelta * (M_TWOPI / DJIMotor::ENC_RESOLUTION));
        currentYawAngle.setValue(angle);

        float startEncoderDelta = drivers->remote.keyPressed(Remote::Key::X) - drivers->remote.keyPressed(Remote::Key::Z);
        heroStartYawUnwrappedEncoder += startEncoderDelta;
#endif

        currentYawAngleDisplay = modm::toDegree(currentYawAngle.getValue());
        currentPitchAngleDisplay = modm::toDegree(currentPitchAngle.getValue());

        // Flush whatever our current output is to the motors
        yawMotor.setDesiredOutput(desiredYawMotorOutput);

        ////////////////
        // DEBUG VARS //
        ////////////////
        yawMotorAngleDisplay = modm::toDegree(currentYawAngle.getValue());
        yawOutputDisplay = desiredYawMotorOutput;
    }

    if (pitchMotor.isMotorOnline()) {
        // Update subsystem state to stay up-to-date with reality
        uint16_t currentPitchEncoderPosition = pitchMotor.getEncoderWrapped();
        currentPitchAngle.setValue(DJIEncoderValueToRadians(currentPitchEncoderPosition) * GIMBAL_PITCH_GEAR_RATIO);

        pitchMotorAngleDisplay = modm::toDegree(currentPitchAngle.getValue());

        pitchOutputDisplay = desiredPitchMotorOutput;

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

float GimbalSubsystem::getChassisRelativeYawAngle(AngleUnit unit) const {
    return tap::algorithms::ContiguousFloat(
               (unit == AngleUnit::Degrees)
                   ? -1 * (modm::toDegree(currentYawAngle.getValue() - modm::toRadian(YAW_OFFSET_ANGLE)))
                   : -1 * (currentYawAngle.getValue() - modm::toRadian(YAW_OFFSET_ANGLE)),
               (unit == AngleUnit::Degrees) ? -180.0f : -M_PI,
               (unit == AngleUnit::Degrees) ? 180.0f : M_PI)
        .getValue();
}

float GimbalSubsystem::getChassisRelativePitchAngle(AngleUnit unit) const {
    return tap::algorithms::ContiguousFloat(
               (unit == AngleUnit::Degrees)
                   ? -1 * (modm::toDegree(currentPitchAngle.getValue() - modm::toRadian(PITCH_OFFSET_ANGLE)))
                   : -1 * (currentPitchAngle.getValue() - modm::toRadian(PITCH_OFFSET_ANGLE)),
               (unit == AngleUnit::Degrees) ? -180.0f : -M_PI,
               (unit == AngleUnit::Degrees) ? 180.0f : M_PI)
        .getValue();
}

}  // namespace src::Gimbal
