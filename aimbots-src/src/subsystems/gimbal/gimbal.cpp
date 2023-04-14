#include "gimbal.hpp"

#include <drivers.hpp>
#include <utils/common_types.hpp>

static inline float DJIEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

static inline float wrapAngleToPiRange(float angle) { return fmod(angle + M_PI, M_TWOPI) - M_PI; }

namespace src::Gimbal {

GimbalSubsystem::GimbalSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      yawMotor(drivers, YAW_MOTOR_ID, GIMBAL_BUS, YAW_DIRECTION, "Yaw Motor"),
      pitchMotor(drivers, PITCH_MOTOR_ID, GIMBAL_BUS, PITCH_DIRECTION, "Pitch Motor"),
      currentYawAngle(0.0f, -M_PI, M_PI),
      currentPitchAngle(0.0f, -M_PI, M_PI),
      targetYawAngle(0.0f, -M_PI, M_PI),
      targetPitchAngle(0.0f, -M_PI, M_PI) {}

void GimbalSubsystem::initialize() {
    yawMotor.initialize();
    pitchMotor.initialize();

    yawMotor.setDesiredOutput(0);
    pitchMotor.setDesiredOutput(0);
}

float pitchOutputDisplay = 0.0f;
float yawOutputDisplay = 0.0f;

float currentYawAngleDisplay = 0.0f;
float currentPitchAngleDisplay = 0.0f;

float currentYawMotorAngleDiplay = 0.0f;
float currentPitchMotorAngleDisplay = 0.0f;

void GimbalSubsystem::refresh() {
    if (yawMotor.isMotorOnline()) {
        // Update subsystem state to stay up-to-date with reality

        uint16_t currentYawEncoderPosition = yawMotor.getEncoderUnwrapped();

        float unwrappedYawAngle = wrapAngleToPiRange(
            GIMBAL_YAW_GEAR_RATIO * DJIEncoderValueToRadians(currentYawEncoderPosition) - YAW_OFFSET_ANGLE);

        currentYawAngle.setValue(unwrappedYawAngle);

        // Flush whatever our current output is to the motors
        yawMotor.setDesiredOutput(desiredYawMotorOutput);

        ////////////////
        // DEBUG VARS //
        ////////////////
        currentYawAngleDisplay = modm::toDegree(currentYawAngle.getValue());
        yawOutputDisplay = desiredYawMotorOutput;
    }

    if (pitchMotor.isMotorOnline()) {
        // Update subsystem state to stay up-to-date with reality
        uint16_t currentPitchEncoderPosition = pitchMotor.getEncoderUnwrapped();

        uint16_t unwrappedPitchAngle =
            GIMBAL_PITCH_GEAR_RATIO *
            wrapAngleToPiRange(DJIEncoderValueToRadians(currentPitchEncoderPosition) - PITCH_OFFSET_ANGLE);

        currentPitchAngle.setValue(unwrappedPitchAngle);

        // Flush whatever our current output is to the motors
        pitchMotor.setDesiredOutput(desiredPitchMotorOutput);

        ////////////////
        // DEBUG VARS //
        ////////////////
        currentPitchAngleDisplay = modm::toDegree(currentPitchAngle.getValue());
        pitchOutputDisplay = desiredPitchMotorOutput;
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
    return (unit == AngleUnit::Radians) ? currentYawAngle.getValue() : modm::toDegree(currentYawAngle.getValue());
}

float GimbalSubsystem::getChassisRelativePitchAngle(AngleUnit unit) const {
    return (unit == AngleUnit::Radians) ? currentPitchAngle.getValue() : modm::toDegree(currentPitchAngle.getValue());
}

}  // namespace src::Gimbal
