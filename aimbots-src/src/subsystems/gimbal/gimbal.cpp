#include "gimbal.hpp"

#include <utils/common_types.hpp>

static inline float DJIEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

static inline float wrapAngleToPiRange(float angle) { return fmodf(angle + M_PI, M_TWOPI) - M_PI; }
namespace src::Gimbal {

GimbalSubsystem::GimbalSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      currentYawAxisAngle(0.0f, -M_PI, M_PI),
      currentPitchAxisAngle(0.0f, -M_PI, M_PI),
      targetYawAxisAngle(YAW_AXIS_START_ANGLE, -M_PI, M_PI),
      targetPitchAxisAngle(PITCH_AXIS_START_ANGLE, -M_PI, M_PI)  //
{
    BuildYawMotors();
    BuildPitchMotors();
}

void GimbalSubsystem::initialize() {
    ForAllYawMotors(&DJIMotor::initialize);
    ForAllPitchMotors(&DJIMotor::initialize);

    setAllDesiredYawMotorOutputs(0);
    setAllDesiredPitchOutputs(0);
    ForAllYawMotors(&GimbalSubsystem::setDesiredOutputToYawMotor);
    ForAllPitchMotors(&GimbalSubsystem::setDesiredOutputToPitchMotor);
}

float pitchOutputDisplay = 0.0f;
float yawOutputDisplay = 0.0f;

float currentYawAxisAngleDisplay = 0.0f;
float currentPitchAxisAngleDisplay = 0.0f;

float currentYawMotorAngleDisplay = 0.0f;
float currentPitchMotorAngleDisplay = 0.0f;

float yawAxisMotorSpeedDisplay = 0.0f;

void GimbalSubsystem::refresh() {
    int yawOnlineCount = 0;
    float yawAxisAngleSum = 0.0f;
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        if (!yawMotors[i]->isMotorOnline()) {
            continue;
        }
        yawOnlineCount++;

        int64_t currentYawEncoderPosition = yawMotors[i]->getEncoderUnwrapped();

        // https://www.desmos.com/calculator/bducsk7y6v
        float wrappedYawAxisAngle =
            GIMBAL_YAW_GEAR_RATIO * (DJIEncoderValueToRadians(currentYawEncoderPosition) - YAW_MOTOR_OFFSET_ANGLES[i]);

        currentYawAxisAnglesByMotor[i]->setValue(wrappedYawAxisAngle);

        yawAxisAngleSum += wrappedYawAxisAngle;
        ////////////////
        // DEBUG VARS //
        ////////////////
        currentYawMotorAngleDisplay = modm::toDegree(DJIEncoderValueToRadians(currentYawEncoderPosition));
        yawOutputDisplay = yawMotors[i]->getOutputDesired();

        yawAxisMotorSpeedDisplay = yawMotors[0]->getShaftRPM();

        // flush the desired output to the motor
        setDesiredOutputToYawMotor(i);
    }

    int pitchOnlineCount = 0;
    float pitchAxisAngleSum = 0.0f;
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        if (!pitchMotors[i]->isMotorOnline()) {
            continue;
        }
        pitchOnlineCount++;

        int64_t currentPitchEncoderPosition = pitchMotors[i]->getEncoderUnwrapped();

        // https://www.desmos.com/calculator/fydwmos1xr
        float wrappedPitchAxisAngle =
            GIMBAL_PITCH_GEAR_RATIO *
            wrapAngleToPiRange(DJIEncoderValueToRadians(currentPitchEncoderPosition) - PITCH_MOTOR_OFFSET_ANGLES[i]);

        currentPitchAxisAnglesByMotor[i]->setValue(wrappedPitchAxisAngle);

        pitchAxisAngleSum += wrappedPitchAxisAngle;
        ////////////////
        // DEBUG VARS //
        ////////////////
        currentPitchMotorAngleDisplay = modm::toDegree(DJIEncoderValueToRadians(currentPitchEncoderPosition));
        pitchOutputDisplay = pitchMotors[i]->getOutputDesired();

        // flush the desired output to the motor
        setDesiredOutputToPitchMotor(i);
    }

    // Set axis angle to be average of all the online motors
    currentYawAxisAngle.setValue(yawAxisAngleSum / yawOnlineCount);
    currentPitchAxisAngle.setValue(pitchAxisAngleSum / pitchOnlineCount);

    currentYawAxisAngleDisplay = modm::toDegree(currentYawAxisAngle.getValue());
    currentPitchAxisAngleDisplay = modm::toDegree(currentPitchAxisAngle.getValue());
}

void GimbalSubsystem::setDesiredOutputToYawMotor(uint8_t YawIdx) {
    // Clamp output to maximum value that the 6020 can handle
    yawMotors[YawIdx]->setDesiredOutput(
        tap::algorithms::limitVal(desiredYawMotorOutputs[YawIdx], -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT));
}

void GimbalSubsystem::setDesiredOutputToPitchMotor(uint8_t PitchIdx) {
    // Clamp output to maximum value that the 6020 can handle
    pitchMotors[PitchIdx]->setDesiredOutput(
        tap::algorithms::limitVal(desiredPitchMotorOutputs[PitchIdx], -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT));
}

float GimbalSubsystem::getYawMotorSetpointError(uint8_t YawIdx, AngleUnit unit) const {
    // how much the motor needs to turn to get to the target angle
    float motorAngleError = currentYawAxisAnglesByMotor[YawIdx]->difference(targetYawAxisAngle) / GIMBAL_YAW_GEAR_RATIO;
    // target - current

    return (unit == AngleUnit::Radians) ? motorAngleError : modm::toDegree(motorAngleError);
}

float GimbalSubsystem::getPitchMotorSetpointError(uint8_t PitchIdx, AngleUnit unit) const {
    // how much the motor needs to turn to get to the target angle
    float motorAngleError =
        currentPitchAxisAnglesByMotor[PitchIdx]->difference(targetPitchAxisAngle) / GIMBAL_PITCH_GEAR_RATIO;

    return (unit == AngleUnit::Radians) ? motorAngleError : modm::toDegree(motorAngleError);
}

}  // namespace src::Gimbal