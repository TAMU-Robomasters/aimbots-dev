#include "gimbal.hpp"

#include <utils/tools/common_types.hpp>

#include "tap/communication/sensors/buzzer/buzzer.hpp"

#ifdef GIMBAL_COMPATIBLE

static inline float wrapAngleToPiRange(float angle) { return fmodf(angle + M_PI, M_TWOPI) - M_PI; }

namespace src::Gimbal {

GimbalSubsystem::GimbalSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      currentYawAxisAngle(0.0f, -M_PI, M_PI),
      currentPitchAxisAngle(0.0f, -M_PI, M_PI),
      targetYawAxisAngle(YAW_AXIS_START_ANGLE, -M_PI, M_PI),
      targetPitchAxisAngle(PITCH_AXIS_START_ANGLE, -M_PI, M_PI)
{
    BuildYawMotors();
    BuildPitchMotors();
}

void GimbalSubsystem::initialize() {
    ForAllYawMotors(&DJIMotor::initialize);
    ForAllPitchMotors(&DJIMotor::initialize);

    setAllDesiredYawMotorOutputs(0);
    setAllDesiredPitchMotorOutputs(0);
    ForAllYawMotors(&GimbalSubsystem::setDesiredOutputToYawMotor);
    ForAllPitchMotors(&GimbalSubsystem::setDesiredOutputToPitchMotor);
}

float pitchOutputDisplay = 0.0f;
float yawOutputDisplay = 0.0f;
bool pitchOnline = false;

float currentYawAxisAngleDisplay = 0.0f;
float currByFuncYawAngleDisplay = 0;
float currentPitchAxisAngleDisplay = 0.0f;

float currentYawMotorAngleDisplay = 0.0f;
float otherYawMotorAngleDisplay = 0.0f;
float currentPitchMotorAngleDisplay = 0.0f;

float yawAxisMotorSpeedDisplay = 0.0f;

int yawOnlineCountDisplay = 0;
int pitchOnlineCountDisplay = 0;

int yawDisplayMotorIdx = 0;  // don't change in code, should only be changed in debugger
int pitchDisplayMotorIdx = 0;

float yawDesiredOutputDisplay = 0;
float pitchDesiredOutputDisplay = 0;

float currentYawAxisAngleByMotorDisplay = 0;

float pitchLimitedOutputDisplay = 0;

void GimbalSubsystem::refresh() {
    int yawOnlineCount = 0;
    float yawAxisAngleSum = 0.0f;

    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        if (!yawMotors[i]->isMotorOnline()) {
            // tap::buzzer::playNote(&drivers->pwm, 932);
            continue;
        }

        // tap::buzzer::playNote(&drivers->pwm, 0);
        yawOnlineCount++;

        int64_t currentYawEncoderPosition = yawMotors[i]->getInternalEncoder().getEncoder().getUnwrappedValue();

        // https://www.desmos.com/calculator/bducsk7y6v
        float wrappedYawAxisAngle =
            GIMBAL_YAW_GEAR_RATIO * (DJIEncoderValueToRadians(currentYawEncoderPosition) - YAW_MOTOR_OFFSET_ANGLES[i]);

        currentYawAxisAnglesByMotor[i]->setWrappedValue(wrappedYawAxisAngle);

        yawAxisAngleSum += wrappedYawAxisAngle;
        ////////////////
        // DEBUG VARS //
        ////////////////
        currentYawAxisAngleByMotorDisplay = currentYawAxisAnglesByMotor[yawDisplayMotorIdx]->getWrappedValue();
        currentYawMotorAngleDisplay =
            modm::toDegree(DJIEncoderValueToRadians(yawMotors[yawDisplayMotorIdx]->getInternalEncoder().getEncoder().getUnwrappedValue()));
        otherYawMotorAngleDisplay =
            modm::toDegree(DJIEncoderValueToRadians(yawMotors[abs(yawDisplayMotorIdx - 1)]->getInternalEncoder().getEncoder().getUnwrappedValue()));
        yawOutputDisplay = yawMotors[i]->getOutputDesired();

        yawAxisMotorSpeedDisplay = yawMotors[yawDisplayMotorIdx]->getInternalEncoder().getShaftRPM();

        yawDesiredOutputDisplay = desiredYawMotorOutputs[yawDisplayMotorIdx];

        // flush the desired output to the motor
        setDesiredOutputToYawMotor(i);
    }
    yawOnlineCountDisplay = yawOnlineCount;

    int pitchOnlineCount = 0;
    pitchOnline = pitchMotors[0]->isMotorOnline();
    float pitchAxisAngleSum = 0.0f;
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        if (!pitchMotors[i]->isMotorOnline()) {
            continue;
        }
        pitchOnlineCount++;

        int64_t currentPitchEncoderPosition = pitchMotors[i]->getInternalEncoder().getEncoder().getUnwrappedValue();

        // https://www.desmos.com/calculator/fydwmos1xr
        float wrappedPitchAxisAngle =
            GIMBAL_PITCH_GEAR_RATIO *
            wrapAngleToPiRange(DJIEncoderValueToRadians(currentPitchEncoderPosition) - PITCH_MOTOR_OFFSET_ANGLES[i]);

        currentPitchAxisAnglesByMotor[i]->setWrappedValue(wrappedPitchAxisAngle);

        pitchAxisAngleSum += wrappedPitchAxisAngle;
        ////////////////
        // DEBUG VARS //
        ////////////////
        currentPitchMotorAngleDisplay =
            modm::toDegree(DJIEncoderValueToRadians(pitchMotors[pitchDisplayMotorIdx]->getInternalEncoder().getEncoder().getUnwrappedValue()));
        pitchOutputDisplay = pitchMotors[pitchDisplayMotorIdx]->getOutputDesired();
        pitchDesiredOutputDisplay = desiredPitchMotorOutputs[pitchDisplayMotorIdx];

        // flush the desired output to the motor
        setDesiredOutputToPitchMotor(i);
    }
    pitchOnlineCountDisplay = pitchOnlineCount;

    // Set axis angle to be average of all the online motors
    if (yawOnlineCount > 0) {
        currentYawAxisAngle.setWrappedValue(yawAxisAngleSum / yawOnlineCount);
    }
    if (pitchOnlineCount > 0) {
        currentPitchAxisAngle.setWrappedValue(pitchAxisAngleSum / pitchOnlineCount);
    }

    currentYawAxisAngleDisplay = modm::toDegree(currentYawAxisAngle.getWrappedValue());
    currByFuncYawAngleDisplay = modm::toDegree(this->getCurrentYawAxisAngle(AngleUnit::Radians));
    currentPitchAxisAngleDisplay = modm::toDegree(currentPitchAxisAngle.getWrappedValue());

    // update gimbal orientation buffer
    std::pair<float, float> orientation;
    orientation.first = currentYawAxisAngle.getWrappedValue();
    orientation.second = currentPitchAxisAngle.getWrappedValue();

    gimbalOrientationBuffer.prependOverwrite(orientation);
}

void GimbalSubsystem::setDesiredOutputToYawMotor(uint8_t YawIdx) {
    // Clamp output to maximum value that the 6020 can handle
    yawMotors[YawIdx]->setDesiredOutput(
        tap::algorithms::limitVal(desiredYawMotorOutputs[YawIdx], -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT));
}

void GimbalSubsystem::setDesiredOutputToPitchMotor(uint8_t PitchIdx) {
    pitchDesiredOutputDisplay = desiredPitchMotorOutputs[pitchDisplayMotorIdx];
    pitchLimitedOutputDisplay =
        tap::algorithms::limitVal<int32_t>(desiredPitchMotorOutputs[PitchIdx], -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT);
    // Clamp output to maximum value that the 6020 can handle
    pitchMotors[PitchIdx]->setDesiredOutput(
        tap::algorithms::limitVal<int32_t>(desiredPitchMotorOutputs[PitchIdx], -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT));
}

float GimbalSubsystem::getYawMotorSetpointError(uint8_t YawIdx, AngleUnit unit) const {
    // how much the motor needs to turn to get to the target angle
    float motorAngleError = currentYawAxisAnglesByMotor[YawIdx]->minDifference(targetYawAxisAngle) / GIMBAL_YAW_GEAR_RATIO;
    // target - current

    return (unit == AngleUnit::Radians) ? motorAngleError : modm::toDegree(motorAngleError);
}

float GimbalSubsystem::getPitchMotorSetpointError(uint8_t PitchIdx, AngleUnit unit) const {
    // how much the motor needs to turn to get to the target angle
    float motorAngleError =
        currentPitchAxisAnglesByMotor[PitchIdx]->minDifference(targetPitchAxisAngle) / GIMBAL_PITCH_GEAR_RATIO;

    return (unit == AngleUnit::Radians) ? motorAngleError : modm::toDegree(motorAngleError);
}

}  // namespace src::Gimbal

#endif  //#ifdef GIMBAL_COMPATIBLE