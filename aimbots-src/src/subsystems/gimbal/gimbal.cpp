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
      currentYawAngle(0.0f, -M_PI, M_PI),
      currentPitchAngle(0.0f, -M_PI, M_PI),
      targetYawAngle(0.0f, -M_PI, M_PI),
      targetPitchAngle(0.0f, PITCH_SOFTSTOP_LOW, PITCH_SOFTSTOP_HIGH)  // bounds validated during construction
{
    BuildYawMotors();
    BuildPitchMotors();
}

void GimbalSubsystem::initialize() {
    ForAllYawMotors(&DJIMotor::initialize);
    ForAllPitchMotors(&DJIMotor::initialize);

    setAllDesiredYawOutputs(0);
    setAllDesiredPitchOutputs(0);
    ForAllYawMotors(&GimbalSubsystem::setDesiredOutputToYawMotor);
    ForAllPitchMotors(&GimbalSubsystem::setDesiredOutputToYawMotor);
}

float pitchOutputDisplay = 0.0f;
float yawOutputDisplay = 0.0f;

float currentYawAngleDisplay = 0.0f;
float currentPitchAngleDisplay = 0.0f;

int16_t currentYawMotorAngleDiplay = 0.0f;
int16_t currentPitchMotorAngleDisplay = 0.0f;

void GimbalSubsystem::refresh() {
    // Get first online Yaw motor
    DJIMotor* yawMotor = *std::find_if(yawMotors.begin(), yawMotors.end(), DJIMotor::isMotorOnline);

    if (yawMotor->isMotorOnline()) {  // check again bc find_if returns end() if not found :/
        uint16_t currentYawEncoderPosition = yawMotor->getEncoderUnwrapped();

        // https://www.desmos.com/calculator/bducsk7y6v
        float wrappedYawAngle = wrapAngleToPiRange(
            GIMBAL_YAW_GEAR_RATIO * DJIEncoderValueToRadians(currentYawEncoderPosition) - YAW_OFFSET_ANGLE);

        currentYawAngle.setValue(wrappedYawAngle);

        // Flush whatever our current output is to the motors
        ForAllYawMotors(&GimbalSubsystem::setDesiredOutputToYawMotor);

        ////////////////
        // DEBUG VARS //
        ////////////////
        currentYawMotorAngleDiplay = modm::toDegree(DJIEncoderValueToRadians(yawMotor->getEncoderWrapped()));
        currentYawAngleDisplay = modm::toDegree(currentYawAngle.getValue());
        yawOutputDisplay = yawMotor->getOutputDesired();
    }

    // Get first online Pitch motor
    DJIMotor* pitchMotor = *std::find_if(pitchMotors.begin(), pitchMotors.end(), DJIMotor::isMotorOnline);

    if (pitchMotor->isMotorOnline()) {  // check again bc find_if returns end() if not found :/
        uint16_t currentPitchEncoderPosition = pitchMotor->getEncoderUnwrapped();

        // https://www.desmos.com/calculator/fydwmos1xr
        uint16_t wrappedPitchAngle =
            GIMBAL_PITCH_GEAR_RATIO *
            wrapAngleToPiRange(DJIEncoderValueToRadians(currentPitchEncoderPosition) - PITCH_OFFSET_ANGLE);

        currentPitchAngle.setValue(wrappedPitchAngle);

        // Flush whatever our current output is to the motors
        ForAllPitchMotors(&GimbalSubsystem::setDesiredOutputToPitchMotor);

        ////////////////
        // DEBUG VARS //
        ////////////////
        currentPitchAngleDisplay = modm::toDegree(currentPitchAngle.getValue());
        currentPitchMotorAngleDisplay = modm::toDegree(DJIEncoderValueToRadians(pitchMotor->getEncoderWrapped()));
        pitchOutputDisplay = pitchMotor->getOutputDesired();
    }
}

void GimbalSubsystem::setDesiredOutputToYawMotor(uint8_t YawIdx) {
    // Clamp output to maximum value that the 6020 can handle
    yawMotors[YawIdx]->setDesiredOutput(
        tap::algorithms::limitVal(desiredYawMotorOutputs[YawIdx], -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT));
}

void GimbalSubsystem::setDesiredOutputToPitchMotor(uint8_t PitchIdx) {
    // Clamp output to maximum value that the 6020 can handle
    pitchMotors[PitchIdx]->setDesiredOutput(
        tap::algorithms::limitVal(desiredYawMotorOutputs[PitchIdx], -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT));
}

float GimbalSubsystem::getYawMotorSetpointError(uint8_t YawIdx, AngleUnit unit) const {
    // How much the motor has to turn to get to the desired target
    float motorSetpointError = targetYawAngle.difference(currentYawAngle);

    // How much the motor has actually turned
    float motorAngleError = motorSetpointError - DJIEncoderValueToRadians(yawMotors[YawIdx]->getEncoderWrapped());

    return (unit == AngleUnit::Radians) ? motorAngleError : modm::toDegree(motorAngleError);
}

}  // namespace src::Gimbal
