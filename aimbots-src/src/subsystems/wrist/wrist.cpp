#include "subsystems/wrist/wrist.hpp"

#include <cmath>

#include "utils/common_types.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {

WristSubsystem::WristSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
    drivers(drivers),
    motors {
        buildMotor(YAW),
        buildMotor(PITCH),
        buildMotor(ROLL)
    },
    positionPIDs {
        new SmoothPID(YAW_POSITION_PID_CONFIG),
        new SmoothPID(PITCH_POSITION_PID_CONFIG),
        new SmoothPID(ROLL_POSITION_PID_CONFIG)
    },
    velocityPIDs {
        new SmoothPID(YAW_VELOCITY_PID_CONFIG),
        new SmoothPID(PITCH_VELOCITY_PID_CONFIG),
        new SmoothPID(ROLL_VELOCITY_PID_CONFIG)
    },
    targetAngles {
        new ContiguousFloat(.0f, -M_PI, M_PI),
        new ContiguousFloat(.0f, -M_PI, M_PI),
        new ContiguousFloat(.0f, -M_PI, M_PI)
    },
    currentAngles {
        new ContiguousFloat(WRIST_MOTOR_OFFSET_ANGLES[YAW], -M_PI, M_PI),
        new ContiguousFloat(WRIST_MOTOR_OFFSET_ANGLES[PITCH], -M_PI, M_PI),
        new ContiguousFloat(WRIST_MOTOR_OFFSET_ANGLES[ROLL], -M_PI, M_PI)
    }
{
}

void WristSubsystem::initialize() {
    ForAllWristMotors(&DJIMotor::initialize);
    ForAllWristMotors(&DJIMotor::setDesiredOutput, static_cast<int32_t>(0));
}

void WristSubsystem::refresh() {
    ForAllWristMotors(&WristSubsystem::updateCurrentAngle);
    ForAllWristMotors(&WristSubsystem::updateMotorPID);
    ForAllWristMotors(&WristSubsystem::setDesiredOutputToMotor);
}

void WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {
    // TODO: not implemented at the moment
}

// Doing cascade PIDs, feed position error to get desired velocity, feed velocity error to
// get desired acceleration basically
void WristSubsystem::updateMotorPID(MotorIndex idx) {
    if (motors[idx]->isMotorOnline()) {
        // ContiguousFloat::difference does (other - this), and we want (target - current)
        float outputPositionErr = currentAngles[idx]->difference(targetAngles[idx]->getValue());

        float desiredVelocity = positionPIDs[idx]->runControllerDerivateError(outputPositionErr);
        float velocityError = desiredVelocity - RPM_TO_RADPS(motors[idx]->getShaftRPM());

        float accelerationOutput = velocityPIDs[idx]->runControllerDerivateError(velocityError);

        setDesiredOutput(idx, accelerationOutput);
    }
}

float WristSubsystem::getUnwrappedRadians(MotorIndex motorIdx) const {
    float inPerOut = WRIST_MOTOR_IN_PER_OUT_RATIOS[motorIdx];
    float scaledEncoderUnwrapped = motors[motorIdx]->getEncoderUnwrapped() / inPerOut;

    return DJIEncoderValueToRadians(scaledEncoderUnwrapped);
}

/** Updates the current output angles scaled after gear boxes */
void WristSubsystem::updateCurrentAngle(MotorIndex motorIdx) {
    float angleScaled = getUnwrappedRadians(motorIdx);
    float angleOffsetted = angleScaled - WRIST_MOTOR_OFFSET_ANGLES[motorIdx];
}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE
