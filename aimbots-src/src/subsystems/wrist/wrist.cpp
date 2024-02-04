#include "subsystems/wrist/wrist.hpp"

#include <cmath>

#include "utils/common_types.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {

static constexpr SmoothPIDConfig YAW_POSITION_PID_CONFIG_temp = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,  // 500
    .maxICumulative = 0.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig YAW_VELOCITY_PID_CONFIG_temp = {
    .kp = 100.0f,  // 3000
    .ki = 0.0f,    // 25
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

WristSubsystem::WristSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
    drivers(drivers),
    motors {
        buildMotor(YAW),
        buildMotor(PITCH),
        buildMotor(ROLL)
    },
    positionPIDs {
        new SmoothPID(YAW_POSITION_PID_CONFIG_temp),
        new SmoothPID(PITCH_POSITION_PID_CONFIG),
        new SmoothPID(ROLL_POSITION_PID_CONFIG)
    },
    velocityPIDs {
        new SmoothPID(YAW_VELOCITY_PID_CONFIG_temp),
        new SmoothPID(PITCH_VELOCITY_PID_CONFIG),
        new SmoothPID(ROLL_VELOCITY_PID_CONFIG)
    },
    targetRadians {
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

float WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {
    // TODO: not implemented at the moment
    return 0.0f;//temp
}

float yawAngle = 0.0f;
float yawVelocity = 0.0f;
float yawDesiredAngle = 0.0f;
float yawDesiredVelocity = 0.0f;
float yawDesiredOut = 0.0f;

// Doing cascade PIDs, feed position error to get desired velocity, feed velocity error to
// get desired acceleration basically
void WristSubsystem::updateMotorPID(MotorIndex idx) {
    if (motors[idx]->isMotorOnline()) {
        // ContiguousFloat::difference does (other - this), and we want (target - current)
        float posErrorRads = currentAngles[idx]->difference(targetRadians[idx]->getValue());

        float desiredVelocityRadsPs = positionPIDs[idx]->runControllerDerivateError(posErrorRads);
        float scaledShaftRadsPs = RPM_TO_RADPS(motors[idx]->getShaftRPM()) / WRIST_MOTOR_IN_PER_OUT_RATIOS[idx];
        float velocityErrorRadsPs = desiredVelocityRadsPs - scaledShaftRadsPs;

        float accelerationOutput = velocityPIDs[idx]->runControllerDerivateError(velocityErrorRadsPs);

        setDesiredOutput(idx, accelerationOutput);

        if (idx == YAW)
        {
            yawAngle = currentAngles[idx]->getValue();
            yawVelocity = scaledShaftRadsPs;
            yawDesiredAngle = targetRadians[idx]->getValue();
            yawDesiredVelocity = desiredVelocityRadsPs;
            yawDesiredOut = accelerationOutput;
        }
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

    currentAngles[motorIdx]->setValue(angleOffsetted);
}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE
