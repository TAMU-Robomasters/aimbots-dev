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
    motors {
        buildMotor(YAW),
        buildMotor(PITCH),
        buildMotor(ROLL)
    },
    positionPIDs {
        SmoothPID(YAW_POSITION_PID_CONFIG_temp),
        SmoothPID(PITCH_POSITION_PID_CONFIG),
        SmoothPID(ROLL_POSITION_PID_CONFIG)
    },
    velocityPIDs {
        SmoothPID(YAW_VELOCITY_PID_CONFIG_temp),
        SmoothPID(PITCH_VELOCITY_PID_CONFIG),
        SmoothPID(ROLL_VELOCITY_PID_CONFIG)
    }
{
}

void WristSubsystem::initialize() {

}

void WristSubsystem::refresh() {
    // ForAllWristMotors(&WristSubsystem::setDesiredOutputToMotor);
}

void WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {
    // TODO: not implemented at the moment
}

float yawAngle = 0.0f;
float yawVelocity = 0.0f;
float yawDesiredAngle = 0.0f;
float yawDesiredVelocity = 0.0f;
float yawDesiredOut = 0.0f;

// Doing cascade PIDs, feed position error to get desired velocity, feed velocity error to
// get desired acceleration basically
void WristSubsystem::updateMotorPID(MotorIndex idx) {
    if (isMotorOnline(idx)) {
        float errorRadians = targetAnglesRads[idx] - getScaledUnwrappedRadiansOffset(idx);

        float desiredVelocityRadsPs = positionPIDs[idx].runControllerDerivateError(errorRadians);
        float velocityErrorRadsPs = desiredVelocityRadsPs - getMotorScaledRadsPs(idx);

        float accelerationOutput = velocityPIDs[idx].runControllerDerivateError(velocityErrorRadsPs);

        desiredMotorOutputs[idx] = accelerationOutput;

        if (idx == YAW)
        {
            yawAngle = getScaledUnwrappedRadiansOffset(idx);
            yawVelocity = getMotorScaledRadsPs(idx);
            yawDesiredAngle = targetAnglesRads[idx];
            yawDesiredVelocity = desiredVelocityRadsPs;
            yawDesiredOut = accelerationOutput;
        }
    }
}

float WristSubsystem::getScaledUnwrappedRadians(MotorIndex motorIdx) const {
    float inPerOut = WRIST_MOTOR_IN_PER_OUT_RATIOS[motorIdx];
    float unwrappedRadians = DJIEncoderValueToRadians(motors[motorIdx].getEncoderUnwrapped());

    return unwrappedRadians / inPerOut;
}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE