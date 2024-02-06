#include "subsystems/wrist/wrist.hpp"
#include "utils/common_types.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {

WristSubsystem::WristSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
    motors {
        buildMotor(YAW),
        buildMotor(PITCH),
        buildMotor(ROLL)
    },
    positionPIDs {
        SmoothPID(YAW_POSITION_PID_CONFIG),
        SmoothPID(PITCH_POSITION_PID_CONFIG),
        SmoothPID(ROLL_POSITION_PID_CONFIG)
    },
    velocityPIDs {
        SmoothPID(YAW_VELOCITY_PID_CONFIG),
        SmoothPID(PITCH_VELOCITY_PID_CONFIG),
        SmoothPID(ROLL_VELOCITY_PID_CONFIG)
    }
{
}

void WristSubsystem::initialize() {
    ForAllWristMotors(&DJIMotor::initialize);
}

void WristSubsystem::refresh() {
    ForAllWristMotors(&WristSubsystem::setDesiredOutputToMotor);
}

void WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {
    // TODO: not implemented at the moment
}

void WristSubsystem::updateAllPIDs() {
    ForAllWristMotors(&WristSubsystem::updateMotorPID);
}

// Doing cascade PIDs, feed position error to get desired velocity, feed velocity error to
// get desired acceleration basically
void WristSubsystem::updateMotorPID(MotorIndex idx) {
    if (isMotorOnline(idx)) {
        float errorRadians = targetAnglesRads[idx] - getScaledUnwrappedRadiansOffset(idx);

        float desiredVelocityRadsPs = positionPIDs[idx].runControllerDerivateError(errorRadians);
        float velocityErrorRadsPs = desiredVelocityRadsPs - getMotorScaledRadsPs(idx);

        float accelerationOutput = velocityPIDs[idx].runControllerDerivateError(velocityErrorRadsPs);

        desiredMotorOutputs[idx] = accelerationOutput;
    }
}

float WristSubsystem::getScaledUnwrappedRadians(MotorIndex motorIdx) const {
    float inPerOut = WRIST_MOTOR_IN_PER_OUT_RATIOS[motorIdx];
    float unwrappedRadians = DJIEncoderValueToRadians(motors[motorIdx].getEncoderUnwrapped());

    return unwrappedRadians / inPerOut;
}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE