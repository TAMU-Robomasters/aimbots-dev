#include "subsystems/wrist/wrist.hpp"

#include <cmath>

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
        new SmoothPID(WRIST_POSITION_PID_CONFIG),
        new SmoothPID(WRIST_POSITION_PID_CONFIG),
        new SmoothPID(WRIST_POSITION_PID_CONFIG)
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
    ForAllWristMotors(&WristSubsystem::updateMotorPositionPID);
    ForAllWristMotors(&WristSubsystem::setDesiredOutputToMotor);
}

void WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {
    // TODO: not implemented at the moment
}

void WristSubsystem::updateMotorPositionPID(MotorIndex idx) {
    if (motors[idx]->isMotorOnline()) {
        // ContiguousFloat::difference does (other - this), and we want (target - current)
        float outputErr = currentAngles[idx]->difference(targetAngles[idx]->getValue());

        positionPIDs[idx]->runControllerDerivateError(outputErr);
        setDesiredOutput(idx, positionPIDs[idx]->getOutput());
    }
}

float WristSubsystem::getUnwrappedRadians(MotorIndex motorIdx) const {
    float scaledEncoderUnwrapped = motors[motorIdx]->getEncoderUnwrapped() / WRIST_GEAR_RATIOS[motorIdx];
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
