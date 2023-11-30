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

//DEBUG
float pitchTargetAngle_disp = 0;
float pitchCurrentAngle_disp = 0;
float pitchDesiredOutput_disp = 0;
float yawTargetAngle_disp = 0;
float yawCurrentAngle_disp = 0;
float yawDesiredOutput_disp = 0;
float rollTargetAngle_disp = 0;
float rollCurrentAngle_disp = 0;
float rollDesiredOutput_disp = 0;

void WristSubsystem::refresh() {
    ForAllWristMotors(&WristSubsystem::updateCurrentAngle);
    ForAllWristMotors(&WristSubsystem::updateMotorPositionPID);
    ForAllWristMotors(&WristSubsystem::setDesiredOutputToMotor);

    yawCurrentAngle_disp = currentAngles[YAW]->getValue();
    yawTargetAngle_disp = targetAngles[YAW]->getValue();
    pitchTargetAngle_disp = targetAngles[PITCH]->getValue();
    rollTargetAngle_disp = targetAngles[ROLL]->getValue();
}

float WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {
    // TODO: not implemented at the moment
    return 0.0f;//temp
}

void WristSubsystem::updateMotorPositionPID(MotorIndex idx) {
    if (motors[idx]->isMotorOnline()) {
        // ContiguousFloat::difference does (other - this), and we want (target - current)
        float outputErr = currentAngles[idx]->difference(targetAngles[idx]->getValue());

        positionPIDs[idx]->runControllerDerivateError(outputErr);
        setDesiredOutput(idx, positionPIDs[idx]->getOutput());
        if (idx == PITCH)
            pitchDesiredOutput_disp = positionPIDs[idx]->getOutput();
        if (idx == YAW)
            yawDesiredOutput_disp = positionPIDs[idx]->getOutput();
        if (idx == ROLL)
            rollDesiredOutput_disp = positionPIDs[idx]->getOutput();
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
    if (motorIdx == PITCH)
        pitchCurrentAngle_disp = angleOffsetted;
    if (motorIdx == YAW)
        yawCurrentAngle_disp = angleOffsetted;
    if (motorIdx == ROLL)
        rollCurrentAngle_disp = angleOffsetted;
}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE
