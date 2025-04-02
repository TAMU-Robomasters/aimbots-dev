#include "subsystems/wrist/control/wrist.hpp"

#include "utils/tools/common_types.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {

WristSubsystem::WristSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
      motors{buildMotor(ROLL), buildMotor(PITCH), buildMotor(YAW)},
      positionPIDs{
          SmoothPID(ROLL_POSITION_PID_CONFIG),
          SmoothPID(PITCH_POSITION_PID_CONFIG),
          SmoothPID(YAW_POSITION_PID_CONFIG)} {}
      /* velocityPIDs{
          SmoothPID(YAW_VELOCITY_PID_CONFIG),
          SmoothPID(PITCH_VELOCITY_PID_CONFIG),
          SmoothPID(ROLL_VELOCITY_PID_CONFIG)} {} */

void WristSubsystem::initialize() { forAllWristMotors(&DJIMotor::initialize); }

void WristSubsystem::refresh() { forAllWristMotors(&WristSubsystem::setDesiredOutputToMotor); }

void WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {
    // TODO: not implemented at the moment
    UNUSED(x);
    UNUSED(y);
    UNUSED(z);
}

void WristSubsystem::updateAllPIDs() {
    setTargetMotorPos();
    forAllWristMotors(&WristSubsystem::updateMotorPID);
}

/* DEBUG */
float inputYaw   = 0;
float inputPitch = 0;
float inputRoll  = 0;
float tmp0 = 0;
float tmp1 = 0;
float tmp2 = 0;

void WristSubsystem::setTargetMotorPos() {
    // Right Motor is motor 0
    // Left Motor is motor 1

    // 0 = ROLL
    // 1 = Pitch
    // 2 = yaw

    // tmp0
    targetMotorPos[0] = targetAnglesRads[0]; // pitch - roll
    // tmp1
    targetMotorPos[1] = (2*targetAnglesRads[1] - targetAnglesRads[2]) / 2; // pitch + roll
    // tmp2
    targetMotorPos[2] = 2*targetAnglesRads[1] - ((2*targetAnglesRads[1] - targetAnglesRads[2]) / 2); // as is, pitch - roll motor
    
    /* targetMotorPos[0] = targetAnglesRads[0];
    targetMotorPos[1] = targetAnglesRads[1];
    targetMotorPos[2] = targetAnglesRads[2]; */
    
    /* DEBUG */
    inputYaw  = targetAnglesRads[0];
    inputPitch = targetAnglesRads[1];
    inputRoll   = targetAnglesRads[2];
    tmp0 = targetMotorPos[0];
    tmp1 = targetMotorPos[1];
    tmp2 = targetMotorPos[2];
}

void WristSubsystem::updateMotorPID(MotorIndex idx) {
    float errorRadians = targetMotorPos[idx] - getScaledUnwrappedRadiansOffset(idx);
    float errorDerivative = getMotorRPM(idx);
    float output = positionPIDs[idx].runController(errorRadians, errorDerivative);

    desiredMotorOutputs[idx] = output;
}

// Old unused code
/* void WristSubsystem::updateMotorPIDVelocity(MotorIndex idx) {
    float motorRpm = getMotorRPM(idx);
    float rpm_error = targetRPMs[idx] - motorRpm;
    float output = velocityPIDs[idx].runControllerDerivateError(rpm_error);

    output = output;  //Suppressing a warning
} */

float WristSubsystem::getScaledUnwrappedRadians(MotorIndex motorIdx) const {
    float inPerOut = WRIST_MOTOR_IN_PER_OUT_RATIOS[motorIdx];
    float unwrappedRadians = DJIEncoderValueToRadians(motors[motorIdx].getEncoderUnwrapped());

    return unwrappedRadians / inPerOut;
}

float WristSubsystem::getMotorScaledRadsPs(MotorIndex motorIdx) const {
    float inPerOut = WRIST_MOTOR_IN_PER_OUT_RATIOS[motorIdx];
    float radsPerSecond = RPM_TO_RADPS(getMotorRPM(motorIdx));
    float scaledOutput = radsPerSecond / inPerOut;

    return scaledOutput;
}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE