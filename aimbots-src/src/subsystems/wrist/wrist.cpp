#include "subsystems/wrist/wrist.hpp"

#include "utils/common_types.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {

WristSubsystem::WristSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
      motors{buildMotor(YAW), buildMotor(PITCH), buildMotor(ROLL)},
      positionPIDs{
          SmoothPID(YAW_POSITION_PID_CONFIG),
          SmoothPID(PITCH_POSITION_PID_CONFIG),
          SmoothPID(ROLL_POSITION_PID_CONFIG)},
      velocityPIDs{
          SmoothPID(YAW_VELOCITY_PID_CONFIG),
          SmoothPID(PITCH_VELOCITY_PID_CONFIG),
          SmoothPID(ROLL_VELOCITY_PID_CONFIG)} {}

void WristSubsystem::initialize() { ForAllWristMotors(&DJIMotor::initialize); }

void WristSubsystem::refresh() { ForAllWristMotors(&WristSubsystem::setDesiredOutputToMotor); }

void WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {
    // TODO: not implemented at the moment
    UNUSED(x);
    UNUSED(y);
    UNUSED(z);
}

void WristSubsystem::updateAllPIDs() {
    // if (motor_control_setting == VELOCITY) {
    //     ForAllWristMotors(&WristSubsystem::updateMotorPID_velocity);
    // } else if (motor_control_setting == POSITION) {
    //     ForAllWristMotors(&WristSubsystem::updateMotorPID);
    // }
    ForAllWristMotors(&WristSubsystem::updateMotorPID);
}

void WristSubsystem::updateMotorPID(MotorIndex idx) {
    float errorRadians = targetAnglesRads[idx] - getScaledUnwrappedRadiansOffset(idx);
    float errorDerivative = getMotorRPM(idx);
    float output = positionPIDs[idx].runController(errorRadians, errorDerivative);

    desiredMotorOutputs[idx] = output;
}

void WristSubsystem::updateMotorPID_velocity(MotorIndex idx) {
    float motorRpm = getMotorRPM(idx);
    float rpm_error = targetRPMs[idx] - motorRpm;
    float output = velocityPIDs[idx].runControllerDerivateError(rpm_error);

    output = output;  //Suppressing a warning
}

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