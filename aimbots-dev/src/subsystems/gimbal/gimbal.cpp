#include "gimbal.hpp"

#include <modm/math.hpp>

static float wrappedEncoderValueToDegrees(int64_t encoderValue) {
    return (360.0f * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

static float wrappedEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

namespace src::Gimbal {

GimbalSubsystem::GimbalSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      yawMotor(drivers,
               YAW_MOTOR_ID,
               GIMBAL_CAN_BUS,
               false,
               "YAW_MOTOR"),
      pitchMotor(drivers,
                 PITCH_MOTOR_ID,
                 GIMBAL_CAN_BUS,
                 false,
                 "PITCH_MOTOR"),
      yawPID(
          POSITION_PID_KP,
          POSITION_PID_KI,
          POSITION_PID_KD,
          POSITION_PID_MAX_ERROR_SUM,
          POSITION_PID_MAX_OUTPUT
      ),
      pitchPID(
          POSITION_PID_KP,
          POSITION_PID_KI,
          POSITION_PID_KD,
          POSITION_PID_MAX_ERROR_SUM,
          POSITION_PID_MAX_OUTPUT
      ),
      targetYawAngle(0.0f),
      targetPitchAngle(0.0f) { }

void GimbalSubsystem::initialize() {
    yawMotor.initialize();
    pitchMotor.initialize();
}

void GimbalSubsystem::refresh(){
    uint16_t currentYawEncoderPosition   = yawMotor.getEncoderWrapped();
    uint16_t currentPitchEncoderPosition = pitchMotor.getEncoderWrapped();

    currentYawAngle   = wrappedEncoderValueToRadians(currentYawEncoderPosition);
    currentPitchAngle = wrappedEncoderValueToRadians(currentPitchEncoderPosition);
}

void GimbalSubsystem::setYawMotorOutputAngleInDegrees(float angle) { 
    targetYawAngle = modm::toRadian(angle);
    yawPID.update(targetYawAngle - currentYawAngle);
    if(yawMotor.isMotorOnline()) {
        yawMotor.setDesiredOutput(yawPID.getValue());
    }
}

void GimbalSubsystem::setYawMotorOutputAngleInRadians(float angle) {
    targetYawAngle = angle;
    yawPID.update(targetYawAngle - currentYawAngle);
    if(yawMotor.isMotorOnline()) {
        yawMotor.setDesiredOutput(yawPID.getValue());
    }
}

void GimbalSubsystem::displaceYawMotorOutputAngleInDegrees(float angle) { 
    targetYawAngle = currentYawAngle + modm::toRadian(angle);
    yawPID.update(modm::toRadian(angle));
    if(yawMotor.isMotorOnline()) {
        yawMotor.setDesiredOutput(yawPID.getValue());
    }
}

void GimbalSubsystem::displaceYawMotorOutputAngleInRadians(float angle) {
    targetYawAngle = currentYawAngle + angle;
    yawPID.update(angle);
    if(yawMotor.isMotorOnline()) {
        yawMotor.setDesiredOutput(yawPID.getValue());
    }
}

void GimbalSubsystem::setPitchMotorOutputAngleInDegrees(float angle) { 
    targetPitchAngle = modm::toRadian(angle);
    pitchPID.update(targetYawAngle - currentYawAngle);
    if(pitchMotor.isMotorOnline()) {
        pitchMotor.setDesiredOutput(pitchPID.getValue());
    }
}

void GimbalSubsystem::setPitchMotorOutputAngleInRadians(float angle) {
    targetPitchAngle = angle;
    pitchPID.update(targetPitchAngle - currentPitchAngle);
    if(pitchMotor.isMotorOnline()) {
        pitchMotor.setDesiredOutput(pitchPID.getValue());
    }
}

void GimbalSubsystem::displacePitchMotorOutputAngleInDegrees(float angle) { 
    targetPitchAngle = currentPitchAngle + modm::toRadian(angle);
    pitchPID.update(modm::toRadian(angle));
    if(pitchMotor.isMotorOnline()) {
        pitchMotor.setDesiredOutput(pitchPID.getValue());
    }
}

void GimbalSubsystem::displacePitchMotorOutputAngleInRadians(float angle) {
    targetPitchAngle = currentPitchAngle + angle;
    pitchPID.update(angle);
    if(pitchMotor.isMotorOnline()) {
        pitchMotor.setDesiredOutput(pitchPID.getValue());
    }
}

float GimbalSubsystem::getCurrentYawAngleInDegrees() const {
    return modm::toDegree(currentYawAngle);
}

float GimbalSubsystem::getCurrentYawAngleInRadians() const {
    return currentYawAngle;
}

float GimbalSubsystem::getCurrentPitchAngleInDegrees() const {
    return modm::toDegree(currentPitchAngle);
}

float GimbalSubsystem::getCurrentPitchAngleInRadians() const {
    return currentPitchAngle;
}

float GimbalSubsystem::getTargetYawAngleInDegrees() const {
    return modm::toDegree(targetYawAngle);
}

float GimbalSubsystem::getTargetYawAngleInRadians() const {
    return targetYawAngle;
}

float GimbalSubsystem::getTargetPitchAngleInDegrees() const {
    return modm::toDegree(targetPitchAngle);
}

float GimbalSubsystem::getTargetPitchAngleInRadians() const {
    return targetPitchAngle;
}

} // namespace src::Gimbal