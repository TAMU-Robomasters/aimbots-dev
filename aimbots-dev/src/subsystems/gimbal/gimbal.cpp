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
    : tap::control::Subsystem(drivers)
    , yawMotor(drivers,
               YAW_MOTOR_ID,
               GIMBAL_CAN_BUS,
               false,
               "YAW_MOTOR")
    , pitchMotor(drivers,
                 PITCH_MOTOR_ID,
                 GIMBAL_CAN_BUS,
                 false,
                 "PITCH_MOTOR")
    , yawPID()
    , pitchPID()
    , targetYawAngle(0.0f)
    , targetPitchAngle(0.0f) { }

void GimbalSubsystem::initialize() {
    yawMotor.initialize();
    pitchMotor.initialize();
}

void GimbalSubsystem::refresh(){
    // TODO: do logic here
}

void GimbalSubsystem::setYawAngleInDegrees(float angle) { targetYawAngle = modm::toRadian(angle); }
void GimbalSubsystem::setYawAngleInRadians(float angle) { targetYawAngle = angle; }

void GimbalSubsystem::setPitchAngleInDegrees(float angle) { targetPitchAngle = modm::toRadian(angle); }
void GimbalSubsystem::setPitchAngleInRadians(float angle) { targetPitchAngle = angle; }

void GimbalSubsystem::displaceYawAngleInDegrees(float displacement) {
    targetYawAngle = getCurrentYawAngleInRadians() + modm::toRadian(displacement);
}

void GimbalSubsystem::displaceYawAngleInRadians(float displacement) {
    targetYawAngle = getCurrentYawAngleInRadians() + displacement;
}

void GimbalSubsystem::displacePitchAngleInDegrees(float displacement) {
    targetPitchAngle = getCurrentPitchAngleInRadians() + modm::toRadian(displacement);
}

void GimbalSubsystem::displacePitchAngleInRadians(float displacement) {
    targetPitchAngle = getCurrentPitchAngleInRadians() + displacement;
}

float GimbalSubsystem::getCurrentYawAngleInDegrees() const {
    return wrappedEncoderValueToDegrees(yawMotor.getEncoderWrapped());
}

float GimbalSubsystem::getCurrentYawAngleInRadians() const {
    return wrappedEncoderValueToRadians(yawMotor.getEncoderWrapped());
}

float GimbalSubsystem::getCurrentPitchAngleInDegrees() const {
    return wrappedEncoderValueToDegrees(pitchMotor.getEncoderWrapped());
}

float GimbalSubsystem::getCurrentPitchAngleInRadians() const {
    return wrappedEncoderValueToRadians(pitchMotor.getEncoderWrapped());
}

float GimbalSubsystem::getTargetYawAngleInDegrees() const { return targetYawAngle; }
float GimbalSubsystem::getTargetYawAngleInRadians() const { return targetYawAngle; }

float GimbalSubsystem::getTargetPitchAngleInDegrees() const { return targetPitchAngle; }
float GimbalSubsystem::getTargetPitchAngleInRadians() const { return targetPitchAngle; }

} // namespace src::Gimbal