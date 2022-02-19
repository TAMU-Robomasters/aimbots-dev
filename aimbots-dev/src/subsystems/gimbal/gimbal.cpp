#include "gimbal.hpp"

//
// Conversion helper functions
//

static inline float wrappedEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

//
// Motor output helper
//
// TODO: In the future we might want to add some ability to limit the
//       rotation of the motors, but for now this should be fine.
//

static inline void setMotorOutput(DJIMotor* motor, float output) {
    // Here we limit the output of the motor so we don't have any weird
    // overflow problems when it gets casted down to a 16-bit integer.

    // FIXME: Get rid of these magic numbers
    output = tap::algorithms::limitVal(output, -30000.0f, 30000.0f);

    if(motor->isMotorOnline()) {
        motor->setDesiredOutput(output);
    }
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
      currentYawAngle(0.0f, 0.0f, M_TWOPI),
      currentPitchAngle(0.0f, 0.0f, M_TWOPI),
      targetYawAngle(0.0f),
      targetPitchAngle(0.0f) { }

void GimbalSubsystem::initialize() {
    yawMotor.initialize();
    pitchMotor.initialize();
}

void GimbalSubsystem::refresh(){
    if(yawMotor.isMotorOnline()) {
        uint16_t currentYawEncoderPosition   = yawMotor.getEncoderWrapped();
        currentYawAngle.setValue(wrappedEncoderValueToRadians(currentYawEncoderPosition));
    }

    if(pitchMotor.isMotorOnline()) {
        uint16_t currentPitchEncoderPosition = pitchMotor.getEncoderWrapped();
        currentPitchAngle.setValue(wrappedEncoderValueToRadians(currentPitchEncoderPosition));
    }
}

void GimbalSubsystem::setYawMotorOutput(float output)
{
    setMotorOutput(&yawMotor, output);
}

void GimbalSubsystem::setPitchMotorOutput(float output)
{
    setMotorOutput(&pitchMotor, output);
}

float GimbalSubsystem::getCurrentYawAngleFromCenterInDegrees() const {
    return tap::algorithms::ContiguousFloat(
        modm::toDegree(currentYawAngle.getValue()) - YAW_START_ANGLE,
        -180.0f,
         180.0f).getValue();
}

float GimbalSubsystem::getCurrentYawAngleFromCenterInRadians() const {
    return tap::algorithms::ContiguousFloat(
        currentYawAngle.getValue() - modm::toRadian(YAW_START_ANGLE),
        -M_PI,
         M_PI).getValue();
}

float GimbalSubsystem::getCurrentPitchAngleFromCenterInDegrees() const {
    return tap::algorithms::ContiguousFloat(
        modm::toDegree(currentPitchAngle.getValue()) - PITCH_START_ANGLE,
        -180.0f,
         180.0f).getValue(); 
}

float GimbalSubsystem::getCurrentPitchAngleFromCenterInRadians() const {
    return tap::algorithms::ContiguousFloat(
        currentPitchAngle.getValue() - modm::toRadian(PITCH_START_ANGLE),
        -M_PI,
         M_PI).getValue();
}

} // namespace src::Gimbal