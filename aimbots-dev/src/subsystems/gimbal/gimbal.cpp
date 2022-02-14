#include "gimbal.hpp"

#include <tap/algorithms/math_user_utils.hpp>

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
      currentYawAngle(0.0f),
      currentPitchAngle(0.0f),
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

void GimbalSubsystem::setYawMotorOutput(float output)
{
    setMotorOutput(&yawMotor, output);
}

void GimbalSubsystem::setPitchMotorOutput(float output)
{
    setMotorOutput(&pitchMotor, output);
}

} // namespace src::Gimbal