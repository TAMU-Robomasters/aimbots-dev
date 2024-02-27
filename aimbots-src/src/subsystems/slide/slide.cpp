#include "slide.hpp"
#include <algorithm>

namespace src::Slide {

static constexpr SmoothPIDConfig SLIDE_X_POSITION_PID_CONFIG_temp = {
    .kp = 150000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

SlideSubsystem::SlideSubsystem(Drivers* drivers)
    : Subsystem(drivers),
    motors {
        DJIMotor(drivers, SLIDE_X_MOTOR_ID, SLIDE_BUS, SLIDE_X_MOTOR_DIRECTION, "slide x motor"),
        DJIMotor(drivers, SLIDE_Z_MOTOR_ID, SLIDE_BUS, SLIDE_Z_MOTOR_DIRECTION, "slide z motor"),
    },
    motorPIDs {
        SmoothPID(SLIDE_X_POSITION_PID_CONFIG_temp),
        SmoothPID(SLIDE_Z_POSITION_PID_CONFIG)
    }
{
}

void SlideSubsystem::initialize()
{
    ForAllSlideMotors(&DJIMotor::initialize);
}

bool isXOnline = false;
float xOutput = 0.0f;
float xTarget = 0.0f;

void SlideSubsystem::refresh() 
{
    ForAllSlideMotors(&SlideSubsystem::refreshDesiredOutput);
    isXOnline = motors[X].isMotorOnline();
    xOutput = desiredOutputs[X];
    xTarget = targetPosesMeters[X];
}

void SlideSubsystem::refreshDesiredOutput(MotorIndex motorIdx)
{
    motors[motorIdx].setDesiredOutput(desiredOutputs[motorIdx]);
}

void SlideSubsystem::updateAllPIDs()
{
    ForAllSlideMotors(&SlideSubsystem::updateMotorPositionPID);
}

void SlideSubsystem::updateMotorPositionPID(MotorIndex motorIdx) {
    float positionRevs = motors[motorIdx].getEncoderUnwrapped() / DJIMotor::ENC_RESOLUTION;
    float positionMeters = positionRevs * SLIDE_METERS_PER_REVS_RATIOS[motorIdx];
    float err = targetPosesMeters[motorIdx] - positionMeters;
    float errDerivative = motors[motorIdx].getShaftRPM();

    motorPIDs[motorIdx].runControllerDerivateError(err);

    desiredOutputs[motorIdx] = static_cast<int32_t>(motorPIDs[motorIdx].getOutput());
}

void SlideSubsystem::setTargetPositionMeters(float x, float z)
{
    targetPosesMeters[X] = std::clamp(x, 0.0f, SLIDE_MAX_POSITIONS_METERS[X]);
    targetPosesMeters[Z] = std::clamp(z, 0.0f, SLIDE_MAX_POSITIONS_METERS[Z]);
}

float SlideSubsystem::getTargetXMeters() const
{
    return targetPosesMeters[X];
}

float SlideSubsystem::getTargetZMeters() const
{
    return targetPosesMeters[Z];
}

}; // namespace src::Slider