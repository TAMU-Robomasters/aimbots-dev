#include "slide.hpp"

namespace src::Slide {

SlideSubsystem::SlideSubsystem(Drivers* drivers)
    : Subsystem(drivers),
    motors({
        DJIMotor(drivers, SLIDE_X_MOTOR_ID, SLIDE_BUS, SLIDE_X_MOTOR_DIRECTION, "slide x motor"),
        DJIMotor(drivers, SLIDE_Y_MOTOR_ID, SLIDE_BUS, SLIDE_Y_MOTOR_DIRECTION, "slide y motor"),
        DJIMotor(drivers, SLIDE_Z_MOTOR_ID, SLIDE_BUS, SLIDE_Z_MOTOR_DIRECTION, "slide z motor"),
    }),
    motorPIDs({
        SmoothPID(SLIDE_X_POSITION_PID_CONFIG),
        SmoothPID(SLIDE_Y_POSITION_PID_CONFIG),
        SmoothPID(SLIDE_Z_POSITION_PID_CONFIG)
    })
{
}

void SlideSubsystem::initialize()
{
    ForAllSlideMotors(&DJIMotor::initialize);
}

void SlideSubsystem::refresh() 
{
    setDesiredOutputs();
}

void SlideSubsystem::updateMotorPositionPID(MotorIndex motorIdx) {
    float position = motors[motorIdx].getEncoderUnwrapped() * SLIDE_METERS_PER_ENCODER_RATIOS[motorIdx];
    float err = targetPoses[motorIdx] - position;

    motorPIDs[motorIdx].runControllerDerivateError(err);

    desiredOutputs[motorIdx] = static_cast<int32_t>(motorPIDs[motorIdx].getOutput());
}

void SlideSubsystem::updateSlidePositionPID()
{
    ForAllSlideMotors(&SlideSubsystem::updateMotorPositionPID);
}

void SlideSubsystem::setDesiredOutputs()
{
    for (auto i = 0; i < 3; i++)
        motors[i].setDesiredOutput(desiredOutputs[i]);
}

void SlideSubsystem::setTargetPosition(float x, float y, float z)
{
    targetPoses[X] = x;
    targetPoses[Y] = y;
    targetPoses[Z] = z;
}

}; // namespace src::Slider