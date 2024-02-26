#include "slide.hpp"
#include <algorithm>

namespace src::Slide {

SlideSubsystem::SlideSubsystem(Drivers* drivers)
    : Subsystem(drivers),
    motors {
        DJIMotor(drivers, SLIDE_X_MOTOR_ID, SLIDE_BUS, SLIDE_X_MOTOR_DIRECTION, "slide x motor"),
        DJIMotor(drivers, SLIDE_Z_MOTOR_ID, SLIDE_BUS, SLIDE_Z_MOTOR_DIRECTION, "slide z motor"),
    },
    motorPIDs {
        SmoothPID(SLIDE_X_POSITION_PID_CONFIG),
        SmoothPID(SLIDE_Z_POSITION_PID_CONFIG)
    }
{
}

void SlideSubsystem::initialize()
{
    ForAllSlideMotors(&DJIMotor::initialize);
}

float xOut_disp = 0;
float zOut_disp = 0;
float xTargetPos_disp = 0;
float zTargetPos_disp = 0;
bool isZOnline = false;

void SlideSubsystem::refresh() 
{
    ForAllSlideMotors(&SlideSubsystem::refreshDesiredOutput);

    isZOnline = motors[Z].isMotorOnline();
    xOut_disp = desiredOutputs[X];
    zOut_disp = desiredOutputs[Z];
    xTargetPos_disp = targetPosesMeters[X];
    zTargetPos_disp = targetPosesMeters[Z];
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

    int32_t output = motorPIDs[motorIdx].runController(err, errDerivative);

    desiredOutputs[motorIdx] = output;
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