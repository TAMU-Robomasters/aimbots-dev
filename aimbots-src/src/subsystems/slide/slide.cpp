#include "slide.hpp"

namespace src::Slide {

SlideSubsystem::SlideSubsystem(Drivers* drivers)
    : Subsystem(drivers),
    targetX(0), targetY(0), targetZ(0),
    desiredXRPM(0), desiredYRPM(0), desiredZRPM(0),
    xMotor(drivers, SLIDE_X_MOTOR_ID, SLIDE_BUS, SLIDE_X_MOTOR_DIRECTION, "slide x motor"),
    yMotor(drivers, SLIDE_Y_MOTOR_ID, SLIDE_BUS, SLIDE_Y_MOTOR_DIRECTION, "slide y motor"),
    zMotor(drivers, SLIDE_Z_MOTOR_ID, SLIDE_BUS, SLIDE_Z_MOTOR_DIRECTION, "slide z motor"),
    xMotorPID(SLIDE_X_POSITION_PID_CONFIG),
    yMotorPID(SLIDE_Y_POSITION_PID_CONFIG),
    zMotorPID(SLIDE_Z_POSITION_PID_CONFIG)
{
}

void SlideSubsystem::initialize() 
{
    xMotor.initialize();
    yMotor.initialize();
    zMotor.initialize();

    // paranoia
    xMotor.setDesiredOutput(0);
    yMotor.setDesiredOutput(0);
    zMotor.setDesiredOutput(0);
}

void SlideSubsystem::refresh() 
{
    setDesiredOutput();
}

void SlideSubsystem::updateSlidePositionPID()
{
    float xPosition = xMotor.getEncoderUnwrapped()*SLIDE_X_M_PER_UNWRAPPED_RATIO;
    float yPosition = yMotor.getEncoderUnwrapped()*SLIDE_Y_M_PER_UNWRAPPED_RATIO;
    float zPosition = zMotor.getEncoderUnwrapped()*SLIDE_Z_M_PER_UNWRAPPED_RATIO;

    float xErr = targetX - xPosition;
    xMotorPID.runControllerDerivateError(xErr);
    desiredXRPM = xMotorPID.getOutput();

    float yErr = targetY - yPosition;
    yMotorPID.runControllerDerivateError(yErr);
    desiredYRPM = yMotorPID.getOutput();

    float zErr = targetZ - zPosition;
    zMotorPID.runControllerDerivateError(zErr);
    desiredZRPM = zMotorPID.getOutput();
}

void SlideSubsystem::setDesiredOutput()
{
    xMotor.setDesiredOutput(static_cast<int32_t>(desiredXRPM));
    yMotor.setDesiredOutput(static_cast<int32_t>(desiredYRPM));
    zMotor.setDesiredOutput(static_cast<int32_t>(desiredZRPM));
}

void SlideSubsystem::setTargetPosition(float x, float y, float z)
{
    targetX = x;
    targetY = y;
    targetZ = z;
}

}; // namespace src::Slider