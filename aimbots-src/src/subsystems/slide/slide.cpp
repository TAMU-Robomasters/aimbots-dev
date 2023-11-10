#include "slide.hpp"

namespace src::Slide {

SlideSubsystem::SlideSubsystem(Drivers* drivers)
    : Subsystem(drivers),
    targetX(0), targetY(0), targetZ(0),
    desiredXOutput(0), desiredYOutput(0), desiredZOutput(0),
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
}

void SlideSubsystem::refresh() 
{
    setDesiredOutput();
}

float z_err_display = 0;
float z_position_display = 0;
float z_out_display = 0;

void SlideSubsystem::updateSlidePositionPID()
{
    float xPosition = xMotor.getEncoderUnwrapped()*SLIDE_X_METERS_PER_ENCODER;
    float yPosition = yMotor.getEncoderUnwrapped()*SLIDE_Y_METERS_PER_ENCODER;
    float zPosition = zMotor.getEncoderUnwrapped()*SLIDE_Z_METERS_PER_ENCODER;
    z_position_display = zPosition;

    float xErr = targetX - xPosition;
    xMotorPID.runControllerDerivateError(xErr);
    desiredXOutput = static_cast<int32_t>(xMotorPID.getOutput());

    float yErr = targetY - yPosition;
    yMotorPID.runControllerDerivateError(yErr);
    desiredYOutput = static_cast<int32_t>(yMotorPID.getOutput());

    float zErr = targetZ - zPosition;
    z_err_display = zErr;
    zMotorPID.runControllerDerivateError(zErr);
    desiredZOutput = static_cast<int32_t>(zMotorPID.getOutput());
    z_out_display = desiredZOutput;
}

void SlideSubsystem::setDesiredOutput()
{
    xMotor.setDesiredOutput(desiredXOutput);
    yMotor.setDesiredOutput(desiredYOutput);
    zMotor.setDesiredOutput(desiredZOutput);
}

void SlideSubsystem::setTargetPosition(float x, float y, float z)
{
    targetX = x;
    targetY = y;
    targetZ = z;
}

}; // namespace src::Slider