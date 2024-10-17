#include "slide.hpp"

#include <algorithm>

#ifdef SLIDE_COMPATIBLE

namespace src::Slide {

SlideSubsystem::SlideSubsystem(Drivers* drivers)
    : Subsystem(drivers),
      motors{
          DJIMotor(drivers, SLIDE_X_MOTOR_ID, SLIDE_BUS, SLIDE_X_MOTOR_DIRECTION, "slide x motor"),
          DJIMotor(drivers, SLIDE_Z_MOTOR_ID, SLIDE_BUS, SLIDE_Z_MOTOR_DIRECTION, "slide z motor"),
      },
      motorPIDs{SmoothPID(SLIDE_X_POSITION_PID_CONFIG), SmoothPID(SLIDE_X_POSITION_PID_CONFIG)} {}

void SlideSubsystem::initialize() { ForAllSlideMotors(&DJIMotor::initialize); }

float xMotorEncDisplay = 0.0f;
float zMotorEncDisplay = 0.0f;

void SlideSubsystem::refresh() {
    xMotorEncDisplay = motors[X].getEncoderUnwrapped() / DJIMotor::ENC_RESOLUTION;
    zMotorEncDisplay = motors[Z].getEncoderUnwrapped() / DJIMotor::ENC_RESOLUTION;
    ForAllSlideMotors(&SlideSubsystem::refreshDesiredOutput);
}

void SlideSubsystem::refreshDesiredOutput(MotorIndex motorIdx) {
    motors[motorIdx].setDesiredOutput(desiredOutputs[motorIdx]);
}

void SlideSubsystem::updateAllPIDs() { ForAllSlideMotors(&SlideSubsystem::updateMotorPositionPID); }

void SlideSubsystem::updateMotorPositionPID(MotorIndex motorIdx) {
    float positionRevs = motors[motorIdx].getEncoderUnwrapped() / DJIMotor::ENC_RESOLUTION;
    float positionMeters = positionRevs * SLIDE_METERS_PER_REVS_RATIOS[motorIdx];
    float err = targetPosesMeters[motorIdx] - positionMeters;

    motorPIDs[motorIdx].runControllerDerivateError(err);

    desiredOutputs[motorIdx] = static_cast<int32_t>(motorPIDs[motorIdx].getOutput());
}

void SlideSubsystem::setTargetPositionMeters(float x, float z) {
    targetPosesMeters[X] = std::clamp(x, -1000.0f, SLIDE_MAX_POSITIONS_METERS[X]);  // Disabling clamping because dart bad
    targetPosesMeters[Z] = std::clamp(z, -1000.0f, SLIDE_MAX_POSITIONS_METERS[Z]);
}

float SlideSubsystem::getTargetXMeters() const { return targetPosesMeters[X]; }

float SlideSubsystem::getTargetZMeters() const { return targetPosesMeters[Z]; }

};  // namespace src::Slide

#endif