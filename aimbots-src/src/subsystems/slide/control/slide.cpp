#include "slide.hpp"

#include <algorithm>

#ifdef SLIDE_COMPATIBLE

namespace src::Slide {

SlideSubsystem::SlideSubsystem(Drivers* drivers)
    : Subsystem(drivers),
{
    BuildSlideMotors();
}

void SlideSubsystem::initialize() { ForAllSlideMotors(&DJIMotor::initialize); }

float xMotorEncDisplay = 0.0f;
float zMotorEncDisplay = 0.0f;

void SlideSubsystem::refresh() {
    xMotorEncDisplay = slideMotors[X].getEncoderUnwrapped() / DJIMotor::ENC_RESOLUTION;
    zMotorEncDisplay = slideMotors[Z].getEncoderUnwrapped() / DJIMotor::ENC_RESOLUTION;
    ForAllSlideMotors(&SlideSubsystem::refreshDesiredOutput);
}

void SlideSubsystem::refreshDesiredOutput(MotorIndex motorIdx) {
    slideMotors[motorIdx].setDesiredOutput(desiredOutputs[motorIdx]);
}

void SlideSubsystem::updateAllPIDs() { ForAllSlideMotors(&SlideSubsystem::updateMotorPositionPID); }

void SlideSubsystem::updateMotorPositionPID(MotorIndex motorIdx) {
    float positionRevs = slideMotors[motorIdx].getEncoderUnwrapped() / DJIMotor::ENC_RESOLUTION;
    float positionMeters = positionRevs * SLIDE_METERS_PER_REVS_RATIOS[motorIdx];
    float err = targetPosesMeters[motorIdx] - positionMeters;

    SLIDE_MOTOR_PIDS[motorIdx].runControllerDerivateError(err);

    desiredOutputs[motorIdx] = static_cast<int32_t>(SLIDE_MOTOR_PIDS[motorIdx].getOutput());
}

void SlideSubsystem::setTargetPositionMeters(float x, float z) {
    targetPosesMeters[X] = std::clamp(x, 0.0f, SLIDE_MAX_POSITIONS_METERS[X]);
    targetPosesMeters[Z] = std::clamp(z, 0.0f, SLIDE_MAX_POSITIONS_METERS[Z]);
}

float SlideSubsystem::getTargetXMeters() const { return targetPosesMeters[X]; }

float SlideSubsystem::getTargetZMeters() const { return targetPosesMeters[Z]; }

};  // namespace src::Slide

#endif