#include "gimbal_chassis_relative_controller.hpp"

namespace src::Gimbal {

GimbalChassisRelativeController::GimbalChassisRelativeController(GimbalSubsystem* gimbalSubsystem)
    : gimbal(gimbalSubsystem)  //
{
    BuildPIDControllers();
}

void GimbalChassisRelativeController::initialize() {
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        yawPositionPIDs[i]->pid.reset();
    }
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        pitchPositionPIDs[i]->pid.reset();
    }
}

float yawPositionPIDOutputDisplay = 0.0f;
float yawMotorSetpointErrorDisplay = 0.0f;
float pitchMotorSetpointErrorDisplay = 0.0f;

void GimbalChassisRelativeController::runYawController(std::optional<float> velocityLimit) {
    UNUSED(velocityLimit);

    float positionPIDOutput = 0.0f;
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        yawMotorSetpointErrorDisplay = gimbal->getYawMotorSetpointError(i, AngleUnit::Degrees);
        positionPIDOutput = yawPositionPIDs[i]->runController(
            gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getYawMotorRPM(i)));

        yawPositionPIDOutputDisplay = positionPIDOutput;

        gimbal->setDesiredYawMotorOutput(i, positionPIDOutput);
    }
}

void GimbalChassisRelativeController::runPitchController(std::optional<float> velocityLimit) {
    UNUSED(velocityLimit);

    float positionPIDOutput = 0.0f;
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        pitchMotorSetpointErrorDisplay = gimbal->getPitchMotorSetpointError(i, AngleUnit::Degrees);

        positionPIDOutput = pitchPositionPIDs[i]->runController(
            gimbal->getPitchMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)));

        gimbal->setDesiredPitchMotorOutput(i, positionPIDOutput);
    }
}

bool GimbalChassisRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal