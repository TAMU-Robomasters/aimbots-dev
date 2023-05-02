#include "gimbal_chassis_relative_controller.hpp"

namespace src::Gimbal {

GimbalChassisRelativeController::GimbalChassisRelativeController(GimbalSubsystem* gimbalSubsystem)
    : gimbal(gimbalSubsystem)  //
{
    BuildPositionPIDs();
}

void GimbalChassisRelativeController::initialize() {
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        yawPositionPIDs[i]->pid.reset();
    }
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        pitchPositionPIDs[i]->pid.reset();
    }
}

float positionPIDOutputDisplay = 0.0f;

void GimbalChassisRelativeController::runYawController(AngleUnit unit, float targetYawAxisAngle, bool vision) {
    UNUSED(vision);
    gimbal->setTargetYawAxisAngle(unit, targetYawAxisAngle);

    float positionPIDOutput = 0.0f;
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
        positionPIDOutput = yawPositionPIDs[i]->runController(
            gimbal->getYawMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getYawMotorRPM(i)));

        positionPIDOutputDisplay = positionPIDOutput;

        gimbal->setDesiredYawMotorOutput(i, positionPIDOutput);
    }
}

void GimbalChassisRelativeController::runPitchController(AngleUnit unit, float targetPitchAxisAngle, bool vision) {
    UNUSED(vision);

    gimbal->setTargetPitchAxisAngle(unit, targetPitchAxisAngle);

    float positionPIDOutput = 0.0f;
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
        positionPIDOutput = pitchPositionPIDs[i]->runController(
            gimbal->getPitchMotorSetpointError(i, AngleUnit::Radians),
            RPM_TO_RADPS(gimbal->getPitchMotorRPM(i)));

        gimbal->setDesiredPitchMotorOutput(i, positionPIDOutput);
    }
}

bool GimbalChassisRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal