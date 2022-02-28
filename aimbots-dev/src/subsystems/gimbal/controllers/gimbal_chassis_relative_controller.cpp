#include "gimbal_chassis_relative_controller.hpp"

#include <subsystems/gimbal/controllers/gimbal_gravity_helper.hpp>

namespace src::Gimbal {

GimbalChassisRelativeController::GimbalChassisRelativeController(GimbalSubsystem* gimbalSubsystem,
                                                                 tap::algorithms::SmoothPidConfig const& yawPIDConfig,
                                                                 tap::algorithms::SmoothPidConfig const& pitchPIDConfig)
    : gimbal(gimbalSubsystem),
      yawPID(yawPIDConfig),
      pitchPID(pitchPIDConfig) {}

void GimbalChassisRelativeController::initialize() {
    yawPID.reset();
    pitchPID.reset();
}

void GimbalChassisRelativeController::runYawController(float deltaTime, float desiredYawAngle) {
    gimbal->setTargetYawAngleInRadians(desiredYawAngle);

    float positionControllerError = gimbal->getContiguousCurrentYawAngle().difference(gimbal->getTargetYawAngleInRadians());

    float yawPIDOutput = yawPID.runController(positionControllerError,
                                              gimbal->getYawMotorVelocity(),
                                              deltaTime);

    gimbal->setYawMotorOutput(yawPIDOutput);
}

void GimbalChassisRelativeController::runPitchController(float deltaTime, float desiredPitchAngle) {
    gimbal->setTargetPitchAngleInRadians(desiredPitchAngle);

    float positionControllerError = gimbal->getContiguousCurrentPitchAngle().difference(gimbal->getTargetPitchAngleInRadians());

    float pitchPIDOutput = yawPID.runController(positionControllerError,
                                                gimbal->getPitchMotorVelocity(),
                                                deltaTime);

    pitchPIDOutput += Calculations::computeGravitationalForceOffset(
        GIMBAL_CENTER_OF_GRAVITY_OFFSET_X,
        GIMBAL_CENTER_OF_GRAVITY_OFFSET_Z,
        -gimbal->getCurrentPitchAngleFromCenterInRadians(),
        GRAVITY_COMPENSATION_MAX);

    gimbal->setPitchMotorOutput(pitchPIDOutput);
}

bool GimbalChassisRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal