#include "gimbal_chassis_relative_controller.hpp"

#include <utils/robot_specific_inc.hpp>

namespace src::Gimbal {

GimbalChassisRelativeController::GimbalChassisRelativeController(GimbalSubsystem* gimbalSubsystem)
    : gimbal(gimbalSubsystem),
      yawPositionPID(YAW_POSITION_PID_CONFIG),
      pitchPositionPID(PITCH_POSITION_PID_CONFIG) {}

void GimbalChassisRelativeController::initialize() {
    yawPositionPID.pid.reset();
    pitchPositionPID.pid.reset();
}

float yawPositionPIDOutputDisplay = 0.0f;

void GimbalChassisRelativeController::runYawController(AngleUnit unit, float desiredYawAngle) {
    gimbal->setTargetYawAngle(unit, desiredYawAngle);

    float positionControllerError = modm::toDegree(gimbal->getCurrentYawAngleAsContiguousFloat().difference(gimbal->getTargetYawAngle(AngleUnit::Radians)));

    float yawPositionPIDOutput = yawPositionPID.runController(positionControllerError, gimbal->getYawMotorRPM());
    yawPositionPIDOutputDisplay = yawPositionPIDOutput;

    gimbal->setYawMotorOutput(yawPositionPIDOutput);
}

void GimbalChassisRelativeController::runPitchController(AngleUnit unit, float desiredPitchAngle) {
    gimbal->setTargetPitchAngle(unit, desiredPitchAngle);

    float positionControllerError = modm::toDegree(gimbal->getCurrentPitchAngleAsContiguousFloat().difference(gimbal->getTargetPitchAngle(AngleUnit::Radians)));

    float pitchPositionPIDOutput = pitchPositionPID.runController(positionControllerError, gimbal->getPitchMotorRPM());

    gimbal->setPitchMotorOutput(pitchPositionPIDOutput);
}

bool GimbalChassisRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal