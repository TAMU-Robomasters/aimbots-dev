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

void GimbalChassisRelativeController::runYawController(AngleUnit unit, float desiredPitchAngle) {
    UNUSED(unit);
    UNUSED(desiredPitchAngle);

    // NOTE: If you are finding that the gimbal is lagging behind the
    //       chassis a fair bit and you came here to find out why,
    //       you should be able to just tune the PID constants until
    //       that behavior stops.

    // This just locks to the chassis's forward direction. If it isn't,
    // then you simply need to adjust YAW_START_ANGLE until it does.
    // The easiest way to do that would probably be to use a watch
    // in the debugger and find what angle, IN DEGREES, describes the
    // forward direction.
    gimbal->setTargetYawAngle(AngleUnit::Degrees, YAW_START_ANGLE);

    // This gets converted to degrees so that we get a higher error. ig
    // we could also just boost our constants, but this takes minimal
    // calculation and seems simpler. subject to change I suppose...
    float positionControllerError =
        modm::toDegree(
            gimbal->getCurrentYawAngleAsContiguousFloat()
                   .difference(gimbal->getTargetYawAngle(AngleUnit::Radians))
        );

    float yawPositionPIDOutput = yawPositionPID.runController(positionControllerError, gimbal->getYawMotorRPM());

    gimbal->setYawMotorOutput(yawPositionPIDOutput);
}

void GimbalChassisRelativeController::runPitchController(AngleUnit unit, float desiredPitchAngle) {
    gimbal->setTargetPitchAngle(unit, desiredPitchAngle);

    // This gets converted to degrees so that we get a higher error. ig
    // we could also just boost our constants, but this takes minimal
    // calculation and seems simpler. subject to change I suppose...
    float positionControllerError =
        modm::toDegree(
            gimbal->getCurrentPitchAngleAsContiguousFloat()
                   .difference(gimbal->getTargetPitchAngle(AngleUnit::Radians))
        );

    float pitchPositionPIDOutput = pitchPositionPID.runController(positionControllerError, gimbal->getPitchMotorRPM());

    gimbal->setPitchMotorOutput(pitchPositionPIDOutput);
}

bool GimbalChassisRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal