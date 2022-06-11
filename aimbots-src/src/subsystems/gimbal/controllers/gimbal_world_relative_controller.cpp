#include "gimbal_world_relative_controller.hpp"

inline static float transChassisToWorldSpace(float chassisRelInitAngle,
                                             float currChassisRelAngle,
                                             float chassisAngle) {
    return chassisAngle + currChassisRelAngle - chassisRelInitAngle;
}

inline static float transWorldToChassisSpace(float chassisRelInitAngle,
                                             float currChassisRelAngle,
                                             float worldAngle) {
    return worldAngle - currChassisRelAngle + chassisRelInitAngle;
}

static void updateWorldRelativeYawTarget(float targetYaw,
                                         float chassisRelativeInitialIMUAngle,
                                         float chassisRelativeIMUAngle,
                                         float& worldRelativeYaw,
                                         src::Gimbal::GimbalSubsystem* gimbal) {
    worldRelativeYaw = targetYaw;

    // TODO: We might want to limit the yaw angle here. We dont need to
    //       rn, so I'll ignore it, but it's something to consider.
    gimbal->setTargetYawAngle(AngleUnit::Radians, transWorldToChassisSpace(
                                                      chassisRelativeInitialIMUAngle,
                                                      chassisRelativeIMUAngle,
                                                      worldRelativeYaw));
}

namespace src::Gimbal {

GimbalWorldRelativeController::GimbalWorldRelativeController(src::Drivers* drivers,
                                                             GimbalSubsystem* gimbal)
    : drivers(drivers), gimbal(gimbal), yawPositionPID(YAW_POSITION_PID_CONFIG), pitchPositionPID(PITCH_POSITION_PID_CONFIG) {}

void GimbalWorldRelativeController::initialize() {
    revolutions = 0;
    previousYaw = getBMIYawUnwrapped();

    chassisRelativeInitialIMUAngle = previousYaw;
    worldRelativeYawTarget = gimbal->getTargetYawAngle(AngleUnit::Radians);
}

void GimbalWorldRelativeController::runYawController(AngleUnit unit, float targetYawAngle) {
    updateRevolutionCounter();

    float chassisRelativeIMUYaw = getBMIYawUnwrapped();

    updateWorldRelativeYawTarget(
        (unit == AngleUnit::Degrees) ? modm::toRadian(targetYawAngle) : targetYawAngle,
        chassisRelativeInitialIMUAngle,
        chassisRelativeIMUYaw,
        worldRelativeYawTarget,
        gimbal);

    float worldSpaceYaw = transChassisToWorldSpace(
        chassisRelativeInitialIMUAngle,
        chassisRelativeIMUYaw,
        gimbal->getUnwrappedYawAngleMeasurement());

    float positionControllerError = ContiguousFloat(worldSpaceYaw, 0, M_TWOPI).difference(worldRelativeYawTarget);
    float yawPositionPIDOutput = yawPositionPID.runController(positionControllerError, gimbal->getYawMotorRPM() + modm::toRadian(drivers->bmi088.getGz()));

    gimbal->setYawMotorOutput(yawPositionPIDOutput);
}

void GimbalWorldRelativeController::runPitchController(AngleUnit unit, float targetPitchAngle) {
    gimbal->setTargetPitchAngle(unit, targetPitchAngle);

    // This gets converted to degrees so that we get a higher error. ig
    // we could also just boost our constants, but this takes minimal
    // calculation and seems simpler. subject to change I suppose...
    float positionControllerError =
        modm::toDegree(
            gimbal->getCurrentPitchAngleAsContiguousFloat()
                .difference(gimbal->getTargetPitchAngle(AngleUnit::Radians)));

    float pitchPositionPIDOutput = pitchPositionPID.runController(positionControllerError, gimbal->getPitchMotorRPM());

    gimbal->setPitchMotorOutput(pitchPositionPIDOutput);
}

bool GimbalWorldRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal