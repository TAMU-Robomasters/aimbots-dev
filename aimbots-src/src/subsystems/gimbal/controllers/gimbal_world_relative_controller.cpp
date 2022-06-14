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

static void updateWorldRelativePitchTarget(float targetPitch,
                                           float chassisRelativeInitialIMUAngle,
                                           float chassisRelativeIMUAngle,
                                           float& worldRelativePitch,
                                           src::Gimbal::GimbalSubsystem* gimbal) {
    worldRelativePitch = targetPitch;

    // TODO: We might want to limit the yaw angle here. We dont need to
    //       rn, so I'll ignore it, but it's something to consider.
    gimbal->setTargetPitchAngle(AngleUnit::Radians, transWorldToChassisSpace(
        chassisRelativeInitialIMUAngle,
        chassisRelativeIMUAngle,
        worldRelativePitch));
}

namespace src::Gimbal {

GimbalWorldRelativeController::GimbalWorldRelativeController(src::Drivers* drivers,
                                                             GimbalSubsystem* gimbal)
    : drivers(drivers),
      gimbal(gimbal),
      yawPositionPID(YAW_POSITION_PID_CONFIG),
      pitchPositionPID(PITCH_POSITION_PID_CONFIG) {}

void GimbalWorldRelativeController::initialize() {
    yawRevolutions = 0;
    previousYaw = getBMIYawUnwrapped();

    chassisRelativeInitialIMUYaw = previousYaw;
    worldSpaceYawTarget = gimbal->getTargetYawAngle(AngleUnit::Radians);
}

void GimbalWorldRelativeController::runYawController(AngleUnit unit, float targetYawAngle) {
    updateYawRevolutionCounter();

    float chassisRelativeIMUYaw = getBMIYawUnwrapped();

    updateWorldRelativeYawTarget(
        (unit == AngleUnit::Degrees) ? modm::toRadian(targetYawAngle) : targetYawAngle,
        chassisRelativeInitialIMUYaw,
        chassisRelativeIMUYaw,
        worldSpaceYawTarget,
        gimbal);

    float worldSpaceYaw = transChassisToWorldSpace(
        chassisRelativeInitialIMUYaw,
        chassisRelativeIMUYaw,
        gimbal->getUnwrappedYawAngleMeasurement());

    float positionControllerError = ContiguousFloat(worldSpaceYaw, 0, M_TWOPI).difference(worldSpaceYawTarget);
    float yawPositionPIDOutput = yawPositionPID.runController(positionControllerError, gimbal->getYawMotorRPM() - modm::toDegree(drivers->fieldRelativeInformant.getGz()));

    gimbal->setYawMotorOutput(yawPositionPIDOutput);
}

void GimbalWorldRelativeController::runPitchController(AngleUnit unit, float targetPitchAngle) {
    gimbal->setTargetPitchAngle(unit, targetPitchAngle);

    float gimbalForwardRelativeIMUPitch = getGimbalForwardRelativeIMUPitch();

    updateWorldRelativePitchTarget(
        (unit == AngleUnit::Degrees) ? modm::toRadian(targetPitchAngle) : targetPitchAngle,
        chassisRelativeInitialIMUPitch,
        gimbalForwardRelativeIMUPitch,
        worldSpacePitchTarget,
        gimbal);

    float worldSpacePitch = transChassisToWorldSpace(
        chassisRelativeInitialIMUPitch,
        gimbalForwardRelativeIMUPitch,
        gimbal->getUnwrappedPitchAngleMeasurement());

    float positionControllerError = ContiguousFloat(worldSpacePitch, 0, M_TWOPI).difference(worldSpacePitchTarget);
    float pitchPositionPIDOutput = pitchPositionPID.runController(positionControllerError, gimbal->getPitchMotorRPM());
}

bool GimbalWorldRelativeController::isOnline() const { return gimbal->isOnline(); }

}  // namespace src::Gimbal