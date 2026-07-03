#include "gimbal_patrol_command.hpp"
#include "utils/tools/common_types.hpp"



#ifdef ALL_SENTRIES

namespace src::Gimbal {

GimbalPatrolCommand::GimbalPatrolCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbalSubsystem,
    GimbalFieldRelativeController* gimbalController,
    GimbalPatrolConfig patrolConfig,
    src::Chassis::ChassisMatchStates& chassisState)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      patrolConfig(patrolConfig),
      chassisState(chassisState),
      safePatrolCoordinates(
          {modm::Location2D<float>({3.0f, 7.0f}, 0.0f),
           modm::Location2D<float>({4.7f, 7.0f}, 0.0f),
           modm::Location2D<float>({4.7f, 5.0f}, 0.0f),
           modm::Location2D<float>({4.5f, 1.5f}, 0.0f)}),
      safePatrolCoordinateTimes({1500, 1000, 600, 1500}),
      capPatrolCoordinates(
          {modm::Location2D<float>({5.5f, 4.0f}, 0.0f),
           modm::Location2D<float>({6.0f, 7.0f}, 0.0f),
           modm::Location2D<float>({7.5f, 4.0f}, 0.0f),
           modm::Location2D<float>({6.0f, 1.5f}, 0.0f)}),
      capPatrolCoordinateTimes({800, 1000, 800, 1000}),
      aggroPatrolCoordinates(
          {modm::Location2D<float>({11.0f, 1.0f}, 0.0f),
           modm::Location2D<float>({10.5f, 1.3f}, 0.0f),
           modm::Location2D<float>({9.5f, 3.0f}, 0.0f),
           modm::Location2D<float>({9.0f, 4.0f}, 0.0f)}),
      aggroPatrolCoordinateTimes({1000, 1500, 600, 1000})
//
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));

    // signed short-way span between the two sector angles; wrapping keeps it in [-pi, pi)
    sectorScanSpan = tap::algorithms::WrappedFloat(patrolConfig.sectorScanStartAngle, -M_PI, M_PI)
                         .minDifference(patrolConfig.sectorScanEndAngle);
    modm_assert(
        sectorScanSpan != 0.0f && fabsf(sectorScanSpan) < M_PI,
        "GimbalPatrolCommand",
        "sector scan span must be nonzero and less than 180 degrees");
}

float currPatrolCoordinateXDisplay = 0.0f;
float currPatrolCoordinateYDisplay = 0.0f;
float currPatrolCoordinateTimeDisplay = 0.0f;

void GimbalPatrolCommand::initialize() {
    commandStartTime = tap::arch::clock::getTimeMilliseconds();
    lastExecuteTime = commandStartTime;
    // start the patrol spin from the gimbal's current field-relative heading. When patrol takes
    // over from the chase command, this is the last position chase left the gimbal at -- so the
    // yaw continues spinning from there instead of snapping back to field-relative zero.
    yawPatrolStartAngle = drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue();

    inSectorScan = false;
    sectorSweepOffset = 0.0f;
    sectorSweepDirection = 1.0f;

    if (patrolFirstStartTime == 0) {
        patrolFirstStartTime = commandStartTime;
    }
}

float targetPatrolYawAxisAngleDisplay = 0.0f;
float targetPatrolPitchAxisAngleDisplay = 0.0f;

float patrolIndexDisplay = 0;
bool patrolTimerDisplay = false;
float patrolTimingDisplay = 0;
bool patrolRunningDisplay = false;

void GimbalPatrolCommand::execute() {
    float targetYawAxisAngle = 0.0f;
    float targetPitchAxisAngle = 0.0f;

    patrolIndexDisplay = patrolCoordinateIndex;

    // patrolTimingDisplay = patrolCoordinateTimes[patrolCoordinateIndex];

    patrolTimerDisplay = patrolTimer.isExpired();
    patrolRunningDisplay = patrolTimer.isStopped();

    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    float dtSeconds = (currentTime - lastExecuteTime) / 1000.0f;
    lastExecuteTime = currentTime;

    updateScanMode();

    // targetYawAxisAngle = getFieldRelativeYawPatrolAngle(AngleUnit::Radians);
    if (inSectorScan) {
        targetYawAxisAngle = getSectorScanYawPatrolAngle(AngleUnit::Radians, dtSeconds);
    } else {
        targetYawAxisAngle = getConstantVelocityYawPatrolAngle(AngleUnit::Radians);
    }
    targetPatrolYawAxisAngleDisplay = targetYawAxisAngle;
    targetPitchAxisAngle = getSinusoidalPitchPatrolAngle(AngleUnit::Radians);
    targetPatrolPitchAxisAngleDisplay = targetPitchAxisAngle;

    controller->setTargetYaw(AngleUnit::Radians, targetYawAxisAngle);
    controller->setTargetPitch(AngleUnit::Radians, targetPitchAxisAngle);

    controller->runYawController();
    controller->runPitchController();  // That parameter is unused and should be unecessary, but complier is strange
}

bool GimbalPatrolCommand::isReady() { return true; }

bool GimbalPatrolCommand::isFinished() const { return false; }

void GimbalPatrolCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

bool inSectorScanDisplay = false;
float sectorScanElapsedMinutesDisplay = 0.0f;
float sectorSweepTargetDisplay = 0.0f;

void GimbalPatrolCommand::updateScanMode() {
    uint32_t elapsedSincePatrolFirstStart = tap::arch::clock::getTimeMilliseconds() - patrolFirstStartTime;
    sectorScanElapsedMinutesDisplay = elapsedSincePatrolFirstStart / 60000.0f;

    bool shouldSectorScan = elapsedSincePatrolFirstStart >= patrolConfig.sectorScanSwitchTimeMillis;
    if (shouldSectorScan == inSectorScan) {
        return;
    }

    if (shouldSectorScan) {
        // entering the sector sweep: project the current heading onto the sector so the sweep
        // continues from wherever the gimbal is pointing instead of snapping to the start angle
        float currentYaw =
            drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue();
        float offsetFromStart = tap::algorithms::WrappedFloat(patrolConfig.sectorScanStartAngle, -M_PI, M_PI)
                                    .minDifference(currentYaw) *
                                sgn(sectorScanSpan);
        sectorSweepOffset = tap::algorithms::limitVal(offsetFromStart, 0.0f, fabsf(sectorScanSpan));
        sectorSweepDirection = 1.0f;
    } else {
        // back to the 360 spin: re-latch the spin origin at the current heading
        commandStartTime = tap::arch::clock::getTimeMilliseconds();
        yawPatrolStartAngle =
            drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue();
    }

    inSectorScan = shouldSectorScan;
    inSectorScanDisplay = inSectorScan;
}

float GimbalPatrolCommand::getSectorScanYawPatrolAngle(AngleUnit unit, float dtSeconds) {
    float sweepSpeed = modm::toRadian(patrolConfig.yawPatrolAngularVelocityDegreesPerSec);
    float spanMagnitude = fabsf(sectorScanSpan);

    sectorSweepOffset += sectorSweepDirection * sweepSpeed * dtSeconds;
    if (sectorSweepOffset >= spanMagnitude) {
        sectorSweepOffset = spanMagnitude;
        sectorSweepDirection = -1.0f;
    } else if (sectorSweepOffset <= 0.0f) {
        sectorSweepOffset = 0.0f;
        sectorSweepDirection = 1.0f;
    }

    float angleRadians = patrolConfig.sectorScanStartAngle + sgn(sectorScanSpan) * sectorSweepOffset;
    sectorSweepTargetDisplay = angleRadians;

    return unit == AngleUnit::Radians ? angleRadians : modm::toDegree(angleRadians);
}

bool yawPositionPIDErrorDisplay = false;
float yawPositionPIDDerivativeDisplay = 0.0f;

void GimbalPatrolCommand::updateYawPatrolTarget() {
    yawPositionPIDErrorDisplay = controller->allOnlineYawControllersSettled(modm::toRadian(15.0f), 0);

    if (controller->allOnlineYawControllersSettled(modm::toRadian(15.0f), 500)) {
        if (patrolTimer.execute()) {
            patrolCoordinateIndex += patrolCoordinateIncrement;
            // if we're settled at the target angle, and the timer expires for the first time, bounce the patrol coordinate
            // index
            if (patrolCoordinateIndex == 0 || (patrolCoordinateIndex == NUM_PATROL_LOCATIONS - 1)) {
                patrolCoordinateIncrement *= -1;
            }
        } else if (patrolTimer.isExpired() || patrolTimer.isStopped()) {
            // if we're settled at the target angle, and the timer has already expired or hasn't ever been started, start the
            // timer
            if (chassisState == src::Chassis::ChassisMatchStates::CAPTURE) {
                patrolTimer.restart(static_cast<uint32_t>(capPatrolCoordinateTimes[patrolCoordinateIndex]));
            } else if (chassisState == src::Chassis::ChassisMatchStates::AGGRO) {
                patrolTimer.restart(static_cast<uint32_t>(aggroPatrolCoordinateTimes[patrolCoordinateIndex]));
            } else {
                patrolTimer.restart(static_cast<uint32_t>(safePatrolCoordinateTimes[patrolCoordinateIndex]));
            }
        }
    }
}

float xy_angleDisplay = 0.0f;

// function assumes gimbal yaw is at 0 degrees (positive x axis)
float GimbalPatrolCommand::getFieldRelativeYawPatrolAngle(AngleUnit unit) {
    this->updateYawPatrolTarget();
    // needs to target XY positions on the field from patrolCoordinates.getRow(patrolCoordinateIndex)
    // convert that to an angle relative to the field's positive x axis
    // currPatrolCoordinateXDisplay = patrolCoordinates[patrolCoordinateIndex].getX();
    // currPatrolCoordinateYDisplay = patrolCoordinates[patrolCoordinateIndex].getY();
    // currPatrolCoordinateTimeDisplay = patrolCoordinateTimes[patrolCoordinateIndex];

    // float zAngle = src::Utils::MatrixHelper::getZAngleBetweenLocations(
    //     drivers->kinematicInformant.getRobotLocation2D(),
    //     patrolCoordinates[patrolCoordinateIndex],
    //     AngleUnit::Radians);
    float zAngle = 0.0f;

    if (chassisState == src::Chassis::ChassisMatchStates::CAPTURE) {
        zAngle = src::Utils::MatrixHelper::getZAngleBetweenLocations(
            drivers->kinematicInformant.getRobotLocation2D(),
            capPatrolCoordinates[patrolCoordinateIndex],
            AngleUnit::Radians);
    } else if (chassisState == src::Chassis::ChassisMatchStates::AGGRO) {
        zAngle = src::Utils::MatrixHelper::getZAngleBetweenLocations(
            drivers->kinematicInformant.getRobotLocation2D(),
            aggroPatrolCoordinates[patrolCoordinateIndex],
            AngleUnit::Radians);
    } else {
        zAngle = src::Utils::MatrixHelper::getZAngleBetweenLocations(
            drivers->kinematicInformant.getRobotLocation2D(),
            safePatrolCoordinates[patrolCoordinateIndex],
            AngleUnit::Radians);
    }

    xy_angleDisplay = modm::toDegree(zAngle);

    if (unit == AngleUnit::Degrees) {
        zAngle = modm::toDegree(zAngle);
    }
    return zAngle;
}

};  // namespace src::Gimbal

#endif
