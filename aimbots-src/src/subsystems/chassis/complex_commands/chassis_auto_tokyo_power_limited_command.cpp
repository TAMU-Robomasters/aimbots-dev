#include "chassis_auto_tokyo_power_limited_command.hpp"

#ifdef CHASSIS_COMPATIBLE

#include <algorithm>

namespace src::Chassis {

// variables for ozone
bool autoTokyoPowerLimitActiveDisplay = false;
bool autoTokyoPowerSensorFreshDisplay = false;
float autoTokyoRequestedMaxWheelSpeedDisplay = 0.0f;
float autoTokyoMaxWheelSpeedDisplay = 0.0f;
float autoTokyoMeasuredPowerWDisplay = 0.0f;
float autoTokyoPowerScaleDisplay = 1.0f;

ChassisAutoTokyoPowerLimitedCommand::ChassisAutoTokyoPowerLimitedCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    Gimbal::GimbalSubsystem* gimbal,
    const TokyoConfig& tokyoConfig,
    bool randomizeSpinRate,
    const SpinRandomizerConfig& randomizerConfig,
    float maxWheelSpeed)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      chassis(chassis),
      tokyoMasterCommand(
          drivers,
          chassis,
          gimbal,
          tokyoConfig,
          0,  // spinDirectionOverride (0 = random)
          randomizeSpinRate,
          randomizerConfig,
          ChassisTokyoMasterMode::NORMAL,
          0.0f,  // joystick2OverrideVelocity (ignored in auto)
          maxWheelSpeed,
          true),  // isAuto: translation comes from the Jetson's nav2 velocity command
      maxWheelSpeed(maxWheelSpeed),
      powerLimitedMaxWheelSpeed(maxWheelSpeed) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
    comprisedCommandScheduler.registerSubsystem(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisAutoTokyoPowerLimitedCommand::initialize() {
    powerLimitedMaxWheelSpeed = maxWheelSpeed;
    tokyoMasterCommand.setMaxWheelSpeed(powerLimitedMaxWheelSpeed);
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoMasterCommand);
}

float ChassisAutoTokyoPowerLimitedCommand::slewToward(
    float current,
    float target,
    float increaseStep,
    float decreaseStep) const {
    if (target > current) {
        return std::min(target, current + increaseStep);
    }
    return std::max(target, current - decreaseStep);
}

float ChassisAutoTokyoPowerLimitedCommand::calculatePowerLimitedMaxWheelSpeed(float requestedMaxWheelSpeed) {
    requestedMaxWheelSpeed = requestedMaxWheelSpeed > 0.0f ? requestedMaxWheelSpeed : maxWheelSpeed;

    if (powerLimitedMaxWheelSpeed <= 0.0f) {
        powerLimitedMaxWheelSpeed = requestedMaxWheelSpeed;
    }

    // If the requested ceiling drops, do not keep the old higher ceiling.
    if (powerLimitedMaxWheelSpeed > requestedMaxWheelSpeed) {
        powerLimitedMaxWheelSpeed = requestedMaxWheelSpeed;
    }

    const bool powerSensorFresh = drivers->espPowerSensor.hasFreshPacket(POWER_SENSOR_FRESH_TIMEOUT_MS);
    const float measuredPowerW = powerSensorFresh ? drivers->espPowerSensor.getPower() : 0.0f;

    float powerScale = 1.0f;
    float targetMaxWheelSpeed = requestedMaxWheelSpeed;
    bool powerLimitActive = false;

    if (POWER_LIMITING_ENABLED && powerSensorFresh && measuredPowerW > TARGET_CHASSIS_POWER_W) {
        // Applying this to the current rpm ceiling makes the new target rpm ceiling fall quickly during power/acceleration spikes
        powerScale = limitVal<float>(
            (TARGET_CHASSIS_POWER_W / measuredPowerW) * POWER_LIMIT_SCALAR,
            POWER_LIMIT_MIN_SCALE,
            1.0f);

        targetMaxWheelSpeed = powerLimitedMaxWheelSpeed * powerScale;
        targetMaxWheelSpeed = limitVal<float>(
            targetMaxWheelSpeed,
            requestedMaxWheelSpeed * POWER_LIMIT_MIN_SCALE,
            requestedMaxWheelSpeed);
        powerLimitActive = true;
    }

    powerLimitedMaxWheelSpeed = slewToward(
        powerLimitedMaxWheelSpeed,
        targetMaxWheelSpeed,
        POWER_LIMIT_RPM_RECOVERY_PER_ITER,
        POWER_LIMIT_RPM_DECREASE_PER_ITER);

    powerLimitedMaxWheelSpeed = limitVal<float>(
        powerLimitedMaxWheelSpeed,
        requestedMaxWheelSpeed * POWER_LIMIT_MIN_SCALE,
        requestedMaxWheelSpeed);

    autoTokyoPowerSensorFreshDisplay = powerSensorFresh;
    autoTokyoMeasuredPowerWDisplay = measuredPowerW;
    autoTokyoPowerScaleDisplay = powerScale;
    autoTokyoPowerLimitActiveDisplay = powerLimitActive;
    autoTokyoRequestedMaxWheelSpeedDisplay = requestedMaxWheelSpeed;

    return powerLimitedMaxWheelSpeed;
}

void ChassisAutoTokyoPowerLimitedCommand::execute() {
    const float limitedMaxWheelSpeed = calculatePowerLimitedMaxWheelSpeed(maxWheelSpeed);

    tokyoMasterCommand.setMaxWheelSpeed(limitedMaxWheelSpeed);
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoMasterCommand);

    autoTokyoMaxWheelSpeedDisplay = limitedMaxWheelSpeed;

    comprisedCommandScheduler.run();
}

void ChassisAutoTokyoPowerLimitedCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoMasterCommand, interrupted);
    chassis->setTokyoDrift(false);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisAutoTokyoPowerLimitedCommand::isReady() { return true; }

bool ChassisAutoTokyoPowerLimitedCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif  // #ifdef CHASSIS_COMPATIBLE
