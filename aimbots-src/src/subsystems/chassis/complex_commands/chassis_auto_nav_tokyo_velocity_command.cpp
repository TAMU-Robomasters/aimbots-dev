#include "chassis_auto_nav_tokyo_velocity_command.hpp"

#include <cstdlib>

#if defined(CHASSIS_COMPATIBLE) && defined(GIMBAL_COMPATIBLE)

namespace src::Chassis {

ChassisAutoNavTokyoVelocityCommand::ChassisAutoNavTokyoVelocityCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    src::Gimbal::GimbalSubsystem* gimbal,
    const TokyoConfig& tokyoConfig,
    int spinDirectionOverride,
    bool randomizeSpinRate,
    const SpinRandomizerConfig& randomizerConfig)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
      tokyoConfig(tokyoConfig),
      spinDirectionOverride(spinDirectionOverride),
      randomizeSpinRate(randomizeSpinRate),
      randomizerConfig(randomizerConfig)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisAutoNavTokyoVelocityCommand::initialize() {
    // picks a direction to begin rotation (override wins, otherwise random)
    if (spinDirectionOverride != 0) {
        rotationDirection = spinDirectionOverride > 0 ? 1 : -1;
    } else {
        rotationDirection = (rand() - RAND_MAX / 2) < 0 ? 1 : -1;
    }

    rotationSpeedRamp.reset(chassis->getDesiredRotation());

    if (randomizeSpinRate) {
        spinRateModifierTimer.restart(0);
    }
}

// Tunable: scales the Jetson velocity command (m/s) into the chassis input range. Tune in Ozone until
// the robot's real translation speed matches the commanded m/s.
// WARNING: this is probably execiding the power limit. need to be reworked later.
float tokyoNavVelocityToInputMultiplier = 1.0f;

// Debug / watch variables
float tokyoNavVelCmdXDisplay = 0.0f;
float tokyoNavVelCmdYDisplay = 0.0f;
float tokyoNavDesiredXDisplay = 0.0f;
float tokyoNavDesiredYDisplay = 0.0f;
float tokyoNavRampTargetDisplay = 0.0f;
float tokyoNavSpinRateModDisplay = 0.0f;
float tokyoNavTurretFieldYawDisplay = 0.0f;
float tokyoNavYawFromChassisCenterDisplay = 0.0f;
bool tokyoNavJetsonOnlineDisplay = false;

void ChassisAutoNavTokyoVelocityCommand::execute() {
    chassis->setTokyoDrift(true);

    tokyoNavJetsonOnlineDisplay = drivers->cvCommunicator.isJetsonOnline();
    if (!tokyoNavJetsonOnlineDisplay) {
        chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
        return;
    }

    // Field-relative chassis velocity command (m/s) from nav2 on the Jetson.
    modm::Vector2f fieldRelativeVelocity = drivers->cvCommunicator.getDesiredTurretRelativeVelocity();
    tokyoNavVelCmdXDisplay = fieldRelativeVelocity.getX();
    tokyoNavVelCmdYDisplay = fieldRelativeVelocity.getY();

    float desiredX = fieldRelativeVelocity.getX() * tokyoNavVelocityToInputMultiplier;
    float desiredY = fieldRelativeVelocity.getY() * tokyoNavVelocityToInputMultiplier;
    float desiredRotation = 0.0f;  // overwritten by the spin ramp below when the gimbal is online

    if (gimbal->isOnline()) {
        // Maximum speed realistically achievable with the current power limit.
        float maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

#if defined(TARGET_SENTRY_SWERVE) 
        maxWheelSpeed = 4000;
#endif

        desiredX *= tokyoConfig.translationalSpeedMultiplier * maxWheelSpeed;
        desiredY *= tokyoConfig.translationalSpeedMultiplier * maxWheelSpeed;

        const float translationalSpeedThreshold = maxWheelSpeed * tokyoConfig.translationalSpeedMultiplier *
                                                  tokyoConfig.translationThresholdToDecreaseRotationSpeed;

        float rampTarget = maxWheelSpeed * rotationDirection * tokyoConfig.rotationalSpeedFractionOfMax;

        if (randomizeSpinRate) {
            if (spinRateModifierTimer.isExpired() || spinRateModifierTimer.isStopped()) {
                Helper::randomizeSpinCharacteristics(
                    &this->spinRateModifier,
                    &this->spinRateModifierDuration,
                    randomizerConfig);
                spinRateModifierTimer.restart(spinRateModifierDuration);
            }
            rampTarget *= spinRateModifier;
        }

        // reduce rotation speed when translation speed is high
        if (fabsf(desiredX) > translationalSpeedThreshold || fabsf(desiredY) > translationalSpeedThreshold) {
            rampTarget *= tokyoConfig.rotationalSpeedMultiplierWhenTranslating;
        }

        tokyoNavSpinRateModDisplay = spinRateModifier;
        tokyoNavRampTargetDisplay = rampTarget;

        rotationSpeedRamp.setTarget(rampTarget);
        rotationSpeedRamp.update(tokyoConfig.rotationalSpeedIncrement);
        desiredRotation = rotationSpeedRamp.getValue();

        // field-relative -> turret-relative, using the turret's field-relative yaw (from its IMU)
        float turretFieldYaw = drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue();
        tokyoNavTurretFieldYawDisplay = turretFieldYaw;
        rotateVector(&desiredX, &desiredY, -turretFieldYaw);

        // turret-relative -> chassis-relative
        float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        tokyoNavYawFromChassisCenterDisplay = yawAngleFromChassisCenter;
        rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);
    } else {
        Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
    }

    tokyoNavDesiredXDisplay = desiredX;
    tokyoNavDesiredYDisplay = desiredY;
    chassis->setTargetRPMs(desiredX, desiredY, desiredRotation);
}

void ChassisAutoNavTokyoVelocityCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisAutoNavTokyoVelocityCommand::isReady() { return true; }

bool ChassisAutoNavTokyoVelocityCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif  // defined(CHASSIS_COMPATIBLE) && defined(GIMBAL_COMPATIBLE)
