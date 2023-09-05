#include "utils/robot_specific_inc.hpp"
#ifdef GIMBAL_UNTETHERED

#include <subsystems/chassis/chassis_helper.hpp>

#include "chassis_helper.hpp"
#include "chassis_kansei_command.hpp"

#warning "tokyo compatible"

namespace src::Chassis {

ChassisKanseiCommand::ChassisKanseiCommand(
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

void ChassisKanseiCommand::initialize() {
    // picks a random direction to begin rotation
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

void ChassisKanseiCommand::execute() {
    float desiredX = 0.0f;
    float desiredY = 0.0f;
    float desiredRotation = 0.0f;

    // we overwrite desiredRotation later if tokyo drifting
    Helper::getUserDesiredInput(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

    if (gimbal->isOnline()) {
        float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        // this is wrapped between -PI and PI

        // The maximum speed that we're realistically able to achieve with the current power limit
        const float maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

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

        // reduces rotation speed when translation speed is high
        if (fabsf(desiredX) > translationalSpeedThreshold || fabsf(desiredY) > translationalSpeedThreshold) {
            rampTarget *= tokyoConfig.rotationalSpeedMultiplierWhenTranslating;
        }

        rotationSpeedRamp.setTarget(rampTarget);
        rotationSpeedRamp.update(tokyoConfig.rotationalSpeedIncrement);
        desiredRotation = rotationSpeedRamp.getValue();

        rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);

    } else {
        Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
    }

    chassis->setTargetRPMs(desiredX, desiredY, desiredRotation);
}

void ChassisKanseiCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisKanseiCommand::isReady() { return true; }

bool ChassisKanseiCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif