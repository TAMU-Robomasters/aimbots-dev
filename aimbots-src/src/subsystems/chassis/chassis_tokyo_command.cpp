#include "utils/robot_specific_inc.hpp"
#ifdef GIMBAL_UNTETHERED

#include <subsystems/chassis/chassis_helper.hpp>

#include "utils/math/random.hpp"

#include "chassis_tokyo_command.hpp"

#warning "tokyo compatible"

namespace src::Chassis {

ChassisTokyoCommand::ChassisTokyoCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    src::Gimbal::GimbalSubsystem* gimbal,
    int spinDirectionOverride,
    bool randomizeSpinRate,
    const ToykoRandomizerConfig& randomizerConfig)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
      spinDirectionOverride(spinDirectionOverride),
      randomizeSpinRate(randomizeSpinRate),
      randomizerConfig(randomizerConfig)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisTokyoCommand::initialize() {
    // picks a random direction to begin rotation
    if (spinDirectionOverride != 0) {
        rotationDirection = spinDirectionOverride > 0 ? 1 : -1;
    } else {
        rotationDirection = (rand() - RAND_MAX / 2) < 0 ? 1 : -1;
    }

    rotationSpeedRamp.reset(chassis->getDesiredRotation());
}

void ChassisTokyoCommand::execute() {
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

        desiredX *= TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed;
        desiredY *= TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed;

        const float translationalSpeedThreshold =
            maxWheelSpeed * TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER * TOKYO_TRANSLATION_THRESHOLD_TO_DECREASE_ROTATION_SPEED;

        float rampTarget = maxWheelSpeed * rotationDirection * TOKYO_ROTATIONAL_SPEED_FRACTION_OF_MAX;

        if (randomizeSpinRate) {
            if (spinRateModifierTimer.execute() || spinRateModifierTimer.isExpired() || spinRateModifierTimer.isStopped()) {
                randomizeSpinCharacteristics();
                spinRateModifierTimer.restart(spinRateModifierDuration);
            } else {
                rampTarget = maxWheelSpeed * rotationDirection * TOKYO_ROTATIONAL_SPEED_FRACTION_OF_MAX * spinRateModifier;
            }
        }

        // reduces rotation speed when translation speed is high
        if (fabsf(desiredX) > translationalSpeedThreshold || fabsf(desiredY) > translationalSpeedThreshold) {
            rampTarget *= TOKYO_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING;
        }

        rotationSpeedRamp.setTarget(rampTarget);
        rotationSpeedRamp.update(TOKYO_ROTATIONAL_SPEED_INCREMENT);
        desiredRotation = rotationSpeedRamp.getValue();

        rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);

    } else {
        Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
    }

    chassis->setTargetRPMs(desiredX, desiredY, desiredRotation);
}

void ChassisTokyoCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisTokyoCommand::isReady() { return true; }

bool ChassisTokyoCommand::isFinished() const { return false; }

void ChassisTokyoCommand::randomizeSpinCharacteristics() {
    this->spinRateModifier = src::Utils::Random::getRandomFloatInBounds(
        randomizerConfig.minSpinRateModifier,
        randomizerConfig.maxSpinRateModifier);

    this->spinRateModifierDuration = src::Utils::Random::getRandomIntegerInBounds(
        randomizerConfig.minSpinRateModifierDuration,
        randomizerConfig.maxSpinRateModifierDuration);
}

}  // namespace src::Chassis

#endif