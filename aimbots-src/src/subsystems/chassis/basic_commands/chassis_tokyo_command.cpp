#include "chassis_tokyo_command.hpp"

#ifdef GIMBAL_UNTETHERED
#ifdef CHASSIS_COMPATIBLE

#include <chrono>

#include <subsystems/chassis/control/chassis_helper.hpp>

#include "subsystems/chassis/control/chassis_helper.hpp"

#warning "tokyo compatible"

namespace src::Chassis {

ChassisTokyoCommand::ChassisTokyoCommand(
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

void ChassisTokyoCommand::initialize() {
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

float xTargetDisplay = 0.0f;
float rampTargetDisplay = 0.0f;
float spinRateModDisplay = 0.0f;
bool boolDisplay = false;
uint16_t timerLeftDisplay = 0;

void ChassisTokyoCommand::execute() {
    chassis->setTokyoDrift(true);
    float desiredX = 0.0f;
    float desiredY = 0.0f;
    float desiredRotation = 0.0f;

    // we overwrite desiredRotation later if tokyo drifting
    Helper::getUserDesiredInput(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
    float maxWheelSpeed;
    if (gimbal->isOnline()) {
        float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        // this is wrapped between -PI and PI

        // The maximum speed that we're realistically able to achieve with the current power limit

        maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        #if defined(TARGET_STANDARD_2025) || defined(TARGET_STANDARD_BLASTOISE)||defined(SENTRY_SWERVE)
            maxWheelSpeed = 4000;
        #endif

        desiredX *= tokyoConfig.translationalSpeedMultiplier * maxWheelSpeed;
        desiredY *= tokyoConfig.translationalSpeedMultiplier * maxWheelSpeed;

        const float translationalSpeedThreshold = maxWheelSpeed * tokyoConfig.translationalSpeedMultiplier *
                                                  tokyoConfig.translationThresholdToDecreaseRotationSpeed;

        float rampTarget = maxWheelSpeed * rotationDirection * tokyoConfig.rotationalSpeedFractionOfMax;
        // boolDisplay = randomizeSpinRate;
        if (randomizeSpinRate) {
            // timerLeftDisplay = spinRateModifierTimer.timeRemaining();
            // boolDisplay = spinRateModifierTimer.isExpired() || spinRateModifierTimer.isStopped();
            if (spinRateModifierTimer.isExpired() || spinRateModifierTimer.isStopped()) {
                // vvvv Previous randomizer
                Helper::randomizeSpinCharacteristics(
                    &this->spinRateModifier,
                    &this->spinRateModifierDuration,
                    randomizerConfig);
                spinRateModifierTimer.restart(spinRateModifierDuration);

                // Helper::sinusodalSpinCharacteristics(
                //     &this->spinRateModifier,
                //     &this->spinRateModifierDuration,
                //     randomizerConfig);
                // spinRateModifierTimer.restart(spinRateModifierDuration); // im assuming ts in sec
            }

            // Helper::sinusodalSpinCharacteristics(
            //     &this->spinRateModifier,
            //     &this->spinRateModifierDuration,
            //     randomizerConfig);
            // spinRateModifierTimer.restart(spinRateModifierDuration); // im assuming ts in sec

            rampTarget *= spinRateModifier; // This is what changes speed? ZHENGHAO-99
        }
        

        // reduces rotation speed when translation speed is high
        if (fabsf(desiredX) > translationalSpeedThreshold || fabsf(desiredY) > translationalSpeedThreshold) {
            rampTarget *= tokyoConfig.rotationalSpeedMultiplierWhenTranslating;
        }
        spinRateModDisplay = spinRateModifier;
        rampTargetDisplay = rampTarget;

        rotationSpeedRamp.setTarget(rampTarget);
        rotationSpeedRamp.update(tokyoConfig.rotationalSpeedIncrement);
        desiredRotation = rotationSpeedRamp.getValue();
        
        rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);

    } else {
        xTargetDisplay = 69420.67;
        Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
    }
    xTargetDisplay = desiredX;
    chassis->setTargetRPMs(desiredX, desiredY, desiredRotation);
}

void ChassisTokyoCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisTokyoCommand::isReady() { return true; }

bool ChassisTokyoCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif  //#ifdef CHASSIS_COMPATIBLE
#endif  //#ifdef GIMBAL_UNTETHERED