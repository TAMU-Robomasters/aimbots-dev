#include "subsystems/chassis/control/chassis_helper.hpp"

#include "utils/math/random.hpp"

#include "tap/architecture/clock.hpp"


#ifdef CHASSIS_COMPATIBLE

int8_t chassisYDesiredWheelspeedWatch = 0;
int8_t chassisXDesiredWheelspeedWatch = 0;

namespace src::Chassis::Helper {

void getUserDesiredInput(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    float* desiredXInput,
    float* desiredYInput,
    float* desiredRotationInput) {
    if (drivers == nullptr || chassis == nullptr || desiredXInput == nullptr || desiredYInput == nullptr ||
        desiredRotationInput == nullptr) {
        return;
    }

    *desiredXInput = drivers->controlOperatorInterface.getChassisXInput();
    *desiredYInput = drivers->controlOperatorInterface.getChassisYInput();
    *desiredRotationInput = drivers->controlOperatorInterface.getChassisRotationInput();
}

void rescaleDesiredInputToPowerLimitedSpeeds(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    float* desiredX,
    float* desiredY,
    float* desiredRotation) {
    if (drivers == nullptr || chassis == nullptr || desiredX == nullptr || desiredY == nullptr ||
        desiredRotation == nullptr) {
        return;
    }

    // Gets the maximum speed that we're realistically able to achieve with the current power limit.
    float maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

#if defined(TARGET_STANDARD_2025) || defined(TARGET_STANDARD_BLASTOISE)
    maxWheelSpeed = 4000.0f;
#endif

    rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, desiredX, desiredY, desiredRotation, maxWheelSpeed);
}

void rescaleDesiredInputToPowerLimitedSpeeds(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    float* desiredX,
    float* desiredY,
    float* desiredRotation,
    float maxWheelSpeed) {
    (void)drivers;

    if (chassis == nullptr || desiredX == nullptr || desiredY == nullptr || desiredRotation == nullptr) {
        return;
    }

    *desiredRotation *= maxWheelSpeed;

    // The maximum translational speed that we can achieve while maintaining the desired rotation speed.
    float rTranslationalGain = chassis->calculateRotationLimitedTranslationalWheelspeed(*desiredRotation, maxWheelSpeed);

    *desiredX = limitVal<float>(*desiredX * maxWheelSpeed, -rTranslationalGain, rTranslationalGain);
    *desiredY = limitVal<float>(*desiredY * maxWheelSpeed, -rTranslationalGain, rTranslationalGain);
}

void randomizeSpinCharacteristics(
    float* spinRateModifier,
    uint32_t* spinRateModifierDuration,
    SpinRandomizerConfig randomizerConfig) {
    *spinRateModifier = src::Utils::Random::getRandomFloatInBounds(
        randomizerConfig.minSpinRateModifier,
        randomizerConfig.maxSpinRateModifier);

    *spinRateModifierDuration = src::Utils::Random::getRandomIntegerInBounds(
        randomizerConfig.minSpinRateModifierDuration,
        randomizerConfig.maxSpinRateModifierDuration);
}

// TODO: make new helper similar to randomizeSpinCharacteristics
// Takes in a time arg so it allows sin function

float timeDisplay = 0.0f;
float ampDisplay = 0.0f;
float sinDisplay = 0.0f;


void sinusodalSpinCharacteristics( // ZHENGHAO-99
    float* spinRateModifier,
    uint32_t* spinRateModifierDuration,
    SpinRandomizerConfig randomizerConfig,
    bool complex) {
    
    // auto timeNow = std::chrono::system_clock::now();
    auto timeEpoch = tap::arch::clock::getTimeMilliseconds();
    float ms = static_cast<float>(timeEpoch % 1000000000);
    
    timeDisplay = ms;
    
    float amp = randomizerConfig.maxSpinRateModifier - randomizerConfig.minSpinRateModifier;
    ampDisplay = amp;
    float funnyFactor = 7.0f;
    
    // sinDisplay = std::sin(0.0023*ms);
    
    if(!complex) {
        *spinRateModifier = randomizerConfig.minSpinRateModifier+(amp/2.0) + (amp * std::sin(0.001*ms)); // idk man
    }
    else {
        // idk js multiply random sins together
        //*spinRateModifier = randomizerConfig.minSpinRateModifier+(amp/2.0) + (amp * std::sin(0.001*ms) * std::sin(0.00067*ms+1.11));
        *spinRateModifier = randomizerConfig.minSpinRateModifier+(amp/2.0) + ((amp*0.5 * std::cos(0.0004*funnyFactor*ms))+(amp*std::sin(0.0005*funnyFactor*ms)*(std::sin(0.0004*ms*(funnyFactor/2)))));
    }

    *spinRateModifierDuration = randomizerConfig.minSpinRateModifierDuration; // ts probably not needed lmao ZHENG-HAO
    // *spinRateModifierDuration = src::Utils::Random::getRandomIntegerInBounds(
    //     randomizerConfig.minSpinRateModifierDuration,
    //     randomizerConfig.maxSpinRateModifierDuration);

    
}

// Pass a ChassisRelative Error to this function, and it will return the error for the nearest chassis corner
float findNearestChassisErrorTo(float chassisRelativeTargetAngle, SnapSymmetryConfig snapSymmetryConfig) {
    float angleBetweenCorners = M_TWOPI / static_cast<float>(snapSymmetryConfig.numSnapPositions);
    WrappedFloat targetContiguousAngle(chassisRelativeTargetAngle, 0, M_TWOPI);

    float nearestCornerError = targetContiguousAngle.minDifference(snapSymmetryConfig.snapAngle);

    for (int i = 1; i < snapSymmetryConfig.numSnapPositions; i++) {
        float currentCornerError =
            targetContiguousAngle.minDifference(snapSymmetryConfig.snapAngle + i * angleBetweenCorners);

        if (fabsf(currentCornerError) < fabsf(nearestCornerError)) {
            nearestCornerError = currentCornerError;
        }
    }

    return nearestCornerError;
}

}  // namespace src::Chassis::Helper

#endif  //#ifdef CHASSIS_COMPATIBLE