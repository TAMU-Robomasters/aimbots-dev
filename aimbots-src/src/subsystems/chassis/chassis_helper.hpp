#pragma once

#include <drivers.hpp>
#include <subsystems/chassis/chassis.hpp>

namespace src::Chassis {
struct SpinRandomizerConfig {
    float minSpinRateModifier = 0.75f;
    float maxSpinRateModifier = 1.0f;
    uint32_t minSpinRateModifierDuration = 500;
    uint32_t maxSpinRateModifierDuration = 3000;
};

struct SnapSymmetryConfig {
    uint8_t numSnapPositions = 1;
    float snapAngle = 0.0f;
};
namespace Helper {
/**
 * @brief Gets the user's desired movement from the control operator interface from [-1, 1]
 */
void getUserDesiredInput(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    float* desiredXInput,
    float* desiredYInput,
    float* desiredRotationInput);

/**
 * @brief Limits translational movement based on rotational movement (inversely proportional)
 *
 * Input range should be [-1, 1], output will be [-maxWheelSpeed, maxWheelSpeed]
 **/
void rescaleDesiredInputToPowerLimitedSpeeds(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    float* desiredX,
    float* desiredY,
    float* desiredRotation);

float findNearestChassisErrorTo(float targetAngle, SnapSymmetryConfig snapSymmetryConfig);
}  // namespace Helper

}  // namespace src::Chassis