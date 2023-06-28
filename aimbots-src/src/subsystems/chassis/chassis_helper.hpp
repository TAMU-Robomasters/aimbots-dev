#pragma once

#include <drivers.hpp>
#include <subsystems/chassis/chassis.hpp>

namespace src::Chassis::Helper {

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

float findNearestChassisErrorTo(float targetAngle, uint8_t numSnapPositions, float starterAngle = 0.0f);

}  // namespace src::Chassis::Helper