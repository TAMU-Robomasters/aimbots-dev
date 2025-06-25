#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define CHASSIS_COMPATIBLE

static constexpr uint8_t DRIVEN_WHEEL_COUNT = 4;
static constexpr uint8_t MOTORS_PER_WHEEL = 1;

static constexpr SmoothPIDConfig CHASSIS_VELOCITY_PID_CONFIG = {
    .kp = 18.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

// 1 for no symmetry, 2 for 180 degree symmetry, 4 for 90 degree symmetry
static constexpr uint8_t CHASSIS_SNAP_POSITIONS = 4;

// CAN Bus 2
static constexpr CANBus CHASSIS_BUS = CANBus::CAN_BUS2;

static constexpr MotorID LEFT_BACK_WHEEL_ID = MotorID::MOTOR1;
static constexpr MotorID LEFT_FRONT_WHEEL_ID = MotorID::MOTOR2;
static constexpr MotorID RIGHT_FRONT_WHEEL_ID = MotorID::MOTOR3;
static constexpr MotorID RIGHT_BACK_WHEEL_ID = MotorID::MOTOR4;

// Mechanical chassis constants, all in m
/**
 * Radius of the wheels (m).
 */
static constexpr float WHEEL_RADIUS = 0.07663f;

static constexpr float WHEELBASE_WIDTH = 0.500f;

static constexpr float WHEELBASE_LENGTH = 0.5372f;

static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

/**
 * @brief Power constants for chassis
 */
static constexpr int MIN_WHEEL_SPEED_SINGLE_MOTOR = 4000;
static constexpr int MAX_WHEEL_SPEED_SINGLE_MOTOR = 8000;
static constexpr int MIN_CHASSIS_POWER = 50;
static constexpr int MAX_CHASSIS_POWER = 80;
static constexpr int WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE =
    (MAX_WHEEL_SPEED_SINGLE_MOTOR - MIN_WHEEL_SPEED_SINGLE_MOTOR) / (MAX_CHASSIS_POWER - MIN_CHASSIS_POWER);
static_assert(WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE >= 0);

/**
 * @brief Behavior constants for chassis
 */

/**
 * The minimum desired wheel speed for chassis rotation, measured in RPM before
 * we start slowing down translational speed.
 */
static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;

static constexpr float FOLLOW_GIMBAL_ANGLE_THRESHOLD = modm::toRadian(20.0f);

static constexpr SmoothPIDConfig ROTATION_POSITION_PID_CONFIG = {
    .kp = 1.65f,  // 0.45
    .ki = 0.0f,
    .kd = 0.005f,  // 0.05
    .maxICumulative = 0.1f,
    .maxOutput = 1.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

/**
 * @brief TOKYO CONSTANTS
 */
// Fraction that user input is multiplied by when "drifting"
static constexpr float TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER = 0.6f;
// Fraction of the maximum translation speed for when rotation speed should be reduced
static constexpr float TOKYO_TRANSLATION_THRESHOLD_TO_DECREASE_ROTATION_SPEED = 0.5f;
// Fraction of max chassis speed applied to rotation speed
static constexpr float TOKYO_ROTATIONAL_SPEED_FRACTION_OF_MAX = 0.75f;
// Fraction to cut rotation speed by when the robot is "drifting"
static constexpr float TOKYO_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING = 0.7f;
// Rotational speed increment per iteration to apply until rotation setpoint is reached
static constexpr float TOKYO_ROTATIONAL_SPEED_INCREMENT = 50.0f;  // rpm

static constexpr float CHASSIS_START_ANGLE_WORLD = modm::toRadian(0.0f);  // theta (about z axis)
