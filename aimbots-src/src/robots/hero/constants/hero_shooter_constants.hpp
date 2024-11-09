#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define SHOOTER_COMPATIBLE

static constexpr uint8_t SHOOTER_MOTOR_COUNT = 4;

static constexpr SmoothPIDConfig SHOOTER_VELOCITY_PID_CONFIG = {
    .kp = 30.0f,
    .ki = 0.10f,
    .kd = 0.0f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr uint16_t shooter_speed_array[4] = {
    10,
    6100,  // {ball m/s, flywheel rpm} //3900
    16,
    6100};  // 6100

static const Matrix<uint16_t, 2, 2> SHOOTER_SPEED_MATRIX(shooter_speed_array);

static constexpr int DEFAULT_BURST_LENGTH = 5;  // balls

static constexpr CANBus SHOOTER_BUS = CANBus::CAN_BUS1;

static constexpr MotorID SHOOTER_1_ID = MotorID::MOTOR1;
static constexpr MotorID SHOOTER_2_ID = MotorID::MOTOR2;
static constexpr MotorID SHOOTER_3_ID = MotorID::MOTOR3;
static constexpr MotorID SHOOTER_4_ID = MotorID::MOTOR4;

static constexpr bool SHOOTER_1_DIRECTION = true;
static constexpr bool SHOOTER_2_DIRECTION = false;
static constexpr bool SHOOTER_3_DIRECTION = false;
static constexpr bool SHOOTER_4_DIRECTION = true;

// This array holds the IDs of all speed monitor barrels on the robot
static const std::array<BarrelID, 1> BARREL_IDS = {BarrelID::TURRET_42MM};
