#pragma once
#include "utils/tools/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

#define SHOOTER_COMPATIBLE

static constexpr uint8_t SHOOTER_MOTOR_COUNT = 2;

static constexpr float HORIZON_OFFSET = 0.0f;


static constexpr SmoothPIDConfig SHOOTER_VELOCITY_PID_CONFIG = {
    .kp = 40.0f,
    .ki = 0.10f,  // 0.10f;
    .kd = 0.00f,
    .maxICumulative = 10.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr uint16_t shooter_speed_array[2] = {30, 7450};  // {m/s, rpm}

static const Matrix<uint16_t, 1, 2> SHOOTER_SPEED_MATRIX(shooter_speed_array);

static constexpr float FEEDER_DEFAULT_RPM = 4150.0f;  // 4500
static constexpr int DEFAULT_BURST_LENGTH = 5;        // balls

static constexpr CANBus SHOOTER_BUS = CANBus::CAN_BUS1;

static constexpr MotorID SHOOTER_1_ID = MotorID::MOTOR2;
static constexpr MotorID SHOOTER_2_ID = MotorID::MOTOR1;

static constexpr bool SHOOTER_1_DIRECTION = false;
static constexpr bool SHOOTER_2_DIRECTION = true;


