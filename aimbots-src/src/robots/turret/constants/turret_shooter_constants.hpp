#pragma once

#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

static constexpr SmoothPIDConfig SHOOTER_VELOCITY_PID_CONFIG = {
    .kp = 40.0f,
    .ki = 0.10f,
    .kd = 0.00f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr uint16_t shooter_speed_array[6] = {
    15, 3900,  // {ball m/s, flywheel rpm}
    18, 4500,
    30, 9000};

static const Matrix<uint16_t, 3, 2> SHOOTER_SPEED_MATRIX(shooter_speed_array);


static constexpr CANBus SHOOTER_BUS = CANBus::CAN_BUS1;

static constexpr MotorID SHOOTER_1_ID = MotorID::MOTOR3;
static constexpr MotorID SHOOTER_2_ID = MotorID::MOTOR4;

static constexpr bool SHOOTER_1_DIRECTION = false;
static constexpr bool SHOOTER_2_DIRECTION = true;