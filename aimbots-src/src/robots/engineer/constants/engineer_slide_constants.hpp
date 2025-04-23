#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define SLIDE_COMPATIBLE

static constexpr SmoothPIDConfig SLIDE_X_POSITION_PID_CONFIG = {
    .kp = 30000.0f,
    .ki = 0.0f,
    .kd = 10.0f,
    .maxICumulative = 0.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig SLIDE_Z_POSITION_PID_CONFIG = {
    .kp = 60000.0f,
    .ki = 10.0f,
    .kd = 8.0f,
    .maxICumulative = 3000.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

// TODO: set these to what they actually are
static constexpr uint8_t SLIDE_MOTOR_COUNT = 2;
static constexpr CANBus SLIDE_BUS = CANBus::CAN_BUS1;
static constexpr MotorID SLIDE_X_MOTOR_ID = MotorID::MOTOR4;
static constexpr MotorID SLIDE_Z_MOTOR_ID = MotorID::MOTOR5;
static constexpr bool SLIDE_X_MOTOR_DIRECTION = false;
static constexpr bool SLIDE_Z_MOTOR_DIRECTION = true;
static constexpr float SLIDE_METERS_PER_REVS_RATIOS[]{(0.254f / 46.0f), (0.184f / 130.0f)};
static constexpr float SLIDE_MAX_POSITIONS_METERS[] = {0.254f, 0.25f};  // 0.254 0.18
