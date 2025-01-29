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

/* === Motor configuration variables ===
 *  --   last updated: 16/jan/2025   --
 *
 * if increasing/decreasing motors 
 *   --> update enum MotorIndex in "slide.hpp" (line 14)
 *   --> update constants + constant arrays below
 * 
 *  thanks & gig em <3
 * 
 */

// DJIMotor config
static constexpr uint8_t SLIDE_MOTOR_COUNT = 2;
static const std::array<MotorID, SLIDE_MOTOR_COUNT> SLIDE_MOTOR_IDS = {MotorID::MOTOR4, MotorID::MOTOR5};
static constexpr CANBus SLIDE_BUS = CANBus::CAN_BUS1;
static const std::array<bool, SLIDE_MOTOR_COUNT> SLIDE_MOTOR_DIRECTIONS = {false, true};
static const std::array<const char*, SLIDE_MOTOR_COUNT> SLIDE_MOTOR_NAMES = {"Slide Motor X", "Slide Motor Y"};
// DJIMotor PIDs
static const std::array<SmoothPIDConfig, SLIDE_MOTOR_COUNT> SLIDE_MOTOR_PIDS = {SLIDE_X_POSITION_PID_CONFIG, SLIDE_Z_POSITION_PID_CONFIG};
// Hardware calculations
static const std::array<float, SLIDE_MOTOR_COUNT> SLIDE_MOTOR_METERS_PER_REVS_RATIOS = {(0.254f / 46.0f), (0.184f / 130.0f)};
static const std::array<float, SLIDE_MOTOR_COUNT> SLIDE_MOTOR_MAX_POSITIONS_METERS = {0.254f, 0.18f};