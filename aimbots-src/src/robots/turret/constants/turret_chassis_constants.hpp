#pragma once

#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/util_macros.hpp"

#include "utils/filters/ema.hpp"



static constexpr uint8_t DRIVEN_WHEEL_COUNT = 4; //
static constexpr uint8_t MOTORS_PER_WHEEL = 1;//

static constexpr uint8_t SHOOTER_MOTOR_COUNT = 2;//

static constexpr SmoothPIDConfig CHASSIS_VELOCITY_PID_CONFIG = {
    .kp = 5.0f,
    .ki = 0.0f,
    .kd = 0.5f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr CANBus CHASSIS_BUS = CANBus::CAN_BUS2;

static constexpr MotorID LEFT_BACK_WHEEL_ID = MotorID::MOTOR1;
static constexpr MotorID LEFT_FRONT_WHEEL_ID = MotorID::MOTOR2;
static constexpr MotorID RIGHT_FRONT_WHEEL_ID = MotorID::MOTOR3;
static constexpr MotorID RIGHT_BACK_WHEEL_ID = MotorID::MOTOR4;



static constexpr float WHEEL_RADIUS = 0.076;

static constexpr float WHEELBASE_WIDTH = 0.366f;

static constexpr float WHEELBASE_LENGTH = 0.366f;

static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);


/**
 * The minimum desired wheel speed for chassis rotation, measured in RPM before
 * we start slowing down translational speed.
 */
static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;