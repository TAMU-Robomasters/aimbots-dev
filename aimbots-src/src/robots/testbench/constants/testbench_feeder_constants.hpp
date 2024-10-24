#pragma once
#include "utils/tools/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

static constexpr SmoothPIDConfig FEEDER_VELOCITY_PID_CONFIG = {
    .kp = 15.0f,
    .ki = 0.0f,
    .kd = 0.8f,
    .maxICumulative = 10.0f,
    .maxOutput = M2006_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr uint8_t PROJECTILES_PER_FEEDER_ROTATION = 10;
static constexpr float FEEDER_DEFAULT_RPM = 3000.0f;
static constexpr int DEFAULT_BURST_LENGTH = 5;  // balls

static constexpr CANBus FEED_BUS = CANBus::CAN_BUS1;


//
static constexpr MotorID FEEDER_ID = MotorID::MOTOR2;
//

static constexpr bool FEEDER_DIRECTION = false;

// #endif