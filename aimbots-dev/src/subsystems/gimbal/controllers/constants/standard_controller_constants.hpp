#pragma once

#include <tap/algorithms/smooth_pid.hpp>

namespace src::Gimbal::Constants {

namespace ChassisRelative {

static constexpr tap::algorithms::SmoothPidConfig YAW_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 32000.0f,
    .tQDerivativeKalman = 0.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

static constexpr tap::algorithms::SmoothPidConfig PITCH_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 32000.0f,
    .tQDerivativeKalman = 0.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
};

}

}