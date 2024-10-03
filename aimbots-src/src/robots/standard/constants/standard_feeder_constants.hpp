#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define FEEDER_COMPATIBLE

static constexpr uint8_t FEEDER_MOTOR_COUNT = 1;

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

static constexpr int UNJAM_TIMER_MS = 300;

static constexpr float FEEDER_DEFAULT_RPM = 4150.0f;  // 4500
static constexpr int DEFAULT_BURST_LENGTH = 5;        // balls
static constexpr CANBus FEEDER_BUS = CANBus::CAN_BUS1;

static const std::array<MotorID, FEEDER_MOTOR_COUNT> FEEDER_MOTOR_IDS = {MotorID::MOTOR7};
static const std::array<const char*, FEEDER_MOTOR_COUNT> FEEDER_MOTOR_NAMES = {"Feeder Motor 1"};
static const std::array<float, FEEDER_MOTOR_COUNT> FEEDER_TARGET_RPMS = {FEEDER_DEFAULT_RPM};
static constexpr uint8_t PROJECTILES_PER_FEEDER_ROTATION = 10;
static constexpr std::array<uint8_t, FEEDER_MOTOR_COUNT> FEEDER_GEAR_RATIOS = {36};

static const std::array<bool, FEEDER_MOTOR_COUNT> FEEDER_DIRECTION = {false};

static constexpr size_t PROJECTILE_SPEED_QUEUE_SIZE = 10;
