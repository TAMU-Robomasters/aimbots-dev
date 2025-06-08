#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define FEEDER_COMPATIBLE

static constexpr uint8_t FEEDER_MOTOR_COUNT = 2;

static constexpr SmoothPIDConfig FEEDER_VELOCITY_PID_CONFIG = {
    .kp = 40.0f,
    .ki = 0.0f,
    .kd = 0.8f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr int UNJAM_TIMER_MS = 50;

static constexpr float FEEDER_DEFAULT_RPM = 12000.0f;

static constexpr CANBus FEEDER_BUS = CANBus::CAN_BUS1;

static const std::array<MotorID, FEEDER_MOTOR_COUNT> FEEDER_MOTOR_IDS = {MotorID::MOTOR7, MotorID::MOTOR8};
static const std::array<const char*, FEEDER_MOTOR_COUNT> FEEDER_MOTOR_NAMES = {"Feeder Motor 1", "Feeder Motor 2"};
static const std::array<float, FEEDER_MOTOR_COUNT> FEEDER_NORMAL_RPMS = {500, 4000};
static const std::array<float, FEEDER_MOTOR_COUNT> FEEDER_UNJAM_RPMS = {1000, 3000};  // Absolute values
static const std::array<FeederGroup, FEEDER_MOTOR_COUNT> FEEDER_MOTOR_GROUPS = {SECONDARY, PRIMARY};
static constexpr float PROJECTILES_PER_FEEDER_ROTATION = 0.5;
static constexpr std::array<uint8_t, FEEDER_MOTOR_COUNT> FEEDER_GEAR_RATIOS = {36, 36};
static const std::array<bool, FEEDER_MOTOR_COUNT> FEEDER_DIRECTION = {true, true};
//