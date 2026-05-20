#pragma once
#include "utils/tools/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"
#include "tap/motor/dji_motor.hpp"

#define FEEDER_COMPATIBLE

static constexpr SmoothPIDConfig FEEDER_VELOCITY_PID_CONFIG = {
    .kp = 50.0f,  // 8000
    .ki = 0.0f,
    .kd = 0.0f,  // 100
    .maxICumulative = 10.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C610,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f, 
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr int UNJAM_TIMER_MS = 400;


static constexpr uint8_t FEEDER_MOTOR_COUNT = 1;

static const std::array<MotorID, FEEDER_MOTOR_COUNT> FEEDER_MOTOR_IDS = {MotorID::MOTOR7};
static const std::array<const char*, FEEDER_MOTOR_COUNT> FEEDER_MOTOR_NAMES = {"Feeder Motor 1"};
static constexpr float PROJECTILES_PER_FEEDER_ROTATION = 11;
static constexpr std::array<uint8_t, FEEDER_MOTOR_COUNT> FEEDER_GEAR_RATIOS = {36};
static const std::array<float, FEEDER_MOTOR_COUNT> FEEDER_NORMAL_RPMS = {110.0f}; // enough to shoot at a rate of 20 Hz (refresh rate of panels)
static const std::array<float, FEEDER_MOTOR_COUNT> FEEDER_UNJAM_RPMS = {110.0f};  // Absolute values
static const std::array<FeederGroup, FEEDER_MOTOR_COUNT> FEEDER_MOTOR_GROUPS = {PRIMARY};
static const std::array<bool, FEEDER_MOTOR_COUNT> FEEDER_DIRECTION = {false};

static float feederFeedforward(float targetRPM) {
    float slope = 48.68f;
    float offset = 666.84f;
    float threshold_RPM = 50.0f;

    if (targetRPM <= threshold_RPM) {
        return slope*threshold_RPM + offset;
    }

    return slope*targetRPM + offset;
}

static constexpr CANBus FEEDER_BUS = CANBus::CAN_BUS1;
