#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define WRIST_COMPATIBLE

static constexpr CANBus WRIST_BUS = CANBus::CAN_BUS1;

static constexpr uint8_t WRIST_MOTOR_COUNT = 7;

static constexpr SmoothPIDConfig WRIST_PID_CONFIG = {
    .kp = 0.0f,  
    .ki = 0.0f,
    .kd = 0.8f,  
    .maxICumulative = 0.0f,
    .maxOutput = M2006_MAX_OUTPUT,
    .tQDerivativeKalman = 0.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
static constexpr SmoothPIDConfig ARM_PID_CONFIG = {
    .kp = 0.0f,  
    .ki = 0.0f,
    .kd = 0.8f,  
    .maxICumulative = 0.0f,
    .maxOutput = M2006_MAX_OUTPUT,
    .tQDerivativeKalman = 0.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
static constexpr SmoothPIDConfig FULL_PID_CONFIG = {
    .kp = 0.0f,  
    .ki = 0.0f,
    .kd = 0.8f,  
    .maxICumulative = 0.0f,
    .maxOutput = M2006_MAX_OUTPUT,
    .tQDerivativeKalman = 0.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
static constexpr SmoothPIDConfig INTERMEDIATE_PID_CONFIG = {
    .kp = 0.0f,  
    .ki = 0.0f,
    .kd = 0.8f,  
    .maxICumulative = 0.0f,
    .maxOutput = M2006_MAX_OUTPUT,
    .tQDerivativeKalman = 0.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static const std::array<float, WRIST_MOTOR_COUNT> WRIST_MOTOR_OFFSET_ANGLES = {
    wrapTo0To2PIRange(modm::toRadian(0.0f)),   // 186.15
    wrapTo0To2PIRange(modm::toRadian(0.0f)),   // 196.21
    wrapTo0To2PIRange(modm::toRadian(0.0f))};  // 196.21

// TODO: SET THESE GEAR RATIOS PLEASE
static const std::array<float, WRIST_MOTOR_COUNT> WRIST_MOTOR_IN_PER_OUT_RATIOS{365.0f, 361.0f, 361.0f};
static const std::array<bool, WRIST_MOTOR_COUNT> WRIST_MOTOR_DIRECTIONS = {true, false, false, false, false, true, false};
static const std::array<MotorID, WRIST_MOTOR_COUNT> WRIST_MOTOR_IDS = {
    MotorID::MOTOR1,
    MotorID::MOTOR2,
    MotorID::MOTOR3,
    MotorID::MOTOR4,
    MotorID::MOTOR5,
    MotorID::MOTOR6,
    MotorID::MOTOR7};  // Yaw, Pitch, Roll
static const std::array<const char*, WRIST_MOTOR_COUNT> WRIST_MOTOR_NAMES = {"Yaw/Pitch Motor 1", "Yaw/Pitch Motor 2", "Yaw Wrist Motor", "Arm Yaw Motor", "Master Full Pitch", "Slave Full Pitch", "Intermediate Pitch"};
