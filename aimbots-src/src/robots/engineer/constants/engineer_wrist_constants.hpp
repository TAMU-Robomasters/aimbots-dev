#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define WRIST_COMPATIBLE

static constexpr CANBus WRIST_BUS = CANBus::CAN_BUS1;

static constexpr uint8_t WRIST_MOTOR_COUNT = 3;

static constexpr uint8_t WRIST_MOTOR_COUNT_PER_SIDE = 1;

static const std::array<float, WRIST_MOTOR_COUNT> WRIST_MOTOR_OFFSET_ANGLES = {
    wrapTo0To2PIRange(modm::toRadian(0.0f)),   // 186.15
    wrapTo0To2PIRange(modm::toRadian(0.0f)),   // 196.21
    wrapTo0To2PIRange(modm::toRadian(0.0f))};  // 196.21

// TODO: SET THESE GEAR RATIOS PLEASE
static const std::array<float, WRIST_MOTOR_COUNT> WRIST_MOTOR_IN_PER_OUT_RATIOS{365.0f, 361.0f, 361.0f};
static const std::array<bool, WRIST_MOTOR_COUNT> WRIST_MOTOR_DIRECTIONS = {true, false, false};
static const std::array<MotorID, WRIST_MOTOR_COUNT> WRIST_MOTOR_IDS = {
    MotorID::MOTOR1,
    MotorID::MOTOR2,
    MotorID::MOTOR3};  // Yaw, Pitch, Roll
static const std::array<const char*, WRIST_MOTOR_COUNT> WRIST_MOTOR_NAMES = {"Yaw Motor", "Pitch Motor", "Roll Motor"};
