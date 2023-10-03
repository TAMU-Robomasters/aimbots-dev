#pragma once
#include "utils/common_types.hpp"

#define ENGINEER_COMPATIBLE
#define TARGET_ENGINEER
#define WRIST_COMPATIBLE

static constexpr uint8_t DRIVEN_WHEEL_COUNT = 4;
static constexpr uint8_t MOTORS_PER_WHEEL = 1;

/**
 * @brief Velocity PID constants
 */
static constexpr float VELOCITY_PID_KP = 20.0f;
static constexpr float VELOCITY_PID_KI = 0.2f;
static constexpr float VELOCITY_PID_KD = 0.0f;
static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 5000.0f;

/**
 * @brief Position PID constants
 */
static constexpr float POSITION_PID_KP = 20.0f;
static constexpr float POSITION_PID_KI = 0.2f;
static constexpr float POSITION_PID_KD = 0.0f;
static constexpr float POSITION_PID_MAX_ERROR_SUM = 5000.0f;

// Mechanical chassis constants, all in m
/**
 * Radius of the wheels (m).
 */
static constexpr float WHEEL_RADIUS = 0.076;

static constexpr float WHEELBASE_WIDTH = 0.366f;

static constexpr float WHEELBASE_LENGTH = 0.366f;

static constexpr float GIMBAL_X_OFFSET = 0.0f;
static constexpr float GIMBAL_Y_OFFSET = 0.0f;

static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

static constexpr float YAW_OFFSET_ANGLE = M_PI_2;
static constexpr float PITCH_OFFSET_ANGLE = M_PI_2;
#warning "DM Richard on Discord if you see this (or just calculate the pitch stop limits yourself idc)"
static constexpr float PITCH_AXIS_SOFTSTOP_LOW = 0.0f;
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = 0.0f;

static constexpr CANBus WRIST_BUS = CANBus::CAN_BUS2;

static constexpr MotorID LEFT_BACK_WHEEL_ID = MotorID::MOTOR1;
static constexpr MotorID LEFT_FRONT_WHEEL_ID = MotorID::MOTOR2;
static constexpr MotorID RIGHT_FRONT_WHEEL_ID = MotorID::MOTOR3;
static constexpr MotorID RIGHT_BACK_WHEEL_ID = MotorID::MOTOR4;
static constexpr MotorID YAW_MOTOR_ID = MotorID::MOTOR5;
static constexpr MotorID PITCH_MOTOR_ID = MotorID::MOTOR6;
static constexpr MotorID ROLL_MOTOR_ID = MotorID::MOTOR7;  // MOTOR7 is a placeholder!!!

// M3508
static constexpr SmoothPIDConfig WRIST_VELOCITY_PID_CONFIG = {
    .kp = 1.0f,
    .ki = 0.0f,
    .kd = 1.5f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

// M2006
static constexpr SmoothPIDConfig WRIST_ROLL_VELOCITY_PID_CONFIG = {
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

enum WheelRPMIndex {  // index used to easily navigate wheel matrices
    LB = 0,
    LF = 1,
    RF = 2,
    RB = 3,
};

// Power limiting constants, will explain later
static constexpr float MAX_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 40.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 5;
static constexpr uint16_t POWER_CONSUMPTION_THRESHOLD = 20;
static constexpr float CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING = 30000;