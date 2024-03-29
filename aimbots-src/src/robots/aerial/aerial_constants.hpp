#pragma once
#include "utils/common_types.hpp"

#define NO_CHASSIS

static constexpr uint8_t DRIVEN_WHEEL_COUNT = 0;
static constexpr uint8_t MOTORS_PER_WHEEL = 0;

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
static constexpr float YAW_POSITION_PID_KP = 0.0f;
static constexpr float YAW_POSITION_PID_KI = 0.0f;
static constexpr float YAW_POSITION_PID_KD = 0.0f;
static constexpr float YAW_POSITION_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float YAW_POSITION_PID_Q_DERIVATIVE_KALMAN = 0.0f;
static constexpr float YAW_POSITION_PID_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float YAW_POSITION_PID_Q_PROPORTIONAL_KALMAN = 0.0f;
static constexpr float YAW_POSITION_PID_R_PROPORTIONAL_KALMAN = 0.0f;

static constexpr float PITCH_POSITION_PID_KP = 0.0f;
static constexpr float PITCH_POSITION_PID_KI = 0.0f;
static constexpr float PITCH_POSITION_PID_KD = 0.0f;
static constexpr float PITCH_POSITION_PID_MAX_ERROR_SUM = 0.0f;
static constexpr float PITCH_POSITION_PID_Q_DERIVATIVE_KALMAN = 0.0f;
static constexpr float PITCH_POSITION_PID_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float PITCH_POSITION_PID_Q_PROPORTIONAL_KALMAN = 0.0f;
static constexpr float PITCH_POSITION_PID_R_PROPORTIONAL_KALMAN = 0.0f;

/**
 * This max output is measured in the c620 robomaster translated current.
 * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
 * The corresponding speed controller output torque current range is
 * -20 ~ 0 ~ 20 A.
 */
static constexpr float VELOCITY_PID_MAX_OUTPUT = 16000.0f;
static constexpr float POSITION_PID_MAX_OUTPUT = 10.0f;

// Mechanical chassis constants, all in m
/**
 * Radius of the wheels (m).
 */
// static constexpr float WHEEL_RADIUS = 0.076;

// static constexpr float WHEELBASE_WIDTH = 0.366f;

// static constexpr float WHEELBASE_LENGTH = 0.366f;

static constexpr float GIMBAL_X_OFFSET = 0.0f;
static constexpr float GIMBAL_Y_OFFSET = 0.0f;

static constexpr float YAW_OFFSET_ANGLE = 90.0f;
static constexpr uint16_t YAW_START_ENCODER = 2048;
static constexpr float PITCH_OFFSET_ANGLE = 30.0f;
#error "DM Richard on Discord if you see this (or just calculate the pitch stop limits yourself idc)"
static constexpr float PITCH_AXIS_SOFTSTOP_LOW = 0.0f;
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = 0.0f;

// static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

static constexpr MotorID YAW_MOTOR_ID = MotorID::MOTOR5;
static constexpr MotorID PITCH_MOTOR_ID = MotorID::MOTOR6;



// Power limiting constants, will explain later
static constexpr float MAX_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 40.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 5;
static constexpr uint16_t POWER_CONSUMPTION_THRESHOLD = 20;
static constexpr float CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING = 30000;
