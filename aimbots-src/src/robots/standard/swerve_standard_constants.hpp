#pragma once
#include "utils/common_types.hpp"

#define SWERVE

static constexpr uint8_t DRIVEN_WHEEL_COUNT = 4;
static constexpr uint8_t MOTORS_PER_WHEEL = 2;

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
static constexpr float YAW_POSITION_PID_KP = 600.0f;
static constexpr float YAW_POSITION_PID_KI = 0.0f;
static constexpr float YAW_POSITION_PID_KD = 0.0f;
static constexpr float YAW_POSITION_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float YAW_POSITION_PID_Q_DERIVATIVE_KALMAN = 0.0f;
static constexpr float YAW_POSITION_PID_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float YAW_POSITION_PID_Q_PROPORTIONAL_KALMAN = 0.0f;
static constexpr float YAW_POSITION_PID_R_PROPORTIONAL_KALMAN = 0.0f;

static constexpr float PITCH_POSITION_PID_KP = 600.0f;
static constexpr float PITCH_POSITION_PID_KI = 0.0f;
static constexpr float PITCH_POSITION_PID_KD = 0.0f;
static constexpr float PITCH_POSITION_PID_MAX_ERROR_SUM = 5000.0f;
static constexpr float PITCH_POSITION_PID_Q_DERIVATIVE_KALMAN = 0.0f;
static constexpr float PITCH_POSITION_PID_R_DERIVATIVE_KALMAN = 0.0f;
static constexpr float PITCH_POSITION_PID_Q_PROPORTIONAL_KALMAN = 0.0f;
static constexpr float PITCH_POSITION_PID_R_PROPORTIONAL_KALMAN = 0.0f;

static constexpr MotorID LEFT_BACK_WHEEL_ID = MotorID::MOTOR1;
static constexpr MotorID LEFT_FRONT_WHEEL_ID = MotorID::MOTOR2;
static constexpr MotorID RIGHT_FRONT_WHEEL_ID = MotorID::MOTOR3;
static constexpr MotorID RIGHT_BACK_WHEEL_ID = MotorID::MOTOR4;

static constexpr MotorID LEFT_BACK_YAW_ID = MotorID::MOTOR5;
static constexpr MotorID LEFT_FRONT_YAW_ID = MotorID::MOTOR6;
static constexpr MotorID RIGHT_FRONT_YAW_ID = MotorID::MOTOR7;
static constexpr MotorID RIGHT_BACK_YAW_ID = MotorID::MOTOR8;

/**
 * This max output is measured in the c620 robomaster translated current.
 * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
 * The corresponding speed controller output torque current range is
 * -20 ~ 0 ~ 20 A.
 */
static constexpr float VELOCITY_PID_MAX_OUTPUT = 16000.0f;

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

static constexpr float YAW_OFFSET_ANGLE = 90.0f;
static constexpr uint16_t YAW_OFFSET_ANGLE = 2048;
static constexpr float PITCH_OFFSET_ANGLE = 30.0f;
#error "DM Richard on Discord if you see this (or just calculate the pitch stop limits yourself idc)"
static constexpr float PITCH_AXIS_SOFTSTOP_LOW = 0.0f;
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = 0.0f;



// Power limiting constants, will explain later
static constexpr float MAX_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 40.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 5;
static constexpr uint16_t POWER_CONSUMPTION_THRESHOLD = 20;
static constexpr float CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING = 30000;

static constexpr float SHOOTER_PID_KP = 50.0f;
static constexpr float SHOOTER_PID_KI = 0.0f;
static constexpr float SHOOTER_PID_KD = 0.0f;
static constexpr float SHOOTER_MAX_I_CUMULATIVE = 10.0f;
static constexpr float SHOOTER_MAX_OUTPUT = 30000.0f;
static constexpr float SHOOTER_TQ_DERIVATIVE_KALMAN = 1.0f;
static constexpr float SHOOTER_TR_DERIVATIVE_KALMAN = 1.0f;
static constexpr float SHOOTER_TQ_PROPORTIONAL_KALMAN = 1.0f;
static constexpr float SHOOTER_TR_PROPORTIONAL_KALMAN = 1.0f;
