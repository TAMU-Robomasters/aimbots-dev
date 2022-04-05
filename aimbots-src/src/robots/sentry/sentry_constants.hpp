#pragma once
#include "utils/common_types.hpp"

/**
 * @brief Defines the number of motors created for the chassis.
 */
static constexpr uint8_t DRIVEN_WHEEL_COUNT = 1;
static constexpr uint8_t MOTORS_PER_WHEEL = 1;

static constexpr uint8_t SHOOTER_MOTOR_COUNT = 4;
/**
 * @brief Definitions for operator interface constants (may change based on preference of drivers)
 *
 */
static constexpr int16_t USER_MOUSE_YAW_MAX = 1000;
static constexpr int16_t USER_MOUSE_PITCH_MAX = 1000;
static constexpr float USER_MOUSE_YAW_SCALAR = (1.0f / USER_MOUSE_YAW_MAX);
static constexpr float USER_MOUSE_PITCH_SCALAR = (1.0f / USER_MOUSE_PITCH_MAX);

static constexpr float CTRL_SCALAR = (1.0f / 4);
static constexpr float SHIFT_SCALAR = (1.0f / 2);

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
static constexpr float YAW_POSITION_PID_KD = 25000.0f;
static constexpr float YAW_POSITION_PID_MAX_ERROR_SUM = 5000.0f;

static constexpr float PITCH_POSITION_PID_KP = 600.0f;
static constexpr float PITCH_POSITION_PID_KI = 0.0f;
static constexpr float PITCH_POSITION_PID_KD = 1000.0f;
static constexpr float PITCH_POSITION_PID_MAX_ERROR_SUM = 5000.0f;

// Used to reverse Feeder Motor direction, should only be 1 or -1
static constexpr float FEEDER_MOTOR_DIRECTION = -1;
static constexpr float YAW_MOTOR_DIRECTION = -1;

// CAN Bus 1
static constexpr MotorID RAIL_WHEEL_ID = MotorID::MOTOR3;
static constexpr MotorID YAW_MOTOR_ID = MotorID::MOTOR5;
static constexpr MotorID PITCH_MOTOR_ID = MotorID::MOTOR6;
static constexpr MotorID FEEDER_ID = MotorID::MOTOR8;

static constexpr CANBus CHASSIS_BUS = CANBus::CAN_BUS1;
static constexpr CANBus GIMBAL_BUS = CANBus::CAN_BUS1;

static constexpr CANBus SHOOTER_BUS = CANBus::CAN_BUS2;

// CAN Bus 2
static constexpr MotorID SHOOTER_1_ID = MotorID::MOTOR1;
static constexpr MotorID SHOOTER_2_ID = MotorID::MOTOR2;
static constexpr MotorID SHOOTER_3_ID = MotorID::MOTOR3;
static constexpr MotorID SHOOTER_4_ID = MotorID::MOTOR4;

/**
 * This max output is measured in the c620 robomaster translated current.
 * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
 * The corresponding speed controller output torque current range is
 * -20 ~ 0 ~ 20 A.
 */
static constexpr float VELOCITY_PID_MAX_OUTPUT = 16000.0f;
static constexpr float POSITION_PID_MAX_OUTPUT = 16000.0f;

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

// Values specific for Sentry hardware setup
static constexpr float YAW_START_ANGLE = 61.0f;
static constexpr float PITCH_START_ANGLE = 220.0f;

static constexpr float PITCH_HARDSTOP_LOW = 270.0f;
static constexpr float PITCH_HARDSTOP_HIGH = 155.0f;

/**
 * Max wheel speed, measured in RPM of the 3508 motor shaft.
 */
static constexpr int MAX_3508_ENC_RPM = 7000;

// Power limiting constants, will explain later
static constexpr float MAX_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 40.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 5;
static constexpr uint16_t POWER_CONSUMPTION_THRESHOLD = 20;
static constexpr float CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING = 30000;

/**
 * @brief Power constants for chassis
 */
static constexpr int MIN_WHEEL_SPEED_SINGLE_MOTOR = 4000;
static constexpr int MAX_WHEEL_SPEED_SINGLE_MOTOR = 8000;
static constexpr int MIN_CHASSIS_POWER = 40;
static constexpr int MAX_CHASSIS_POWER = 120;
static constexpr int WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE =
    (MAX_WHEEL_SPEED_SINGLE_MOTOR - MIN_WHEEL_SPEED_SINGLE_MOTOR) /
    (MAX_CHASSIS_POWER - MIN_CHASSIS_POWER);
static_assert(WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE >= 0);

/**
 * @brief Behavior constants for chassis
 */

/**
 * The minimum desired wheel speed for chassis rotation, measured in RPM before
 * we start slowing down translational speed.
 */
static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;
