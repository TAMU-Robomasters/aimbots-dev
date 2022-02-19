#pragma once
#include "utils/common_types.hpp"

static constexpr uint8_t DRIVEN_WHEEL_COUNT = 1;
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

static constexpr MotorID RAIL_WHEEL_ID  = MotorID::MOTOR1;
static constexpr MotorID YAW_MOTOR_ID   = MotorID::MOTOR5;
static constexpr MotorID PITCH_MOTOR_ID = MotorID::MOTOR6;

static constexpr CANBus CHASSIS_CAN_BUS = CANBus::CAN_BUS1;
static constexpr CANBus GIMBAL_CAN_BUS  = CANBus::CAN_BUS2;

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

static constexpr float YAW_START_ANGLE = 90.0f;
static constexpr float PITCH_START_ANGLE = 90.0f;

/**
 * Constants for gimbal gravity compensation
 */
static constexpr float GIMBAL_CENTER_OF_GRAVITY_OFFSET_X = 0.0f; // millimeters
static constexpr float GIMBAL_CENTER_OF_GRAVITY_OFFSET_Z = 0.0f; // millimeters
static constexpr float GRAVITY_COMPENSATION_MAX          = 10.0f;

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