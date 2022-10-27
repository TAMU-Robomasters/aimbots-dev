#pragma once
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

/**
 * @brief Defines the number of motors created for the chassis.
 */
static constexpr uint8_t DRIVEN_WHEEL_COUNT = 1;
static constexpr uint8_t MOTORS_PER_WHEEL = 1;

static constexpr uint8_t SHOOTER_MOTOR_COUNT = 4;

// in radians
static constexpr float DEV_BOARD_YAW_OFFSET = 0.0f;

/**
 * @brief Definitions for operator interface constants (may change based on preference of drivers)
 *
 */
static constexpr int16_t USER_MOUSE_YAW_MAX = 1000;
static constexpr int16_t USER_MOUSE_PITCH_MAX = 1000;
static constexpr float USER_MOUSE_YAW_SCALAR = (1.0f / USER_MOUSE_YAW_MAX);
static constexpr float USER_MOUSE_PITCH_SCALAR = (1.0f / USER_MOUSE_PITCH_MAX);

static constexpr float USER_JOYSTICK_YAW_SCALAR = 0.3f;
static constexpr float USER_JOYSTICK_PITCH_SCALAR = 0.3f;

static constexpr float CTRL_SCALAR = (1.0f / 4);
static constexpr float SHIFT_SCALAR = (1.0f / 2);

static constexpr SmoothPIDConfig CHASSIS_VELOCITY_PID_CONFIG = {
    .kp = 18.0f,
    .ki = 0.0f,
    .kd = 2.0f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig SHOOTER_VELOCITY_PID_CONFIG = {
    .kp = 30.0f,
    .ki = 0.10f,
    .kd = 0.00f,
    .maxICumulative = 10.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig FEEDER_VELOCITY_PID_CONFIG = {
    .kp = 20.0f,
    .ki = 0.01f,
    .kd = 0.0f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

/**
 * @brief Position PID constants
 */
static constexpr SmoothPIDConfig YAW_POSITION_PID_CONFIG = {
    .kp = 500.0f,
    .ki = 0.0f,
    .kd = 700.0f,
    .maxICumulative = 10.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig PITCH_POSITION_PID_CONFIG = {
    .kp = 350.0f,
    .ki = 1.3f,
    .kd = 200.0f,
    .maxICumulative = 1000.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr float kGRAVITY = 6000.0f;
static constexpr float HORIZON_OFFSET = -30.0f;

// sentry only has one speed: death
static constexpr uint16_t shooter_speed_array[2] =
    {30, 8000};  // {m/s, rpm}

static const Matrix<uint16_t, 1, 2> SHOOTER_SPEED_MATRIX(shooter_speed_array);

static constexpr float FEEDER_DEFAULT_RPM = 500.0f;

static constexpr int DEFAULT_BURST_LENGTH = 10;  // total balls in burst

static constexpr int MAX_BURST_LENGTH = 20;
static constexpr int MIN_BURST_LENGTH = 4;

// CAN Bus 1
static constexpr MotorID RAIL_WHEEL_ID = MotorID::MOTOR3;
static constexpr MotorID YAW_MOTOR_ID = MotorID::MOTOR5;
static constexpr MotorID PITCH_MOTOR_ID = MotorID::MOTOR6;
static constexpr MotorID FEEDER_ID = MotorID::MOTOR8;

static constexpr CANBus CHASSIS_BUS = CANBus::CAN_BUS1;
static constexpr CANBus GIMBAL_BUS = CANBus::CAN_BUS1;

static constexpr CANBus SHOOTER_BUS = CANBus::CAN_BUS2;

static constexpr CANBus FEED_BUS = CANBus::CAN_BUS1;

// CAN Bus 2
static constexpr MotorID SHOOTER_1_ID = MotorID::MOTOR1;
static constexpr MotorID SHOOTER_2_ID = MotorID::MOTOR2;
static constexpr MotorID SHOOTER_3_ID = MotorID::MOTOR3;
static constexpr MotorID SHOOTER_4_ID = MotorID::MOTOR4;

static constexpr bool SHOOTER_1_DIRECTION = true;
static constexpr bool SHOOTER_2_DIRECTION = true;
static constexpr bool SHOOTER_3_DIRECTION = false;
static constexpr bool SHOOTER_4_DIRECTION = false;

static constexpr bool FEEDER_DIRECTION = true;

static constexpr bool YAW_DIRECTION = false;
static constexpr bool PITCH_DIRECTION = true;

// Mechanical chassis constants, all in m
/**
 * Radius of the wheels (m).
 */
static constexpr float WHEEL_RADIUS = 0.0206375f;

static constexpr float WHEELBASE_WIDTH = 0.403174f;

static constexpr float WHEELBASE_LENGTH = 0.366f;  // meters!!1!

static constexpr float GIMBAL_X_OFFSET = 0.0f;
static constexpr float GIMBAL_Y_OFFSET = 0.0f;

static constexpr float left_sentry_rail_pole_location[3] = {-4.375f, -0.960f, 0.0f};
static const Matrix<float, 1, 3> left_sentry_rail_pole_location_matrix(left_sentry_rail_pole_location);
// x, y, z in meters
// x is along length of field, y is along width of field, z is vertical

static constexpr float RAIL_POLE_DIAMETER = 0.061f;

static constexpr float robot_starting_rail_location_array[3] = {((WHEELBASE_WIDTH + RAIL_POLE_DIAMETER) / 2.0f), 0.0f, 0.0f};
static const Matrix<float, 1, 3> robot_starting_rail_location(robot_starting_rail_location_array);

static constexpr float FULL_RAIL_LENGTH = 2.130f;                                                       // meters, pole center to pole center
static constexpr float FULL_RAIL_LENGTH_CM = 213.0f;                                                    // cm
static constexpr float USABLE_RAIL_LENGTH = FULL_RAIL_LENGTH - (WHEELBASE_WIDTH + RAIL_POLE_DIAMETER);  // in meters

static const Matrix<float, 1, 3> ROBOT_STARTING_POSITION = left_sentry_rail_pole_location_matrix + robot_starting_rail_location * src::utils::MatrixHelper::xy_rotation_matrix(AngleUnit::Degrees, 45.0f);

static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f) * (44.0f / 18.0f);

// field-relative math is based on

// Values specific for Sentry hardware setup
static constexpr float YAW_START_ANGLE = 61.0f;
static constexpr float PITCH_START_ANGLE = 152.0f;

static constexpr float PITCH_SOFTSTOP_LOW = 105.42f;
static constexpr float PITCH_SOFTSTOP_HIGH = 218.84f;

// PITCH PATROL FUNCTION CONSTANTS
static constexpr float PITCH_PATROL_AMPLITUDE = 12.5f;  // degrees
static constexpr float PITCH_PATROL_FREQUENCY = 1.5f * M_PI;
static constexpr float PITCH_PATROL_OFFSET = 20.0f;  // degrees offset from horizon

/**
 * Max wheel speed, measured in RPM of the 3508 motor shaft.
 */
static constexpr int MAX_3508_ENC_RPM = 7000;

// Power limiting constants, will explain later
static constexpr float POWER_LIMIT_SAFETY_FACTOR = 0.85f;
static constexpr float STARTING_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 10.0f;
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


/**
 * @brief Sentry Rotation and Distance/Position matrixies 
 */
namespace src::robots::sentry {

float R_cam2gimb[9] = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};
float P_cam2gimb[3] = {1, 2, 3};

float R_gimb2cam[9] = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};
float P_gimb2cam[3] = {-1,-2,-3};

float R_chas2gimb[9] = {1, 0, 0,
                        0, 1, 0,
                        0, 0, 1};
float P_chas2gimb[3] = {1, 2, 3};

float R_gimb2chas[9] = {1, 0, 0,
                        0, 1, 0,
                        0, 0, 1};
float P_gimb2chas[3] = {-1,-2,-3};

float R_chas2field[9] = {1, 0, 0,
                         0, 1, 0,
                         0, 0, 1};
float P_chas2field[3] = {1, 2, 3};

float R_field2chas[9] = {1, 0, 0,
                         0, 1, 0,
                         0, 0, 1};
float P_field2chas[3] = {-1,-2,-3};
}