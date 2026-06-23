#pragma once
#include "tap/motor/dji_motor.hpp"
#include "modm/math/geometry/angle.hpp"
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define GIMBAL_UNTETHERED
#define GIMBAL_COMPATIBLE

#define YAW_3508
#define INVERTED_YAW_ENCODER

/**
 * @brief GIMBAL SETUP 
 */
static constexpr CANBus YAW_GIMBAL_BUS = CANBus::CAN_BUS1;
static constexpr CANBus PITCH_GIMBAL_BUS = CANBus::CAN_BUS1;

static constexpr uint8_t YAW_MOTOR_COUNT = 1;
static constexpr uint8_t PITCH_MOTOR_COUNT = 1;

static const std::array<bool, YAW_MOTOR_COUNT> YAW_MOTOR_DIRECTIONS = {false};
static const std::array<MotorID, YAW_MOTOR_COUNT> YAW_MOTOR_IDS = {MotorID::MOTOR5};
static const std::array<const char*, YAW_MOTOR_COUNT> YAW_MOTOR_NAMES = {"Yaw Motor 1"};
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static const std::array<float, YAW_MOTOR_COUNT> YAW_MOTOR_OFFSET_ANGLES = {RevEncoderValueToRadians(21925)};
static constexpr float YAW_AXIS_START_ANGLE = modm::toRadian(0.0f);

 //static constexpr float GIMBAL_YAW_GEAR_RATIO = 1.0f/2.0754716981f;  // for 2025 Hero
static constexpr float GIMBAL_YAW_GEAR_RATIO = 1.0f/2.0f;  // for 2025 Hero
/* Changing this means the encoder-readable range of the YAW axis is reduced to 360deg * GIMBAL_YAW_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the robot will be started within the same GIMBAL_YAW_GEAR_RATIO range
 * every time. We also assume that 1 / GIMBAL_YAW_GEAR_RATIO is an integer multiple of 360deg. */
 static constexpr float GIMBAL_YAW_MOTOR_GEAR_RATIO = 2.0f/1.0f;

static const std::array<bool, PITCH_MOTOR_COUNT> PITCH_MOTOR_DIRECTIONS = {false};
static const std::array<MotorID, PITCH_MOTOR_COUNT> PITCH_MOTOR_IDS = {MotorID::MOTOR6};
static const std::array<const char*, PITCH_MOTOR_COUNT> PITCH_MOTOR_NAMES = {"Pitch Motor 1"};
static const std::array<float, PITCH_MOTOR_COUNT> PITCH_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(155.0f))};
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */

static constexpr float PITCH_AXIS_START_ANGLE = modm::toRadian(0.0f);

static constexpr float GIMBAL_PITCH_GEAR_RATIO = (5.0f / 17.0f);
/*Changing this means the encoder-readable range of the PITCH axis is reduced to 360deg * GIMBAL_PITCH_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the range of the pitch axis is hardware-limited to not exceed this
 * range, but the motor angle may cross 0 in this range. Example Range: 278deg to 28deg */

//static constexpr float GIMBAL_YAW_MOTOR_GEAR_RATIO = (38.0f / 1.0f);

static constexpr float PITCH_AXIS_SOFTSTOP_LOW = (-0.61);
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = (0.56);

//20 220


/// Chassis Relative Velocity Yaw Feedforward Equation
// Derived by setting the desired yaw voltage to different values and measuring the velocity of the yaw
static inline float chassisRelativeVelocityYawFeedforward(float desiredYawVelocity) {
    return (desiredYawVelocity + 40.0367) / 0.0203;;
}

/**
 * @brief Position PID constants
 */
static constexpr SmoothPIDConfig YAW_POSITION_PID_CONFIG = {
    .kp = 900.0f,
    .ki = 30.0f,
    .kd = 0.0f,
    .maxICumulative = 5000.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig PITCH_POSITION_PID_CONFIG = {
    .kp = 30.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 10.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

// VISION PID CONSTANTS
static constexpr SmoothPIDConfig YAW_POSITION_CASCADE_PID_CONFIG = {
    .kp = 20.5, 
    .ki = 0.0f,
    .kd = 0.0f,  // 0.15
    .maxICumulative = 1000.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620, // for 3508
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig PITCH_POSITION_CASCADE_PID_CONFIG = {
    .kp = 30.0f,  // 30
    .ki = 0.0f,
    .kd = 0.0f,  // 0
    .maxICumulative = 1000.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

// VELOCITY PID CONSTANTS
static constexpr SmoothPIDConfig YAW_VELOCITY_PID_CONFIG = {
    .kp = 1200.0f,  
    .ki = 10.0f,    
    .kd = 0.0f,
    .maxICumulative = 5000.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.1,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig PITCH_VELOCITY_PID_CONFIG = {
    .kp = 400.0f, 
    .ki = 10.0f,  
    .kd = 0.0f,
    .maxICumulative = 15000,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.1f,
    .errorDerivativeFloor = 0.0f,
};

const modm::Pair<float, float> YAW_FEEDFORWARD_VELOCITIES[11] = {
    {0.0f, 0.0f},
    {1.75f, 3'000.0f},
    {6.0f, 6'000.0f},
    {10.2f, 9'000.0f},
    {14.4f, 12'000.0f},
    {18.8f, 15'000.0f},
    {23.05f, 18'000.0f},
    {27.30f, 21'000.0f},
    {31.5f, 24'000.0f},
    {32.71f, 27'000.0f},
    {32.72f, 30'000.0f}};

const modm::Pair<float, float> PITCH_FEEDFORWARD_VELOCITIES[11] = {
    {0.0f, 0.0f},
    {3.75f, 3'000.0f},
    {8.5f, 6'000.0f},
    {12.75f, 9'000.0f},
    {17.67f, 12'000.0f},
    {22.5f, 15'000.0f},
    {26.75f, 18'000.0f},
    {31.5f, 21'000.0f},
    {35.5f, 24'000.0f},
    {36.15f, 27'000.0f},
    {36.35f, 30'000.0f}};

const modm::interpolation::Linear<modm::Pair<float, float>> YAW_VELOCITY_FEEDFORWARD(YAW_FEEDFORWARD_VELOCITIES, 11);
const modm::interpolation::Linear<modm::Pair<float, float>> PITCH_VELOCITY_FEEDFORWARD(PITCH_FEEDFORWARD_VELOCITIES, 11);

static constexpr float GIMBAL_X_OFFSET = 0.0f;
static constexpr float GIMBAL_Y_OFFSET = 0.0f;

static constexpr float CHASSIS_VELOCITY_YAW_LOAD_FEEDFORWARD = 1.0f;
static constexpr float CHASSIS_VELOCITY_PITCH_LOAD_FEEDFORWARD = 1.0f;

static constexpr float CHASSIS_LINEAR_ACCELERATION_PITCH_COMPENSATION = 0.0f;

static constexpr float kGRAVITY = 4500.0f;
static constexpr float HORIZON_OFFSET = -0.0f;
