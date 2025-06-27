#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define GIMBAL_COMPATIBLE

#define GIMBAL_UNTETHERED

/**
 * @brief GIMBAL SETUP
 */

static constexpr CANBus PITCH_GIMBAL_BUS = CANBus::CAN_BUS1;

static constexpr uint8_t YAW_MOTOR_COUNT = 1;
static constexpr uint8_t PITCH_MOTOR_COUNT = 1;

#if defined(TARGET_SENTRY_BRAVO)
static constexpr CANBus YAW_GIMBAL_BUS = CANBus::CAN_BUS1;
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static const std::array<float, YAW_MOTOR_COUNT> YAW_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(42.58f))};
static const std::array<float, PITCH_MOTOR_COUNT> PITCH_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(229.0f))};

static constexpr float PITCH_AXIS_SOFTSTOP_LOW = modm::toRadian(-2.0f);
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = modm::toRadian(30.0f);
// LOW should be lesser than HIGH, otherwise switch the motor direction

#elif defined(TARGET_SENTRY_SWERVE)
static constexpr CANBus YAW_GIMBAL_BUS = CANBus::CAN_BUS1;
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static const std::array<float, YAW_MOTOR_COUNT> YAW_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(54.58f))};
static const std::array<float, PITCH_MOTOR_COUNT> PITCH_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(147.8f))};

static constexpr float PITCH_AXIS_SOFTSTOP_LOW = modm::toRadian(0.0f);
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = modm::toRadian(40.0f);
// LOW should be lesser than HIGH, otherwise switch the motor direction

#endif

static const std::array<bool, YAW_MOTOR_COUNT> YAW_MOTOR_DIRECTIONS = {false};
static const std::array<MotorID, YAW_MOTOR_COUNT> YAW_MOTOR_IDS = {MotorID::MOTOR8};//TODO
static const std::array<const char*, YAW_MOTOR_COUNT> YAW_MOTOR_NAMES = {"Yaw Motor 1"};

static constexpr float YAW_AXIS_START_ANGLE = modm::toRadian(0.0f);

static constexpr float GIMBAL_YAW_GEAR_RATIO = (1.0f / 2.0f);  // for 2024 Sentry
/*Changing this means the encoder-readable range of the YAW axis is reduced to 360deg * GIMBAL_YAW_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the robot will be started within the same GIMBAL_YAW_GEAR_RATIO range
 * every time. We also assume that 1 / GIMBAL_YAW_GEAR_RATIO is an integer multiple of 360deg. */

static const std::array<bool, PITCH_MOTOR_COUNT> PITCH_MOTOR_DIRECTIONS = {false};
static const std::array<MotorID, PITCH_MOTOR_COUNT> PITCH_MOTOR_IDS = {MotorID::MOTOR6};
static const std::array<const char*, PITCH_MOTOR_COUNT> PITCH_MOTOR_NAMES = {"Pitch Motor 1"};

static constexpr float PITCH_AXIS_START_ANGLE = modm::toRadian(0.0f);

static constexpr float GIMBAL_PITCH_GEAR_RATIO = (5.0f / 17.0f);  // for 2023 Sentry
/*Changing this means the encoder-readable range of the PITCH axis is reduced to 360deg * GIMBAL_PITCH_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the range of the pitch axis is hardware-limited to not exceed this
 * range, but the motor angle may cross 0 in this range. Example Range: 278deg to 28deg */

/**
 * @brief Position PID constants
 */
static constexpr SmoothPIDConfig YAW_POSITION_PID_CONFIG = {
    .kp = 2'000.0f,
    .ki = 100.0f,
    .kd = 3'000.0f,
    .maxICumulative = 0.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 10.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig PITCH_POSITION_PID_CONFIG = {
    .kp = 20'000.0f,
    .ki = 0.0f,
    .kd = 100.0f,
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
    .kp = 20.0f,  // 35
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 1000.0f,
    .maxOutput = 40.0f,  // 40 rad/s is maximum speed of 6020
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig PITCH_POSITION_CASCADE_PID_CONFIG = {
    .kp = 25.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 1000.0f,
    .maxOutput = 35.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

// VELOCITY PID CONSTANTS
static constexpr SmoothPIDConfig YAW_VELOCITY_PID_CONFIG = {
    .kp = 2000.0f,
    .ki = 25.0f,
    .kd = 0.0f,
    .maxICumulative = 2000.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 10.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig PITCH_VELOCITY_PID_CONFIG = {
    .kp = 900.0f,
    .ki = 30.0f,
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

static constexpr float kGRAVITY = 0.0f;
static constexpr float HORIZON_OFFSET = 0.0f;

const modm::Pair<float, float> YAW_FEEDFORWARD_VELOCITIES[11] = {
    {0.0f, 0.0f},
    {1.5f, 3'000.0f},
    {5.15f, 6'000.0f},
    {8.8f, 9'000.0f},
    {12.6f, 12'000.0f},
    {16.45f, 15'000.0f},
    {20.25f, 18'000.0f},
    {24.11f, 21'000.0f},
    {27.97f, 24'000.0f},
    {29.18f, 27'000.0f},
    {29.2f, 30'000.0f}};

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
// clang-format on

const modm::interpolation::Linear<modm::Pair<float, float>> YAW_VELOCITY_FEEDFORWARD(YAW_FEEDFORWARD_VELOCITIES, 11);
const modm::interpolation::Linear<modm::Pair<float, float>> PITCH_VELOCITY_FEEDFORWARD(PITCH_FEEDFORWARD_VELOCITIES, 11);

static constexpr float GIMBAL_X_OFFSET = 0.0f;  //-0.05
static constexpr float GIMBAL_Y_OFFSET = 0.0f;  //-0.05

// PITCH PATROL FUNCTION CONSTANTS
// TODO: Gimbal Control Command needs to be modified to work in radians, and then convert these constants.
static constexpr float PITCH_PATROL_AMPLITUDE = modm::toRadian(12.5f);
static constexpr float PITCH_PATROL_FREQUENCY = 1.5f * M_PI;
static constexpr float PITCH_PATROL_OFFSET = 20.0f;  // degrees offset from horizon
static constexpr float PITCH_OFFSET_ANGLE = 0;       // In degrees currently