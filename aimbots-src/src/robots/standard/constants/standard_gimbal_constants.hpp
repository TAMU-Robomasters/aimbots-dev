#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define GIMBAL_COMPATIBLE
#define GIMBAL_UNTETHERED

/**
 * @brief GIMBAL SETUP
 */

static constexpr CANBus YAW_GIMBAL_BUS = CANBus::CAN_BUS2;
static constexpr CANBus PITCH_GIMBAL_BUS = CANBus::CAN_BUS1;

static constexpr uint8_t YAW_MOTOR_COUNT = 1;
static constexpr uint8_t PITCH_MOTOR_COUNT = 1;

#if defined(TARGET_STANDARD_BLASTOISE)
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static const std::array<float, YAW_MOTOR_COUNT> YAW_MOTOR_OFFSET_ANGLES = {modm::toRadian(144.27f)};
static const std::array<float, PITCH_MOTOR_COUNT> PITCH_MOTOR_OFFSET_ANGLES = {modm::toRadian(11.11f)};
static constexpr float PITCH_AXIS_SOFTSTOP_LOW = modm::toRadian(-14.5f);
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = modm::toRadian(17.5f);
static const std::array<bool, PITCH_MOTOR_COUNT> PITCH_MOTOR_DIRECTIONS = {false};
// LOW should be lesser than HIGH, otherwise switch the motor direction

#elif defined(TARGET_STANDARD_2025)
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot cassis? */
static const std::array<float, YAW_MOTOR_COUNT> YAW_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(-211.2f))};
static const std::array<float, PITCH_MOTOR_COUNT> PITCH_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(298.25f))};
static constexpr float PITCH_AXIS_SOFTSTOP_LOW = modm::toRadian(-14.5f);
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = modm::toRadian(17.5f);
static const std::array<bool, PITCH_MOTOR_COUNT> PITCH_MOTOR_DIRECTIONS = {true};
// LOW should be lesser than HIGH, otherwise switch the motor direction

#elif defined(TARGET_STANDARD_SQUIRTLE)
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static const std::array<float, YAW_MOTOR_COUNT> YAW_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(254.75f))};
static const std::array<float, PITCH_MOTOR_COUNT> PITCH_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(-110.64f))};
static constexpr float PITCH_AXIS_SOFTSTOP_LOW = modm::toRadian(-14.5f);
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = modm::toRadian(17.5f);
static const std::array<bool, PITCH_MOTOR_COUNT> PITCH_MOTOR_DIRECTIONS = {false};
// LOW should be lesser than HIGH, otherwise switch the motor direction

#elif defined(TARGET_STANDARD_2023)
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static const std::array<float, YAW_MOTOR_COUNT> YAW_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(251.89f))};
static const std::array<float, PITCH_MOTOR_COUNT> PITCH_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(166.86f))};
static constexpr float PITCH_AXIS_SOFTSTOP_LOW = modm::toRadian(-24.0f);
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = modm::toRadian(22.0f);
static const std::array<bool, PITCH_MOTOR_COUNT> PITCH_MOTOR_DIRECTIONS = {false};
// LOW should be lesser than HIGH, otherwise switch the motor direction

#endif

static const std::array<bool, YAW_MOTOR_COUNT> YAW_MOTOR_DIRECTIONS = {false};
static const std::array<MotorID, YAW_MOTOR_COUNT> YAW_MOTOR_IDS = {MotorID::MOTOR5};
static const std::array<const char*, YAW_MOTOR_COUNT> YAW_MOTOR_NAMES = {"Yaw Motor 1"};
static constexpr float YAW_AXIS_START_ANGLE = modm::toRadian(0.0f);

static constexpr float GIMBAL_YAW_GEAR_RATIO = (1.0f / 2.0f);  // for 2024 Standard
/*Changing this means the encoder-readable range of the YAW axis is reduced to 360deg * GIMBAL_YAW_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the robot will be started within the same GIMBAL_YAW_GEAR_RATIO range
 * every time. We also assume that 1 / GIMBAL_YAW_GEAR_RATIO is an integer multiple of 360deg. */
static const std::array<MotorID, PITCH_MOTOR_COUNT> PITCH_MOTOR_IDS = {MotorID::MOTOR6};
static const std::array<const char*, PITCH_MOTOR_COUNT> PITCH_MOTOR_NAMES = {"Pitch Motor 1"};
static constexpr float PITCH_AXIS_START_ANGLE = modm::toRadian(0.0f);

static constexpr float GIMBAL_PITCH_GEAR_RATIO = (30.0f / 102.0f);  // for 2024 Standard
/*Changing this means the encoder-readable range of the PITCH axis is reduced to 360deg * GIMBAL_PITCH_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the range of the pitch axis is hardware-limited to not exceed this
 * range, but the motor angle may cross 0 in this range. Example Range: 278deg to 28deg */

/**
 * @brief Position PID constants
 */
static constexpr SmoothPIDConfig YAW_POSITION_PID_CONFIG = {
    .kp = 50'000.0f,  // 600
    .ki = 0.0f,
    .kd = 1'000.0f,  // 500
    .maxICumulative = 0.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 10000.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig PITCH_POSITION_PID_CONFIG = {
    .kp = 50'000.0f,
    .ki = 0.0f,
    .kd = 850.0f,
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
    .kp = 30.0f,  // 30
    .ki = 0.0f,
    .kd = 0.5f,
    .maxICumulative = 1.0f,
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
    .maxICumulative = 1.0f,
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
    .kp = 1850.0f,  // 3000
    .ki = 25.0f,    // 25
    .kd = 0.0f,
    .maxICumulative = 2000.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 1000.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig PITCH_VELOCITY_PID_CONFIG = {
    .kp = 700.0f,
    .ki = 15.0f,
    .kd = 0.0f,
    .maxICumulative = 3000.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr float CHASSIS_VELOCITY_YAW_LOAD_FEEDFORWARD = 1.0f;
static constexpr float CHASSIS_VELOCITY_PITCH_LOAD_FEEDFORWARD = 1.0f;

static constexpr float CHASSIS_LINEAR_ACCELERATION_PITCH_COMPENSATION = 0.0f;

static constexpr float kGRAVITY = -1500.0f;  // Negative because weight is behind pitch motor
static constexpr float HORIZON_OFFSET = 0.0f;

// clang-format off
const modm::Pair<float, float> YAW_FEEDFORWARD_VELOCITIES[11] = {
                                                                    {0.0f, 0.0f},
                                                                    {1.5f, 3'000.0f},
                                                                    {5.25f, 6'000.0f},
                                                                    {9.0f, 9'000.0f},
                                                                    {13.2f, 12'000.0f},
                                                                    {17.2f, 15'000.0f},
                                                                    {21.0f, 18'000.0f},
                                                                    {24.85f, 21'000.0f},
                                                                    {28.6f, 24'000.0f},
                                                                    {29.75f, 27'000.0f},
                                                                    {29.9f, 30'000.0f}
                                                                    };


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
                                                                    {36.35f, 30'000.0f}
                                                                    };
// clang-format on

const modm::interpolation::Linear<modm::Pair<float, float>> YAW_VELOCITY_FEEDFORWARD(YAW_FEEDFORWARD_VELOCITIES, 11);
const modm::interpolation::Linear<modm::Pair<float, float>> PITCH_VELOCITY_FEEDFORWARD(PITCH_FEEDFORWARD_VELOCITIES, 11);

// -------------------------------------------------------------------------------------------------------------------------
