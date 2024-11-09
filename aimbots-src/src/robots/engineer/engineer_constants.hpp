#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define GIMBAL_COMPATIBLE
#define CHASSIS_COMPATIBLE

// #define CV_COMPATIBLE

#define SLIDE_COMPATIBLE
#define WRIST_COMPATIBLE
#define GRABBER_COMPATIBLE

// #define TURRET_HAS_IMU
#define GIMBAL_UNTETHERED  // I don't think this refers to the gimbal subsystem itself but rather a behavior of the gimbal

static constexpr SongTitle STARTUP_SONG = SongTitle::CRAB_RAVE;

/**
 * @brief Defines the number of motors created for the chassis.
 */
static constexpr uint8_t DRIVEN_WHEEL_COUNT = 4;
static constexpr uint8_t MOTORS_PER_WHEEL = 1;

static constexpr uint8_t SHOOTER_MOTOR_COUNT = 2;

/**
 * @brief GIMBAL SETUP
 */
static constexpr CANBus YAW_GIMBAL_BUS = CANBus::CAN_BUS1;
static constexpr CANBus PITCH_GIMBAL_BUS = CANBus::CAN_BUS1;
static constexpr CANBus WRIST_BUS = CANBus::CAN_BUS1;
static constexpr tap::gpio::Pwm::Pin GRABBER_PIN = tap::gpio::Pwm::C1;

static constexpr uint8_t YAW_MOTOR_COUNT = 2;
static constexpr uint8_t WRIST_MOTOR_COUNT = 3;
static constexpr uint8_t PITCH_MOTOR_COUNT = 1;

static const std::array<bool, YAW_MOTOR_COUNT> YAW_MOTOR_DIRECTIONS = {false, false};
static const std::array<MotorID, YAW_MOTOR_COUNT> YAW_MOTOR_IDS = {MotorID::MOTOR6, MotorID::MOTOR7};
static const std::array<const char*, YAW_MOTOR_COUNT> YAW_MOTOR_NAMES = {"Yaw Motor 1", "Yaw Motor 2"};
static const std::array<float, YAW_MOTOR_COUNT> YAW_MOTOR_OFFSET_ANGLES = {
    wrapTo0To2PIRange(modm::toRadian(186.15f)),
    wrapTo0To2PIRange(modm::toRadian(196.21f))};

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

// Ignore all of these gimbal constants, last year's refactor did an oopsie and requires these for non-gimbal robots
static constexpr float YAW_AXIS_START_ANGLE = modm::toRadian(0.0f);

static constexpr float GIMBAL_YAW_GEAR_RATIO = (1.0f / 2.0f);  // for 2023 Standard
/*Changing this means the encoder-readable range of the YAW axis is reduced to 360deg * GIMBAL_YAW_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the robot will be started within the same GIMBAL_YAW_GEAR_RATIO range
 * every time. We also assume that 1 / GIMBAL_YAW_GEAR_RATIO is an integer multiple of 360deg. */

static const std::array<bool, PITCH_MOTOR_COUNT> PITCH_MOTOR_DIRECTIONS = {true};
static const std::array<MotorID, PITCH_MOTOR_COUNT> PITCH_MOTOR_IDS = {MotorID::MOTOR6};
static const std::array<const char*, PITCH_MOTOR_COUNT> PITCH_MOTOR_NAMES = {"Pitch Motor 1"};
static const std::array<float, YAW_MOTOR_COUNT> PITCH_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(-144.88f))};
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static constexpr float PITCH_AXIS_START_ANGLE = modm::toRadian(0.0f);

static constexpr float GIMBAL_PITCH_GEAR_RATIO = (30.0f / 102.0f);  // for 2023 Standard
/*Changing this means the encoder-readable range of the PITCH axis is reduced to 360deg * GIMBAL_PITCH_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the range of the pitch axis is hardware-limited to not exceed this
 * range, but the motor angle may cross 0 in this range. Example Range: 278deg to 28deg */

static constexpr float PITCH_AXIS_SOFTSTOP_LOW = modm::toRadian(-24.0f);
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = modm::toRadian(22.0f);
// LOW should be lesser than HIGH, otherwise switch the motor direction
//----------------------------------------------------------------------------------------------------------------

static constexpr SmoothPIDConfig SLIDE_X_POSITION_PID_CONFIG = {
    .kp = 30000.0f,
    .ki = 0.0f,
    .kd = 10.0f,
    .maxICumulative = 0.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig SLIDE_Z_POSITION_PID_CONFIG = {
    .kp = 60000.0f,
    .ki = 10.0f,
    .kd = 8.0f,
    .maxICumulative = 3000.0f,
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
    .kp = 10000.0f,
    .ki = 0.0f,
    .kd = 0.7f,
    .maxICumulative = 0.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f};

static constexpr SmoothPIDConfig PITCH_POSITION_PID_CONFIG = {
    .kp = 10000.0f,
    .ki = 0.0f,
    .kd = 0.7f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig ROLL_POSITION_PID_CONFIG = {
    .kp = 10000.0f,
    .ki = 0.0f,
    .kd = 0.7f,
    .maxICumulative = 10.0f,
    .maxOutput = M2006_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

// VISION PID CONSTANTS
static constexpr SmoothPIDConfig YAW_POSITION_CASCADE_PID_CONFIG = {
    .kp = 20.0f,  // 30
    .ki = 0.0f,
    .kd = 0.0f,
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
    .kp = 10000.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f};

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

static constexpr SmoothPIDConfig ROLL_VELOCITY_PID_CONFIG = {
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

static Vector3f IMU_MOUNT_POSITION{0.0992f, 0.0f, 0.0534f};

static constexpr SmoothPIDConfig CHASSIS_VELOCITY_PID_CONFIG = {
    .kp = 18.0f,
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

static constexpr SmoothPIDConfig WRIST_POSITION_PID_CONFIG = {
    .kp = 3000.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

// 1 for no symmetry, 2 for 180 degree symmetry, 4 for 90 degree symmetry
static constexpr uint8_t CHASSIS_SNAP_POSITIONS = 2;

static constexpr int PROJECTILES_PER_FEEDER_ROTATION = 1;  // Engineer with a glock

// TODO: set these to what they actually are
static constexpr uint8_t SLIDE_MOTOR_COUNT = 2;
static constexpr CANBus SLIDE_BUS = CANBus::CAN_BUS1;
static constexpr MotorID SLIDE_X_MOTOR_ID = MotorID::MOTOR4;
static constexpr MotorID SLIDE_Z_MOTOR_ID = MotorID::MOTOR5;
static constexpr bool SLIDE_X_MOTOR_DIRECTION = false;
static constexpr bool SLIDE_Z_MOTOR_DIRECTION = true;
static constexpr float SLIDE_METERS_PER_REVS_RATIOS[]{(0.254f / 46.0f), (0.184f / 130.0f)};
static constexpr float SLIDE_MAX_POSITIONS_METERS[] = {0.254f, 0.18f};

// CAN Bus 2
static constexpr CANBus CHASSIS_BUS = CANBus::CAN_BUS2;

static constexpr MotorID LEFT_BACK_WHEEL_ID = MotorID::MOTOR1;
static constexpr MotorID LEFT_FRONT_WHEEL_ID = MotorID::MOTOR2;
static constexpr MotorID RIGHT_FRONT_WHEEL_ID = MotorID::MOTOR3;
static constexpr MotorID RIGHT_BACK_WHEEL_ID = MotorID::MOTOR4;

// Mechanical chassis constants, all in m
/**
 * Radius of the wheels (m).
 */
static constexpr float WHEEL_RADIUS = 0.076f;

static constexpr float WHEELBASE_WIDTH = 0.3849f;

static constexpr float WHEELBASE_LENGTH = 0.3284f;

static constexpr float GIMBAL_X_OFFSET = 0.0f;
static constexpr float GIMBAL_Y_OFFSET = 0.0f;

static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

// Power limiting variables, explained in power_limiter.hpp
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
    (MAX_WHEEL_SPEED_SINGLE_MOTOR - MIN_WHEEL_SPEED_SINGLE_MOTOR) / (MAX_CHASSIS_POWER - MIN_CHASSIS_POWER);
static_assert(WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE >= 0);

/**
 * @brief Behavior constants for chassis
 */

/**
 * The minimum desired wheel speed for chassis rotation, measured in RPM before
 * we start slowing down translational speed.
 */
static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;

static constexpr float FOLLOW_GIMBAL_ANGLE_THRESHOLD = modm::toRadian(20.0f);

static constexpr SmoothPIDConfig ROTATION_POSITION_PID_CONFIG = {
    .kp = 1.65f,  // 1.25f
    .ki = 0.0f,
    .kd = 0.005f,  // 0.03f
    .maxICumulative = 0.1f,
    .maxOutput = 1.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

/**
 * @brief Transformation Matrices, specific to robot
 */

// clang-format off
static Vector3f CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN{ // in meters
    -0.002201f, // x
    0.1348f, // y
    -0.0498f,  // z
};

static Vector3f TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN{
    0.0f, // x
    0.0f, // y
    0.0f  // z
};

static Vector3f CHASSIS_START_POSITION_RELATIVE_TO_WORLD{
    0.0f, // x
    0.0f, // y
    0.0f, // z
};

static Vector3f BARREL_POSITION_FROM_GIMBAL_ORIGIN{
    -0.001727f, //x = 0.04498
    0.0f, //y - does not matter too much because projectile comes out this axis
    -0.00587f, //z = 0.01683
};
// clang-format on

static constexpr float CHASSIS_START_ANGLE_WORLD = modm::toRadian(0.0f);  // theta (about z axis)

static constexpr float CIMU_CALIBRATION_EULER_X = modm::toRadian(0.0f);
static constexpr float CIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float CIMU_CALIBRATION_EULER_Z = modm::toRadian(90.0f);

static constexpr float TIMU_CALIBRATION_EULER_X = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Z = modm::toRadian(0.0f);

// This array holds the IDs of all speed monitor barrels on the robot
static const std::array<BarrelID, 1> BARREL_IDS = {BarrelID::TURRET_17MM_1};

static constexpr size_t PROJECTILE_SPEED_QUEUE_SIZE = 10;