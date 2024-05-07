#pragma once
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

#define GIMBAL_COMPATIBLE
#define CHASSIS_COMPATIBLE
#define SHOOTER_COMPATIBLE
#define FEEDER_COMPATIBLE

// #define TURRET_HAS_IMU
#define GIMBAL_UNTETHERED

static constexpr SongTitle STARTUP_SONG = SongTitle::WE_ARE_NUMBER_ONE;

/**
 * @brief Defines the number of motors created for the chassis.
 */
static constexpr uint8_t DRIVEN_WHEEL_COUNT = 4;
static constexpr uint8_t MOTORS_PER_WHEEL = 1;

static constexpr uint8_t SHOOTER_MOTOR_COUNT = 4;

/**
 * @brief GIMBAL SETUP
 */
static constexpr CANBus YAW_GIMBAL_BUS = CANBus::CAN_BUS2;
static constexpr CANBus PITCH_GIMBAL_BUS = CANBus::CAN_BUS1;

static constexpr uint8_t YAW_MOTOR_COUNT = 1;
static constexpr uint8_t PITCH_MOTOR_COUNT = 1;

static const std::array<bool, YAW_MOTOR_COUNT> YAW_MOTOR_DIRECTIONS = {false};
static const std::array<MotorID, YAW_MOTOR_COUNT> YAW_MOTOR_IDS = {MotorID::MOTOR5};
static const std::array<const char*, YAW_MOTOR_COUNT> YAW_MOTOR_NAMES = {"Yaw Motor 1"};
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static const std::array<float, YAW_MOTOR_COUNT> YAW_MOTOR_OFFSET_ANGLES = {
    wrapTo0To2PIRange(modm::toRadian(42.58f))};  // 198.2
static constexpr float YAW_AXIS_START_ANGLE = modm::toRadian(0.0f);

static constexpr float GIMBAL_YAW_GEAR_RATIO = (1.0f / 2.0f);  // for 2023 Sentry
/*Changing this means the encoder-readable range of the YAW axis is reduced to 360deg * GIMBAL_YAW_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the robot will be started within the same GIMBAL_YAW_GEAR_RATIO range
 * every time. We also assume that 1 / GIMBAL_YAW_GEAR_RATIO is an integer multiple of 360deg. */

static const std::array<bool, PITCH_MOTOR_COUNT> PITCH_MOTOR_DIRECTIONS = {true};
static const std::array<MotorID, PITCH_MOTOR_COUNT> PITCH_MOTOR_IDS = {MotorID::MOTOR6};
static const std::array<const char*, PITCH_MOTOR_COUNT> PITCH_MOTOR_NAMES = {"Pitch Motor 1"};
static const std::array<float, PITCH_MOTOR_COUNT> PITCH_MOTOR_OFFSET_ANGLES = {
    wrapTo0To2PIRange(modm::toRadian(229.0f))};  // 60.6
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static constexpr float PITCH_AXIS_START_ANGLE = modm::toRadian(0.0f);

static constexpr float GIMBAL_PITCH_GEAR_RATIO = (30.0f / 102.0f);  // for 2023 Sentry
/*Changing this means the encoder-readable range of the PITCH axis is reduced to 360deg * GIMBAL_PITCH_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the range of the pitch axis is hardware-limited to not exceed this
 * range, but the motor angle may cross 0 in this range. Example Range: 278deg to 28deg */

static constexpr float PITCH_AXIS_SOFTSTOP_LOW = modm::toRadian(-9.0f);
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = modm::toRadian(30.0f);
// LOW should be lesser than HIGH, otherwise switch the motor direction

/**
 * @brief Position PID constants
 */
static constexpr SmoothPIDConfig YAW_POSITION_PID_CONFIG = {
    .kp = 20'000.0f,
    .ki = 0.0f,
    .kd = 10.0f,
    .maxICumulative = 0.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
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
    .kp = 2200.0f,
    .ki = 25.0f,
    .kd = 0.0f,
    .maxICumulative = 2000.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
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

static constexpr float CHASSIS_VELOCITY_YAW_LOAD_FEEDFORWARD = 1.0f;
static constexpr float CHASSIS_VELOCITY_PITCH_LOAD_FEEDFORWARD = 1.0f;

static constexpr float CHASSIS_LINEAR_ACCELERATION_PITCH_COMPENSATION = 0.0f;
static constexpr float kGRAVITY = 0.0f;
static constexpr float HORIZON_OFFSET = 0.0f;

// clang-format off
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
                                                                    {29.2f, 30'000.0f}
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

// -----------------------------------------------------------------------------------------------------------------------
static Vector3f IMU_MOUNT_POSITION{0.0f, 0.0f, 0.0f};

static constexpr SmoothPIDConfig CHASSIS_VELOCITY_PID_CONFIG = {
    .kp = 18.0f,
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

static constexpr SmoothPIDConfig FEEDER_VELOCITY_PID_CONFIG = {
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

static constexpr int UNJAM_TIMER_MS = 300;

static constexpr SmoothPIDConfig SHOOTER_VELOCITY_PID_CONFIG = {
    .kp = 40.0f,
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

// 1 for no symmetry, 2 for 180 degree symmetry, 4 for 90 degree symmetry
static constexpr uint8_t CHASSIS_SNAP_POSITIONS = 4;

// clang-format off
// Sentry shoots at the speed of death
static constexpr uint16_t shooter_speed_array[2] = {30, 7450};  // {m/s, rpm}
// clang-format on

static const Matrix<uint16_t, 1, 2> SHOOTER_SPEED_MATRIX(shooter_speed_array);

static constexpr float FEEDER_DEFAULT_RPM = 4150.0f;

static constexpr uint8_t PROJECTILES_PER_FEEDER_ROTATION = 19;
static constexpr uint8_t FEEDER_GEAR_RATIO = 36;

static constexpr int DEFAULT_BURST_LENGTH = 10;  // total balls in burst

// CAN Bus 2
static constexpr CANBus CHASSIS_BUS = CANBus::CAN_BUS2;

static constexpr MotorID LEFT_BACK_WHEEL_ID = MotorID::MOTOR1;
static constexpr MotorID LEFT_FRONT_WHEEL_ID = MotorID::MOTOR2;
static constexpr MotorID RIGHT_FRONT_WHEEL_ID = MotorID::MOTOR3;
static constexpr MotorID RIGHT_BACK_WHEEL_ID = MotorID::MOTOR4;

// CAN Bus 1
static constexpr CANBus SHOOTER_BUS = CANBus::CAN_BUS1;
static constexpr CANBus FEED_BUS = CANBus::CAN_BUS1;

//
static constexpr MotorID FEEDER_ID = MotorID::MOTOR8;
//
static constexpr MotorID SHOOTER_1_ID = MotorID::MOTOR1;
static constexpr MotorID SHOOTER_2_ID = MotorID::MOTOR2;
static constexpr MotorID SHOOTER_3_ID = MotorID::MOTOR3;
static constexpr MotorID SHOOTER_4_ID = MotorID::MOTOR4;
//

static constexpr bool SHOOTER_1_DIRECTION = true;
static constexpr bool SHOOTER_2_DIRECTION = false;
static constexpr bool SHOOTER_3_DIRECTION = false;
static constexpr bool SHOOTER_4_DIRECTION = true;

static constexpr bool FEEDER_DIRECTION = false;
// Mechanical chassis constants, all in m
/**
 * Radius of the wheels (m).
 */
static constexpr float WHEEL_RADIUS = 0.07663f;

static constexpr float WHEELBASE_WIDTH = 0.357f;  // updated for 2023

static constexpr float WHEELBASE_LENGTH = 0.357f;

static constexpr float GIMBAL_X_OFFSET = 0.0f;  //-0.05
static constexpr float GIMBAL_Y_OFFSET = 0.0f;  //-0.05

static constexpr float CHASSIS_GEARBOX_RATIO = (187.0f / 3591.0f);

// Power limiting constants, will explain later
static constexpr float POWER_LIMIT_SAFETY_FACTOR = 0.85f;
static constexpr float STARTING_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 15.0f;

/**
 * @brief Power constants for chassis
 */
static constexpr int MIN_WHEEL_SPEED_SINGLE_MOTOR = 4000;
static constexpr int MAX_WHEEL_SPEED_SINGLE_MOTOR = 8000;
static constexpr int MIN_CHASSIS_POWER = 50;
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
    .kp = 1.65f,
    .ki = 0.0f,
    .kd = 0.005f,
    .maxICumulative = 0.9f,
    .maxOutput = 1.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

/**
 * @brief TOKYO CONSTANTS
 */
// Fraction that user input is multiplied by when "drifting"
static constexpr float TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER = 0.6f;
// Fraction of the maximum translation speed for when rotation speed should be reduced
static constexpr float TOKYO_TRANSLATION_THRESHOLD_TO_DECREASE_ROTATION_SPEED = 0.5f;
// Fraction of max chassis speed applied to rotation speed
static constexpr float TOKYO_ROTATIONAL_SPEED_FRACTION_OF_MAX = 0.75f;
// Fraction to cut rotation speed by when the robot is "drifting"
static constexpr float TOKYO_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING = 0.7f;
// Rotational speed increment per iteration to apply until rotation setpoint is reached
static constexpr float TOKYO_ROTATIONAL_SPEED_INCREMENT = 50.0f;  // rpm

/**
 * @brief Transformation Matrices, specific to robot
 */
// clang-format off
static Vector3f CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN{ // in meters
    0.0002f, // x
    0.04894f, // y
    0.084879f,  // z
};

static Vector3f TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN{
    0.0f, // x
    0.0f, // y
    0.0f  // z
};

static Vector3f CHASSIS_START_POSITION_RELATIVE_TO_WORLD{
    -2.830f, // x
    -0.730f, // y
    0.0f, // z
};

//0.04301 how far apart barrels are
static Vector3f BARREL_POSITION_FROM_GIMBAL_ORIGIN{
    0.01785f - (0.5f * 0.04301f), //x = 0.04498
    0.0f, //y - does not matter too much because projectile comes out this axis
    -0.00018f, //z = 0.01683
};
// clang-format on

static constexpr float CHASSIS_START_ANGLE_WORLD = modm::toRadian(0.0f);  // theta (about z axis)

static constexpr float CIMU_CALIBRATION_EULER_X = modm::toRadian(0.0f);
static constexpr float CIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float CIMU_CALIBRATION_EULER_Z = modm::toRadian(0.0f);

static constexpr float TIMU_CALIBRATION_EULER_X = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Z = modm::toRadian(0.0f);

static constexpr size_t PROJECTILE_SPEED_QUEUE_SIZE = 10;

// This array holds the IDs of all speed monitor barrels on the robot
static const std::array<BarrelID, 2> BARREL_IDS = {BarrelID::TURRET_17MM_1, BarrelID::TURRET_17MM_2};

// PITCH PATROL FUNCTION CONSTANTS
// TODO: Gimbal Control Command needs to be modified to work in radians, and then convert these constants.
static constexpr float PITCH_PATROL_AMPLITUDE = modm::toRadian(12.5f);
static constexpr float PITCH_PATROL_FREQUENCY = 1.5f * M_PI;
static constexpr float PITCH_PATROL_OFFSET = 20.0f;  // degrees offset from horizon
static constexpr float PITCH_OFFSET_ANGLE = 0;       // In degrees currently

// Sentry Chassis Travel Waypoints
static constexpr int NUMBER_OF_WAYPOINTS = 5;

enum waypointName { SENTRY_START = 0, SOUTHEAST_CORRIDOR, EAST_MIDLINE, NORTHEAST_CORRIDOR, BUFF_POINT };

static const Vector<float, 2> SENTRY_WAYPOINTS[NUMBER_OF_WAYPOINTS] = {
    {CHASSIS_START_POSITION_RELATIVE_TO_WORLD[0], CHASSIS_START_POSITION_RELATIVE_TO_WORLD[1]},  // Starting Location
    {-2.50f, -1.75f},  // North-East of starting location
    {-2.50f, 0.0f},    // Midline on East side of field
    {-2.50f, 1.75f},   // Just past midline of the field, can shoot at enemy reload zone
    {0.0f, 0.0f}};     // Central Buff Zone

static const std::array<waypointName, 3> SETUP_TO_AGGRO = {
    waypointName::SENTRY_START,
    waypointName::SOUTHEAST_CORRIDOR,
    waypointName::NORTHEAST_CORRIDOR};

static const std::array<waypointName, 3> AGGRO_TO_CAPTURE = {
    waypointName::NORTHEAST_CORRIDOR,
    waypointName::EAST_MIDLINE,
    waypointName::BUFF_POINT};

static const std::array<waypointName, 3> CAPTURE_TO_AGGRO = {
    waypointName::BUFF_POINT,
    waypointName::EAST_MIDLINE,
    waypointName::NORTHEAST_CORRIDOR};
