#pragma once
#include "utils/common_types.hpp"

#define GIMBAL_COMPATIBLE
#define CHASSIS_COMPATIBLE
#define SHOOTER_COMPATIBLE
#define FEEDER_COMPATIBLE
#define HOPPER_LID_COMPATIBLE

//Zero wheels of the Chassis on the Aerial but we need the Chassis for the kinematic informant to work
static constexpr uint8_t DRIVEN_WHEEL_COUNT = 0;
static constexpr uint8_t MOTORS_PER_WHEEL = 1;

static constexpr uint8_t SHOOTER_MOTOR_COUNT = 2;

/**
 * @brief GIMBAL SETUP
 */
static constexpr CANBus YAW_GIMBAL_BUS = CANBus::CAN_BUS2;
static constexpr CANBus PITCH_GIMBAL_BUS = CANBus::CAN_BUS1;

static constexpr uint8_t YAW_MOTOR_COUNT = 2;
static constexpr uint8_t PITCH_MOTOR_COUNT = 1;

static const std::array<bool, YAW_MOTOR_COUNT> YAW_MOTOR_DIRECTIONS = {false, false};
static const std::array<MotorID, YAW_MOTOR_COUNT> YAW_MOTOR_IDS = {MotorID::MOTOR5, MotorID::MOTOR7};
static const std::array<const char*, YAW_MOTOR_COUNT> YAW_MOTOR_NAMES = {"Yaw Motor 1", "Yaw Motor 2"};
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static const std::array<float, YAW_MOTOR_COUNT> YAW_MOTOR_OFFSET_ANGLES = {
    wrapTo0To2PIRange(modm::toRadian(186.15f)),
    wrapTo0To2PIRange(modm::toRadian(196.21f))};
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
// "DM Richard on Discord if you see this (or just calculate the pitch stop limits yourself idc)"

// static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

static constexpr MotorID YAW_MOTOR_ID = MotorID::MOTOR5;
static constexpr MotorID PITCH_MOTOR_ID = MotorID::MOTOR6;


// Power limiting constants, will explain later
static constexpr float MAX_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 40.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 5;
static constexpr uint16_t POWER_CONSUMPTION_THRESHOLD = 20;
static constexpr float CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING = 30000;


static Vector3f IMU_MOUNT_POSITION{0.0f, 0.0f, 0.0f};
//PID Configs
static constexpr SmoothPIDConfig CHASSIS_VELOCITY_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 0.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 0.0f,
    .tRProportionalKalman = 0.0f,
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
    .ki = 0.10f,  // 0.10f;
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

// This array holds the IDs of all speed monitor barrels on the robot
static const std::array<BarrelID, 1> BARREL_IDS = {BarrelID::TURRET_17MM_1};

static constexpr size_t PROJECTILE_SPEED_QUEUE_SIZE = 10;


// clang-format off
static constexpr uint16_t shooter_speed_array[6] = {  // ONLY TUNE WITH FULL BATTERY
    15,
    4300,  // {ball m/s, flywheel rpm}
    18,
    4850,
    30,
    7050};
// clang-format on
static constexpr float FEEDER_DEFAULT_RPM = 4150.0f;  // 4500
static constexpr int DEFAULT_BURST_LENGTH = 5;        // balls

// CAN Bus 2
static constexpr CANBus CHASSIS_BUS = CANBus::CAN_BUS2;



// CAN Bus 1
static constexpr CANBus SHOOTER_BUS = CANBus::CAN_BUS1;
static constexpr CANBus FEED_BUS = CANBus::CAN_BUS1;
static constexpr CANBus BARREL_BUS = CANBus::CAN_BUS1;  // TODO: check CAN ID for Barrel Swap

//
static constexpr MotorID FEEDER_ID = MotorID::MOTOR7;
//
static constexpr MotorID SHOOTER_1_ID = MotorID::MOTOR3;
static constexpr MotorID SHOOTER_2_ID = MotorID::MOTOR4;
//
static constexpr MotorID SWAP_MOTOR_ID = MotorID::MOTOR1;  // TODO: check motor ID for Barrel Swap

static constexpr bool SHOOTER_1_DIRECTION = false;
static constexpr bool SHOOTER_2_DIRECTION = true;


static constexpr bool FEEDER_DIRECTION = false;

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


//These values will change when we look at the bot
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


static constexpr float CHASSIS_START_ANGLE_WORLD = modm::toRadian(0.0f);  // theta (about z axis)

static constexpr float CIMU_CALIBRATION_EULER_X = modm::toRadian(180.0f);
static constexpr float CIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float CIMU_CALIBRATION_EULER_Z = modm::toRadian(90.0f);

static constexpr float TIMU_CALIBRATION_EULER_X = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Z = modm::toRadian(0.0f);

