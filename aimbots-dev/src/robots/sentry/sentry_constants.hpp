#pragma once

/**
 * Velocity PID gains and constants.
 */
static constexpr float VELOCITY_PID_KP = 20.0f;
static constexpr float VELOCITY_PID_KI = 0.2f;
static constexpr float VELOCITY_PID_KD = 0.0f;
static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 5000.0f;
/**
 * This max output is measured in the c620 robomaster translated current.
 * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
 * The corresponding speed controller output torque current range is
 * -20 ~ 0 ~ 20 A.
 */
static constexpr float VELOCITY_PID_MAX_OUTPUT = 16000.0f;

/**
 * Rotation PID:
 * A PD controller for chassis autorotation PID that runs on error between
 * chassis rotation error.
 *
 * Description of controller:
 * - First runs kalman filter on the input angle error. All the error calculations in
 *   the controller uses this kalman filtered gain.
 * - Next, calculates the proportional term using the kalman filtered angle.
 *   Also uses kalman filtered angle and previous kalman filtered angle for the
 *   derivative term; however, the derivative term will be calculated only if the
 *   filtered angle is greater than `MIN_ERROR_ROTATION_D`.
 * - The wheel speed is calculated by then adding p and d terms and clamping the output
 *   to `MAX_WHEEL_SPEED_SINGLE_MOTOR`.
 *
 * The P gain is specified by the user and thus is not specified below.
 */
static constexpr float CHASSIS_REVOLVE_PID_MAX_P = 3000;
/**
 * Derivative term used in chassis PID.
 */
static constexpr float CHASSIS_REVOLVE_PID_KD = 500.0f;
/**
 * Derivative max term.
 */
static constexpr float CHASSIS_REVOLVE_PID_MAX_D = 3500.0f;
/**
 * The maximum revolve error before we start using the derivative term.
 */
static constexpr int MIN_ERROR_ROTATION_D = 0;

/**
 * The maximum output allowed out of the rotation PID controller.
 */
static constexpr float MAX_OUTPUT_ROTATION_PID = 4000.0f;

// mechanical chassis constants, all in m
/**
 * Radius of the wheels (m).
 */
static constexpr float WHEEL_RADIUS = 0.076;
/**
 * Distance from center of the two front wheels (m).
 */
static constexpr float WIDTH_BETWEEN_WHEELS_Y = 0.366f;
/**
 * Distance from center of the front and rear wheels (m).
 */
static constexpr float WIDTH_BETWEEN_WHEELS_X = 0.366f;
/**
 * Gimbal offset from the center of the chassis, see note above for explanation of x and y (m).
 */
static constexpr float GIMBAL_X_OFFSET = 0.0f;
/**
 * @see `GIMBAL_X_OFFSET`.
 */
static constexpr float GIMBAL_Y_OFFSET = 0.0f;
static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

static constexpr MotorID RAIL_MOTOR_ID = MotorID::MOTOR1;

/**
 * Max wheel speed, measured in RPM of the encoder (rather than shaft)
 * we use this for wheel speed since this is how dji's motors measures motor speed.
 */
static const int MAX_WHEEL_SPEED_SINGLE_MOTOR = 7000;

// Power limiting constants, will explain later
static constexpr float MAX_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 40.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 5;
static constexpr uint16_t POWER_CONSUMPTION_THRESHOLD = 20;
static constexpr float CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING = 30000;