#pragma once

#include <modm/math/filter/pid.hpp>
#include <tap/architecture/clock.hpp>
#include <tap/architecture/periodic_timer.hpp>
#include <tap/architecture/timeout.hpp>
#include <tap/communication/can/can_bus.hpp>
#include <tap/communication/serial/uart.hpp>
#include <tap/control/command.hpp>
#include <tap/control/comprised_command.hpp>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/gpio/digital.hpp"  //maybe not
#include "tap/communication/serial/remote.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/motor/servo.hpp"

#include "modm/container/deque.hpp"
#include "modm/math/geometry/vector.hpp"
#include "modm/math/interpolation/linear.hpp"
#include "modm/math/matrix.hpp"
#include "pid/smooth_pid_wrap.hpp"

inline float pow2(float x) { return x * x; }

static inline float wrapTo0To2PIRange(float angle) {
    if (angle < 0.0f) {
        return angle + M_TWOPI;
    } else if (angle > M_TWOPI) {
        return angle - M_TWOPI;
    } else {
        return angle;
    }
}

template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

#define REMAP(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define RPM_TO_RADPS(x) (x * M_TWOPI / 60.0f)
#define RADPS_TO_RPM(x) (x * 60.0f / M_TWOPI)

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
using DJIMotor = tap::mock::DjiMotorMock;
#else
#include "tap/motor/dji_motor.hpp"
using DJIMotor = tap::motor::DjiMotor;
#endif

enum class AngleUnit : uint8_t {
    Degrees,
    Radians,
};

// if this looks cursed, that's because it is.
// currently, we're using a 1x3 matrix for X, Y, TIME patrol coordinates and also X, Y, Z location coordinates.
// ideally, we'd use a 1x4 matrix for patrol coordinates but we don't require Z right now. will change later if pitch
// patrol becomes field-relative
enum Dimensions { X = 0, Y = 1, Z = 2, TIME = 2 };

// :)
enum LinearAxis : uint8_t { X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2 };

static constexpr float DS3218_MIN_PWM = 0.1325f;
static constexpr float DS3218_MAX_PWM = 0.85f;

static constexpr float M3508_MAX_OUTPUT = 16'384.0f;
static constexpr float M2006_MAX_OUTPUT = 10'000.0f;
static constexpr float GM6020_MAX_OUTPUT = 30'000.0f;

static constexpr float GM6020_VELOCITY_FILTER_ALPHA = 1.0f;

static constexpr uint32_t MICROSECONDS_PER_SECOND = 1000000;
static constexpr uint32_t MICROSECONDS_PER_MS = 1000;

using StockPID = modm::Pid<float>;
using SmoothPID = src::Utils::SmoothPIDWrapper;
using SmoothPIDConfig = tap::algorithms::SmoothPidConfig;

using TapCommand = tap::control::Command;
using TapComprisedCommand = tap::control::ComprisedCommand;

using ChassisPowerLimiter = tap::control::chassis::PowerLimiter;

using MotorID = tap::motor::MotorId;

using CANBus = tap::can::CanBus;

using UartPort = tap::communication::serial::Uart::UartPort;

using Remote = tap::communication::serial::Remote;

using PeriodicMilliTimer = tap::arch::PeriodicMilliTimer;
using PeriodicMicroTimer = tap::arch::PeriodicMicroTimer;
using MilliTimeout = tap::arch::MilliTimeout;
using MicroTimeout = tap::arch::MicroTimeout;

using Servo = tap::motor::Servo;
using InputPins = tap::gpio::Digital::InputPin;

template <typename T, size_t ROWS, size_t COLUMNS>
using Matrix = modm::Matrix<T, ROWS, COLUMNS>;
template <typename T, std::uint8_t N>
using Vector = modm::Vector<T, N>;

using Matrix3f = modm::Matrix3f;
using Matrix4f = modm::Matrix4f;
using Vector3f = modm::Vector3f;
using Vector4f = modm::Vector4f;
template <typename T, std::size_t N>
using Deque = modm::BoundedDeque<T, N>;

template <class... Args>
using DJIMotorFunc = void (DJIMotor::*)(Args...);

static inline void scheduleIfNotScheduled(tap::control::CommandScheduler& scheduler, tap::control::Command* cmd) {
    if (!scheduler.isCommandScheduled(cmd)) {
        scheduler.addCommand(cmd);
    }
}

static inline void descheduleIfScheduled(
    tap::control::CommandScheduler& scheduler,
    tap::control::Command* cmd,
    bool interrupted) {
    if (scheduler.isCommandScheduled(cmd)) {
        scheduler.removeCommand(cmd, interrupted);
    }
}

// clang-format off
const modm::Pair<float, float> GM6020_FEEDFORWARD_VELOCITIES[11] = {
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

const modm::interpolation::Linear<modm::Pair<float, float>> GM6020_VELOCITY_FEEDFORWARD(GM6020_FEEDFORWARD_VELOCITIES, 11);
