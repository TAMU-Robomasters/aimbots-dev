#pragma once

#include <modm/math/filter/pid.hpp>
#include <tap/architecture/clock.hpp>
#include <tap/communication/can/can_bus.hpp>
#include <tap/communication/serial/uart.hpp>
#include <tap/control/command.hpp>
#include <tap/control/comprised_command.hpp>
#include <tap/architecture/periodic_timer.hpp>
#include <tap/architecture/timeout.hpp>

#include "pid/smooth_pid_wrap.hpp"
#include "tap/communication/gpio/digital.hpp"  //maybe not
#include "tap/communication/serial/remote.hpp"

#include "modm/math/matrix.hpp"
#include "tap/control/chassis/power_limiter.hpp"

// #include <bit_cast>

#include "tap/algorithms/math_user_utils.hpp"

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

enum Dimensions {
    X = 0,
    Y = 1,
    Z = 2,
    TIME = 2
};
// if this looks cursed, that's because it is.
// currently, we're using a 1x3 matrix for X, Y, TIME patrol coordinates and also X, Y, Z location coordinates.
// ideally, we'd use a 1x4 matrix for patrol coordinates but we don't require Z right now. will change later if pitch patrol becomes field-relative

static constexpr float M3508_MAX_OUTPUT = 30000.0f;
static constexpr float M2006_MAX_OUTPUT = 10000.0f;
static constexpr float GM6020_MAX_OUTPUT = 16000.0f;

using StockPID = modm::Pid<float>;
using SmoothPID = src::utils::SmoothPIDWrapper;
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

// using clock = tap::arch::clock;
template <typename T, uint8_t ROWS, uint8_t COLUMNS>
using Matrix = modm::Matrix<T, ROWS, COLUMNS>;

template <class... Args>
using DJIMotorFunc = void (DJIMotor::*)(Args...);

using InputPins = tap::gpio::Digital::InputPin;

// template <uint32_t f>
// using bitToFloat = std::bit_cast<float>(f);
