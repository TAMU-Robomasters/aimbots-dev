#pragma once

#include <modm/math/filter/pid.hpp>
#include <tap/architecture/clock.hpp>
#include <tap/communication/can/can_bus.hpp>
#include <tap/control/command.hpp>
#include <tap/control/comprised_command.hpp>

#include "pid/smooth_pid_wrap.hpp"
#include "tap/communication/serial/remote.hpp"

// #include <bit_cast>

#include "tap/algorithms/math_user_utils.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
using DJIMotor = tap::mock::DjiMotorMock;
#else
#include "tap/motor/dji_motor.hpp"
using DJIMotor = tap::motor::DjiMotor;
#endif

#include "modm/math/matrix.hpp"
#include "tap/control/chassis/power_limiter.hpp"

static constexpr float M3508_MAX_OUTPUT = 30000.0f;
static constexpr float M2006_MAX_OUTPUT = 10000.0f;
static constexpr float GM6020_MAX_OUTPUT = 16000.0f;

using StockPID = modm::Pid<float>;
using SmoothPID = src::utils::SmoothPIDWrapper;
using SmoothPIDConfig = tap::algorithms::SmoothPidConfig;

using CANBus = tap::can::CanBus;

using TapCommand = tap::control::Command;
using TapComprisedCommand = tap::control::ComprisedCommand;
using ChassisPowerLimiter = tap::control::chassis::PowerLimiter;

using MotorID = tap::motor::MotorId;

using TapCommand = tap::control::Command;

using Remote = tap::communication::serial::Remote;

// using clock = tap::arch::clock;
template <typename T, uint8_t ROWS, uint8_t COLUMNS>
using Matrix = modm::Matrix<T, ROWS, COLUMNS>;

template <class... Args>
using DJIMotorFunc = void (DJIMotor::*)(Args...);
