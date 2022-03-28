#pragma once

#include <modm/math/filter/pid.hpp>
#include <tap/architecture/clock.hpp>
#include <tap/communication/can/can_bus.hpp>
#include <tap/control/command.hpp>

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

using StockPID = modm::Pid<float>;
using SmoothPID = src::utils::SmoothPIDWrapper;

using CANBus = tap::can::CanBus;

using TapCommand = tap::control::Command;
using ChassisPowerLimiter = tap::control::chassis::PowerLimiter;

using MotorID = tap::motor::MotorId;

using TapCommand = tap::control::Command;

using Remote = tap::communication::serial::Remote;

// using clock = tap::arch::clock;
template <typename T, uint8_t ROWS, uint8_t COLUMNS>
using Matrix = modm::Matrix<T, ROWS, COLUMNS>;

template <class... Args>
using DJIMotorFunc = void (DJIMotor::*)(Args...);

// template <uint32_t f>
// using bitToFloat = std::bit_cast<float>(f);
