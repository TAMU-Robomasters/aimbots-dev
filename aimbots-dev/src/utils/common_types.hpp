#pragma once

#include <modm/math/filter/pid.hpp>
#include <tap/algorithms/smooth_pid.hpp>
#include <tap/communication/can/can_bus.hpp>
#include <tap/motor/dji_motor.hpp>
#include <tap/communication/serial/uart.hpp>

#include "modm/math/matrix.hpp"
#include "tap/control/chassis/power_limiter.hpp"

using StockPID = modm::Pid<float>;
using SmoothPID = tap::algorithms::SmoothPid;

using CANBus = tap::can::CanBus;
using TapUartPort = tap::serial::Uart::UartPort;

using ChassisPowerLimiter = tap::control::chassis::PowerLimiter;

using MotorID = tap::motor::MotorId;
using DJIMotor = tap::motor::DjiMotor;

template <typename T, uint8_t ROWS, uint8_t COLUMNS>
using Matrix = modm::Matrix<T, ROWS, COLUMNS>;

template <class... Args>
using DJIMotorFunc = void (DJIMotor::*)(Args...);