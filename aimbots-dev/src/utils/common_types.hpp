#pragma once

#include <modm/math/filter/pid.hpp>
#include <tap/algorithms/smooth_pid.hpp>
#include <tap/communication/can/can_bus.hpp>
#include <tap/motor/dji_motor.hpp>

#include "modm/math/matrix.hpp"
#include "tap/control/chassis/power_limiter.hpp"

using StockPID = modm::Pid<float>;
using SmoothPID = tap::algorithms::SmoothPid;

using CANBus = tap::can::CanBus;

using ChassisPowerLimiter = tap::control::chassis::PowerLimiter;

using MotorID = tap::motor::MotorId;
using DJIMotor = tap::motor::DjiMotor;

template <typename T, uint8_t ROWS, uint8_t COLUMNS>
using Matrix = modm::Matrix<T, ROWS, COLUMNS>;

typedef void (DJIMotor::*DJIMotorFunc)();