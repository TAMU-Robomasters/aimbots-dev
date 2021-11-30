#pragma once

#include <modm/math/filter/pid.hpp>
#include <tap/algorithms/smooth_pid.hpp>
#include <tap/communication/can/can_bus.hpp>
#include "tap/control/chassis/power_limiter.hpp"
#include <tap/motor/dji_motor.hpp>

using StockPID = modm::Pid<float>;
using SmoothPID = tap::algorithms::SmoothPid;

using CANBus = tap::can::CanBus;

using ChassisPowerLimiter = tap::control::chassis::PowerLimiter;

using MotorID = tap::motor::MotorId;
using DJIMotor = tap::motor::DjiMotor;