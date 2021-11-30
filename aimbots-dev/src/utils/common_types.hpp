#pragma once

#include <modm/math/filter/pid.hpp>
#include <tap/algorithms/smooth_pid.hpp>
#include <tap/communication/can/can_bus.hpp>
#include <tap/motor/dji_motor.hpp>

using CANBus = tap::can::CanBus;

using MotorID = tap::motor::MotorId;
using DJIMotor = tap::motor::DjiMotor;

using StockPID = modm::Pid<float>;
using SmoothPID = tap::algorithms::SmoothPid;