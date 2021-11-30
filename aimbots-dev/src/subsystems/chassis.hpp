#pragma once

#include <tap/algorithms/smooth_pid.hpp>
#include <tap/communication/can/can_bus.hpp>
#include <tap/control/subsystem.hpp>
#include <tap/drivers.hpp>
#include <tap/motor/dji_motor.hpp>

namespace Chassis {

class ChassisSubsystem : public tap::control::Subsystem {
    using DJIMotor = tap::motor::DjiMotor;
    using DJIMotorID = tap::motor::MotorId;

    using CanBus = tap::can::CanBus;
    using PID = tap::algorithms::SmoothPid;

   public:
    ChassisSubsystem(tap::Drivers* drivers);

    void initialize() override;

    void refresh() override;

    void calculateMecanumTargets(double x, double y, double theta);

    void setPower();

   private:
    DJIMotor lfMotor, lbMotor, rfMotor, rbMotor;
    PID lfControl, lbControl, rfControl, rbControl;

    float wheelTargets[4];

    // TODO: Maybe split this up for all motors?
    static constexpr CanBus CAN_BUS = CanBus::CAN_BUS1;

    static constexpr DJIMotorID LF_MOTOR_ID = DJIMotorID::MOTOR1;
    static constexpr DJIMotorID LB_MOTOR_ID = DJIMotorID::MOTOR2;
    static constexpr DJIMotorID RF_MOTOR_ID = DJIMotorID::MOTOR3;
    static constexpr DJIMotorID RB_MOTOR_ID = DJIMotorID::MOTOR4;

    // FIXME: Set these to their proper values
    static constexpr float VELOCITY_KP = 0.0f;
    static constexpr float VELOCITY_KI = 0.0f;
    static constexpr float VELOCITY_KD = 0.0f;

    // What are these for?
    static constexpr float Q_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float R_DERIVATIVE_KALMAN = 0.0f;
    static constexpr float Q_PROPORTIONAL_KALMAN = 0.0f;
    static constexpr float R_PROPORTIONAL_KALMAN = 0.0f;

    // What are these for?
    static constexpr float PID_MAX_CUMULATIVE = 0.0f;
    static constexpr float PID_MAX_OUTPUT = 0.0f;
};

};  // namespace Chassis
