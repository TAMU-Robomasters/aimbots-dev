#include "chassis.hpp"

namespace Chassis {

ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      lbMotor(
          drivers,
          LB_MOTOR_ID,
          CAN_BUS,
          false,
          "LB_MOTOR"),
      lfMotor(
          drivers,
          LF_MOTOR_ID,
          CAN_BUS,
          false,
          "LF_MOTOR"),
      rfMotor(
          drivers,
          RF_MOTOR_ID,
          CAN_BUS,
          false,
          "RF_MOTOR"),
      rbMotor(
          drivers,
          RB_MOTOR_ID,
          CAN_BUS,
          false,
          "RB_MOTOR"),
      lbControl(
          VELOCITY_KP,
          VELOCITY_KI,
          VELOCITY_KD,
          PID_MAX_CUMULATIVE,
          PID_MAX_OUTPUT,
          Q_DERIVATIVE_KALMAN,
          R_DERIVATIVE_KALMAN,
          Q_PROPORTIONAL_KALMAN,
          R_DERIVATIVE_KALMAN),
      lfControl(
          VELOCITY_KP,
          VELOCITY_KI,
          VELOCITY_KD,
          PID_MAX_CUMULATIVE,
          PID_MAX_OUTPUT,
          Q_DERIVATIVE_KALMAN,
          R_DERIVATIVE_KALMAN,
          Q_PROPORTIONAL_KALMAN,
          R_DERIVATIVE_KALMAN),
      rfControl(
          VELOCITY_KP,
          VELOCITY_KI,
          VELOCITY_KD,
          PID_MAX_CUMULATIVE,
          PID_MAX_OUTPUT,
          Q_DERIVATIVE_KALMAN,
          R_DERIVATIVE_KALMAN,
          Q_PROPORTIONAL_KALMAN,
          R_DERIVATIVE_KALMAN),
      rbControl(
          VELOCITY_KP,
          VELOCITY_KI,
          VELOCITY_KD,
          PID_MAX_CUMULATIVE,
          PID_MAX_OUTPUT,
          Q_DERIVATIVE_KALMAN,
          R_DERIVATIVE_KALMAN,
          Q_PROPORTIONAL_KALMAN,
          R_DERIVATIVE_KALMAN) {}

void ChassisSubsystem::calculateMecanumTargets(double x, double y, double theta) {
    // Calculate the rotation matrix for the robot.
    double c = cos(theta);
    double s = sin(theta);

    // Calculate the powers for each motor.
    double lbPower = y - x - (c * s);
    double lfPower = y + x + (c * s);
    double rfPower = y - x + (c * s);
    double rbPower = y + x - (c * s);

    // Set the motor powers.
    wheelTargets[0] = lbPower;
    wheelTargets[1] = lfPower;
    wheelTargets[2] = rfPower;
    wheelTargets[3] = rbPower;
}

void ChassisSubsystem::setPower() {  // Sets the power
    lbMotor.setDesiredOutput(wheelTargets[0]);
    lfMotor.setDesiredOutput(wheelTargets[1]);
    rfMotor.setDesiredOutput(wheelTargets[2]);
    rbMotor.setDesiredOutput(wheelTargets[3]);
}

void ChassisSubsystem::initialize() {
    lbMotor.initialize();
    lfMotor.initialize();
    rfMotor.initialize();
    rbMotor.initialize();
}

void ChassisSubsystem::refresh() {
}

};  // namespace Chassis