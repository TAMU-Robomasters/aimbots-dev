#include "subsystems/shooter/shooter.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

#include "utils/common_types.hpp"

namespace src::Shooter {

ShooterSubsystem::ShooterSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      flywheel1(drivers, SHOOTER_1_ID, SHOOTER_BUS, true, "Flywheel One"),
      flywheel2(drivers, SHOOTER_2_ID, SHOOTER_BUS, false, "Flywheel Two"),
      flywheel1PID(
        SHOOTER_PID_KP,
        SHOOTER_PID_KI,
        SHOOTER_PID_KD,
        SHOOTER_MAX_I_CUMULATIVE,
        SHOOTER_MAX_OUTPUT,
        SHOOTER_TQ_DERIVATIVE_KALMAN,
        SHOOTER_TR_DERIVATIVE_KALMAN,
        SHOOTER_TQ_PROPORTIONAL_KALMAN,
        SHOOTER_TR_PROPORTIONAL_KALMAN),
      flywheel2PID( SHOOTER_PID_KP,
        SHOOTER_PID_KI,
        SHOOTER_PID_KD,
        SHOOTER_MAX_I_CUMULATIVE,
        SHOOTER_MAX_OUTPUT,
        SHOOTER_TQ_DERIVATIVE_KALMAN,
        SHOOTER_TR_DERIVATIVE_KALMAN,
        SHOOTER_TQ_PROPORTIONAL_KALMAN,
        SHOOTER_TR_PROPORTIONAL_KALMAN ),
#ifdef TARGET_SENTRY
      flywheel3(drivers, SHOOTER_3_ID, SHOOTER_BUS, true, "Flywheel Three"),
      flywheel4(drivers, SHOOTER_4_ID, SHOOTER_BUS, false, "Flywheel Four"),
      flywheel3PID( SHOOTER_PID_KP,
        SHOOTER_PID_KI,
        SHOOTER_PID_KD,
        SHOOTER_MAX_I_CUMULATIVE,
        SHOOTER_MAX_OUTPUT,
        SHOOTER_TQ_DERIVATIVE_KALMAN,
        SHOOTER_TR_DERIVATIVE_KALMAN,
        SHOOTER_TQ_PROPORTIONAL_KALMAN,
        SHOOTER_TR_PROPORTIONAL_KALMAN ),
      flywheel4PID( SHOOTER_PID_KP,
        SHOOTER_PID_KI,
        SHOOTER_PID_KD,
        SHOOTER_MAX_I_CUMULATIVE,
        SHOOTER_MAX_OUTPUT,
        SHOOTER_TQ_DERIVATIVE_KALMAN,
        SHOOTER_TR_DERIVATIVE_KALMAN,
        SHOOTER_TQ_PROPORTIONAL_KALMAN,
        SHOOTER_TR_PROPORTIONAL_KALMAN ),
#endif
      targetRPMs(Matrix<float, SHOOTER_MOTOR_COUNT, 1>::zeroMatrix()),
      motors(Matrix<DJIMotor*, SHOOTER_MOTOR_COUNT, 1>::zeroMatrix()),
      velocityPIDs(Matrix<SmoothPID*, SHOOTER_MOTOR_COUNT, 1>::zeroMatrix())
//
{
    motors[TOP][0] = &flywheel1;  // TOP == RIGHT
    motors[BOT][0] = &flywheel2;  // BOT == LEFT
    velocityPIDs[TOP][0] = &flywheel1PID;
    velocityPIDs[BOT][0] = &flywheel2PID;
#ifdef TARGET_SENTRY
    motors[TOP_LEFT][0] = &flywheel3;
    motors[BOT_LEFT][0] = &flywheel4;
    velocityPIDs[TOP_LEFT][0] = &flywheel3PID;
    velocityPIDs[BOT_LEFT][0] = &flywheel4PID;
#endif
}

void ShooterSubsystem::initialize() {
    ForAllShooterMotors(&DJIMotor::initialize);

    ForAllShooterMotors(&DJIMotor::setDesiredOutput, static_cast<int32_t>(0.0f));
}

// Update the actual RPMs of the motors; the calculation is called from ShooterCommand
void ShooterSubsystem::refresh() {
    ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
}

float PIDoutDisplay = 0.0f;
float shaftSpeedDisplay = 0.0f;
// TODO: need to tune PID

void ShooterSubsystem::updateMotorVelocityPID(MotorIndex motorIdx) {
    float err = targetRPMs[motorIdx][0] - motors[motorIdx][0]->getShaftRPM();
    float PIDOut = velocityPIDs[motorIdx][0]->runControllerDerivateError(err);
    PIDoutDisplay = err;
    shaftSpeedDisplay = motors[motorIdx][0]->getShaftRPM();
    motors[motorIdx][0]->setDesiredOutput(static_cast<int32_t>(PIDOut));
}

void ShooterSubsystem::setTargetRPM(MotorIndex motorIdx, float targetRPM) {
    targetRPMs[motorIdx][0] = targetRPM;
}

};  // namespace src::Shooter