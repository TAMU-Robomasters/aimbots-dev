#include "subsystems/shooter/shooter.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

#include "utils/common_types.hpp"

namespace src::Shooter {

ShooterSubsystem::ShooterSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      flywheel1(drivers, SHOOTER_1_ID, SHOOTER_BUS, true, "Flywheel One"),
      flywheel2(drivers, SHOOTER_2_ID, SHOOTER_BUS, false, "Flywheel Two"),
      flywheel1PID(50.0f, 0.0f, 0.0f, 10, 30000, 1, 1, 1, 0),
      flywheel2PID(50.0f, 0.0f, 0.0f, 10, 30000, 1, 1, 1, 0),
#ifdef TARGET_SENTRY
      flywheel3(drivers, SHOOTER_3_ID, SHOOTER_BUS, true, "Flywheel Three"),
      flywheel4(drivers, SHOOTER_4_ID, SHOOTER_BUS, false, "Flywheel Four"),
      flywheel3PID(50.0f, 0.0f, 0.0f, 10, 30000, 1, 1, 1, 0),
      flywheel4PID(50.0f, 0.0f, 0.0f, 10, 30000, 1, 1, 1, 0),
#endif
      targetRPMs(Matrix<float, SHOOTER_MOTOR_COUNT, 1>::zeroMatrix()),
      desiredOutputs(Matrix<float, SHOOTER_MOTOR_COUNT, 1>::zeroMatrix()),
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

float PIDoutDisplay = 0.0f;
float shaftSpeedDisplay = 0.0f;

// Update the actual RPMs of the motors; the calculation is called from ShooterCommand
void ShooterSubsystem::refresh() {
    ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
    shaftSpeedDisplay = 69.0f;
    if (motors[TOP][0]->isMotorOnline()) {
        shaftSpeedDisplay = motors[TOP][0]->getShaftRPM();
        PIDoutDisplay = velocityPIDs[TOP][0]->getOutput();
    }

    ForAllShooterMotors(&ShooterSubsystem::setDesiredOutput);
}

void ShooterSubsystem::updateMotorVelocityPID(MotorIndex motorIdx) {
    float err = targetRPMs[motorIdx][0] - motors[motorIdx][0]->getShaftRPM();
    float PIDOut = velocityPIDs[motorIdx][0]->runControllerDerivateError(err);
    desiredOutputs[motorIdx][0] = PIDOut;
}

void ShooterSubsystem::setTargetRPM(MotorIndex motorIdx, float targetRPM) {
    targetRPMs[motorIdx][0] = targetRPM;
}

float powerDisplay = 0.0f;

void ShooterSubsystem::setDesiredOutput(MotorIndex motorIdx) {
    powerDisplay = 4.2f;
    motors[motorIdx][0]->setDesiredOutput(static_cast<int32_t>(desiredOutputs[motorIdx][0]));
}
};  // namespace src::Shooter