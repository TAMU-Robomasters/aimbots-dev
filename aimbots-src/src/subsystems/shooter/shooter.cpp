#include "subsystems/shooter/shooter.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

#include "utils/common_types.hpp"

namespace src::Shooter {

ShooterSubsystem::ShooterSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      flywheel1(drivers, SHOOTER_1_ID, SHOOTER_BUS, SHOOTER_1_DIRECTION, "Flywheel One"),
      flywheel2(drivers, SHOOTER_2_ID, SHOOTER_BUS, SHOOTER_2_DIRECTION, "Flywheel Two"),
      flywheel1PID(SHOOTER_VELOCITY_PID_CONFIG),
      flywheel2PID(SHOOTER_VELOCITY_PID_CONFIG),
#ifdef TARGET_SENTRY
      flywheel3(drivers, SHOOTER_3_ID, SHOOTER_BUS, SHOOTER_3_DIRECTION, "Flywheel Three"),
      flywheel4(drivers, SHOOTER_4_ID, SHOOTER_BUS, SHOOTER_4_DIRECTION, "Flywheel Four"),
      flywheel3PID(SHOOTER_VELOCITY_PID_CONFIG),
      flywheel4PID(SHOOTER_VELOCITY_PID_CONFIG),

#endif
      targetRPMs(Matrix<float, SHOOTER_MOTOR_COUNT, 1>::zeroMatrix()),
      desiredOutputs(Matrix<int32_t, SHOOTER_MOTOR_COUNT, 1>::zeroMatrix()),
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

float FWLeft1 = 0.0f;
float FWLeft2 = 0.0f;
float FWRight1 = 0.0f;
float FWRight2 = 0.0f;

// Update the actual RPMs of the motors; the calculation is called from ShooterCommand
void ShooterSubsystem::refresh() {
    // Debug info
    if (motors[TOP][0]->isMotorOnline()) {
        shaftSpeedDisplay = motors[TOP][0]->getShaftRPM();
        PIDoutDisplay = velocityPIDs[TOP][0]->getOutput();

        FWLeft1 = motors[TOP][0]->getShaftRPM();
        FWLeft2 = motors[BOT][0]->getShaftRPM();
        FWRight1 = motors[TOP][1]->getShaftRPM();
        FWRight2 = motors[BOT][1]->getShaftRPM();
    }

    ForAllShooterMotors(&ShooterSubsystem::setDesiredOutputToMotor);
}

// Returns the speed of the shooter motor with the highest absolute value of RPM
float ShooterSubsystem::getHighestMotorSpeed() const {
    float highestMotorSpeed = 0.0f;
    for (int i = 0; i < SHOOTER_MOTOR_COUNT; i++) {
        MotorIndex mi = static_cast<MotorIndex>(i);
        if (motors[mi][0]->isMotorOnline()) {
            float motorSpeed = motors[mi][0]->getShaftRPM();
            if (fabs(motorSpeed) > highestMotorSpeed) {
                highestMotorSpeed = motorSpeed;
            }
        }
    }
    return highestMotorSpeed;
}

float ShooterSubsystem::getMotorSpeed(MotorIndex motorIdx) const {
    if (motors[motorIdx][0]->isMotorOnline()) {
        return motors[motorIdx][0]->getShaftRPM();
    }
    return 0.0f;
}

void ShooterSubsystem::updateMotorVelocityPID(MotorIndex motorIdx) {
    if (motors[motorIdx][0]->isMotorOnline()) {  // Check if motor is online when getting info from it
        float err = targetRPMs[motorIdx][0] - motors[motorIdx][0]->getShaftRPM();
        float PIDOut = velocityPIDs[motorIdx][0]->runControllerDerivateError(err);
        setDesiredOutput(motorIdx, PIDOut);
    }
}

void ShooterSubsystem::setTargetRPM(MotorIndex motorIdx, float targetRPM) {
    targetRPMs[motorIdx][0] = targetRPM;
}

float powerDisplay = 0.0f;

void ShooterSubsystem::setDesiredOutput(MotorIndex motorIdx, float desiredOutput) {
    desiredOutputs[motorIdx][0] = static_cast<int32_t>(desiredOutput);
}

void ShooterSubsystem::setDesiredOutputToMotor(MotorIndex motorIdx) {
    if (motors[motorIdx][0]->isMotorOnline()) {  // Check if motor is online when setting to it
        motors[motorIdx][0]->setDesiredOutput(desiredOutputs[motorIdx][0]);
    }
}
};  // namespace src::Shooter