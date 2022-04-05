#include "subsystems/feeder/feeder.hpp"

// static float pidOut;
namespace src::Feeder {

FeederSubsystem::FeederSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      feederVelPID(20, 0, 0, 0, 8000, 1, 0, 1, 0),
      targetRPM(0),
      desiredOutput(0),
      feederMotor(drivers, FEEDER_ID, FEED_BUS, false, "Feeder Motor") {
}

void FeederSubsystem::initialize() {
    feederMotor.initialize();
}

// refreshes the velocity PID given the target RPM and the current RPM
void FeederSubsystem::refresh() {
    updateMotorVelocityPID();

    setDesiredOutput();
}

void FeederSubsystem::updateMotorVelocityPID() {
    float err = targetRPM - feederMotor.getShaftRPM();
    feederVelPID.runController(err, feederVelPID.runControllerDerivateError(err, 1), 1);
    desiredOutput = feederVelPID.getOutput();
}

float FeederSubsystem::setTargetRPM(float rpm) {
    targetRPM = rpm;
    return targetRPM;
}

void FeederSubsystem::setDesiredOutput() {  // takes the input from the velocity PID and sets the motor to that RPM
    feederMotor.setDesiredOutput(static_cast<int32_t>(desiredOutput));
}
}  // namespace src::Feeder
