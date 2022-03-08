#include "subsystems/feeder/feeder.hpp"

// static float pidOut;
namespace src::Feeder {

FeederSubsystem::FeederSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      feederMotor(drivers, FEEDER_ID, FEED_BUS, false, "Feeder Motor"),
      feederVelPID(20, 0, 0, 0, 8000, 1, 0, 1, 0),
      targetRPM(0) {
}

void FeederSubsystem::initialize() {
    feederMotor.initialize();
}

void FeederSubsystem::refresh() {  // refreshes the velocity PID given the target RPM and the current RPM
    updateMotorVelocityPID();

    setDesiredOutput();
}

void FeederSubsystem::updateMotorVelocityPID() {
    float err = targetRPM - feederMotor.getShaftRPM();
    feederVelPID.runController(err, feederVelPID.runControllerDerivateError(err, 1), 1);
}

float FeederSubsystem::setTargetRPM(float rpm) {
    targetRPM = rpm;
    return targetRPM;
}

void FeederSubsystem::setDesiredOutput() {  // takes the input from the velocity PID and sets the motor to that RPM
    feederMotor.setDesiredOutput(feederVelPID.getOutput());
}
}  // namespace src::Feeder
