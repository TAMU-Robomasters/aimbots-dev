#include "subsystems/feeder/feeder.hpp"

// static float pidOut;
namespace src::Feeder {

FeederSubsystem::FeederSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      feederVelPID(FEEDER_VELOCITY_PID_CONFIG),
      targetRPM(0),
      desiredOutput(0),
      feederMotor(drivers, FEEDER_ID, FEED_BUS, FEEDER_DIRECTION, "Feeder Motor") {
}

void FeederSubsystem::initialize() {
    feederMotor.initialize();
}

// refreshes the velocity PID given the target RPM and the current RPM
void FeederSubsystem::refresh() {
    updateMotorVelocityPID();

    setDesiredOutput();
}

float feederPidDisplay = 0;

void FeederSubsystem::updateMotorVelocityPID() {
    float err = targetRPM - feederMotor.getShaftRPM();
    feederVelPID.runController(err, feederVelPID.runControllerDerivateError(err, 1), 1);
    feederPidDisplay = feederVelPID.getOutput();
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
