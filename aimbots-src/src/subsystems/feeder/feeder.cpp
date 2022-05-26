#include "subsystems/feeder/feeder.hpp"

// static float pidOut;
namespace src::Feeder {

FeederSubsystem::FeederSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      feederVelPID(FEEDER_VELOCITY_PID_CONFIG),
      targetRPM(0),
      desiredOutput(0),
      feederMotor(drivers, FEEDER_ID, FEED_BUS, FEEDER_DIRECTION, "Feeder Motor"),
      limitSwitchLeft(static_cast<std::string>("C6"), EdgeType::RISING),
      limitSwitchRight(static_cast<std::string>("C7"), EdgeType::RISING) {
}

void FeederSubsystem::initialize() {
    feederMotor.initialize();
    limitSwitchLeft.initialize();
    limitSwitchRight.initialize();
}

// refreshes the velocity PID given the target RPM and the current RPM
void FeederSubsystem::refresh() {
    updateMotorVelocityPID();
    setDesiredOutput();
    limitSwitchLeft.refresh();
    limitSwitchRight.refresh();
}

float feederPidDisplay = 0;

void FeederSubsystem::updateMotorVelocityPID() {
    float err = targetRPM - feederMotor.getShaftRPM();
    feederVelPID.runControllerDerivateError(err);
    // feederVelPID.runController(err, feederVelPID.runControllerDerivateError(err, 1), 1);
    feederPidDisplay = feederVelPID.getOutput();
    desiredOutput = feederVelPID.getOutput();
}

float targetRPMDisplay = 0;

float FeederSubsystem::setTargetRPM(float rpm) {
    targetRPMDisplay = rpm;
    this->targetRPM = rpm;
    return targetRPM;
}

void FeederSubsystem::setDesiredOutput() {  // takes     the input from the velocity PID and sets the motor to that RPM
    feederMotor.setDesiredOutput(static_cast<int32_t>(desiredOutput));
}

int FeederSubsystem::getTotalLimitCount() const{ return limitSwitchLeft.getCurrentCount() + limitSwitchRight.getCurrentCount(); }
int FeederSubsystem::getLeftLimitCount() const{ return limitSwitchLeft.getCurrentCount(); }
int FeederSubsystem::getRightLimitCount() const{ return limitSwitchRight.getCurrentCount(); }

}  // namespace src::Feeder
