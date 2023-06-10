#include "subsystems/feeder/feeder.hpp"

namespace src::Feeder {

FeederSubsystem::FeederSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
      targetRPM(0),
      desiredOutput(0),
      feederVelPID(FEEDER_VELOCITY_PID_CONFIG),
      feederMotor(drivers, FEEDER_ID, FEED_BUS, FEEDER_DIRECTION, "Feeder Motor"),
      limitSwitchLeft(static_cast<std::string>("C6"), src::Informants::EdgeType::RISING)
#ifdef TARGET_SENTRY
      ,
      limitSwitchRight(static_cast<std::string>("C7"), src::Informants::EdgeType::RISING)
#endif
{
}

//Watch Variables
int16_t heatCurrentDisplay = 0;
int16_t barrelDDisplay = 0;

int16_t heatMaxDisplay = 0;

void FeederSubsystem::initialize() {
    feederMotor.initialize();
    limitSwitchLeft.initialize();
#ifdef TARGET_SENTRY
    limitSwitchRight.initialize();
#endif
}

// refreshes the velocity PID given the target RPM and the current RPM
void FeederSubsystem::refresh() {
    updateMotorVelocityPID();
    setDesiredOutput();
    limitSwitchLeft.refresh();
#ifdef TARGET_SENTRY
    limitSwitchRight.refresh();
#endif
}

void FeederSubsystem::updateMotorVelocityPID() {
    float err = targetRPM - feederMotor.getShaftRPM();
    feederVelPID.runControllerDerivateError(err);
    desiredOutput = feederVelPID.getOutput();
}

float FeederSubsystem::setTargetRPM(float rpm) {
    this->targetRPM = rpm;
    return targetRPM;
}

void FeederSubsystem::setDesiredOutput() {  // takes the input from the velocity PID and sets the motor to that RPM
    feederMotor.setDesiredOutput(static_cast<int32_t>(desiredOutput));
}

int FeederSubsystem::getTotalLimitCount() const {
#ifndef TARGET_SENTRY
    return limitSwitchLeft.getCurrentCount();
#endif
#ifdef TARGET_SENTRY
    return limitSwitchLeft.getCurrentCount() + limitSwitchRight.getCurrentCount();
#endif
}

}  // namespace src::Feeder