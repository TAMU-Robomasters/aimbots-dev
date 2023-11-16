#include "subsystems/feeder/feeder.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

FeederSubsystem::FeederSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
      targetRPM(0),
      desiredOutput(0),
      feederVelPID(FEEDER_VELOCITY_PID_CONFIG),
      feederMotor(drivers, FEEDER_ID, FEED_BUS, FEEDER_DIRECTION, "Feeder Motor"),
      limitSwitch(static_cast<std::string>("C6"), src::Informants::EdgeType::RISING)

{
}

// Watch Variables
int16_t heatCurrentDisplay = 0;
int16_t barrelDDisplay = 0;

int16_t heatMaxDisplay = 0;

void FeederSubsystem::initialize() {
    feederMotor.initialize();
    limitSwitch.initialize();
}

float feederDesiredOutputDisplay = 0;
float feederShaftRPMDisplay = 0;
bool isFeederOnlineDisplay = false;

// refreshes the velocity PID given the target RPM and the current RPM
void FeederSubsystem::refresh() {
    isFeederOnlineDisplay = feederMotor.isMotorOnline();
    feederDesiredOutputDisplay = targetRPM;
    feederShaftRPMDisplay = feederMotor.getShaftRPM();

    updateMotorVelocityPID();
    setDesiredOutput();
    limitSwitch.refresh();

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

bool FeederSubsystem::getPressed() {
    return limitSwitch.isFalling();
}

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE