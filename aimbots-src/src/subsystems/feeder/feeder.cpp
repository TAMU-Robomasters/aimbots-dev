#include "subsystems/feeder/feeder.hpp"
#ifndef ENGINEER
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
int16_t heat1 = 0;
int16_t heat2 = 0;

int16_t heatmax1 = 0;
int16_t heatmax2 = 0;

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

bool FeederSubsystem::isBarrelHeatAcceptable(float maxPercentage) {
    using RefSerialRxData = tap::communication::serial::RefSerial::Rx;
    auto turretData = drivers->refSerial.getRobotData().turret;

    uint16_t lastHeat = 0;
    uint16_t heatLimit = 0;

    heat2 = 0;

    auto launcherID = turretData.launchMechanismID;
    switch (launcherID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1: {
            lastHeat = turretData.heat17ID1;
            heatLimit = turretData.heatLimit17ID1;
            heat2 = 1;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_17MM_2: {
            lastHeat = turretData.heat17ID2;
            heatLimit = turretData.heatLimit17ID2;
            heat2 = 2;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_42MM: {
            lastHeat = turretData.heat42;
            heatLimit = turretData.heatLimit42;
            heat2 = 3;
            break;
        }
        default:
            break;
    }

    heat1 = lastHeat;
    heatmax1 = heatLimit;

    return (lastHeat <= (static_cast<float>(heatLimit) * maxPercentage));
}

}  // namespace src::Feeder
#endif