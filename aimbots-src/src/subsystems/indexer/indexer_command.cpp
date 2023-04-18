#include "burst_feeder_command.hpp"
#ifndef ENGINEER

namespace src::Feeder {
BurstFeederCommand::BurstFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float speed, float acceptableHeatThreshold, int burstLength)
    : drivers(drivers),
      feeder(feeder),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      startingTotalBallCount(0),
      burstLength(burstLength) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void BurstFeederCommand::initialize() {
    startingTotalBallCount = feeder->getTotalLimitCount();
}

void BurstFeederCommand::execute() {
    feeder->setTargetRPM(speed);
}

void BurstFeederCommand::end(bool) {
    feeder->setTargetRPM(0);
}

bool BurstFeederCommand::isReady() {
    return feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

bool BurstFeederCommand::isFinished() const {
    int elapsedTotal = feeder->getTotalLimitCount() - startingTotalBallCount;
    return (elapsedTotal >= burstLength) || !feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

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

    auto launcherID = turretData.launchMechanismID;
    switch (launcherID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1: {
            lastHeat = turretData.heat17ID1;
            heatLimit = turretData.heatLimit17ID1;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_17MM_2: {
            lastHeat = turretData.heat17ID2;
            heatLimit = turretData.heatLimit17ID2;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_42MM: {
            lastHeat = turretData.heat42;
            heatLimit = turretData.heatLimit42;
            break;
        }
        default:
            break;
    }

    return (lastHeat <= (static_cast<float>(heatLimit) * maxPercentage));
}

FullAutoFeederCommand::FullAutoFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float speed, float acceptableHeatThreshold)
    : drivers(drivers),
      feeder(feeder),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      unjamSpeed(-3000.0f)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FullAutoFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

void FullAutoFeederCommand::execute() {
    if (fabs(feeder->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
        feeder->setTargetRPM(unjamSpeed);
        unjamTimer.restart(175);
    }

    if (unjamTimer.execute()) {
        feeder->setTargetRPM(speed);
        startupThreshold.restart(500);
    }
}

void FullAutoFeederCommand::end(bool) { feeder->setTargetRPM(0.0f); }

bool FullAutoFeederCommand::isReady() { return feeder->isBarrelHeatAcceptable(acceptableHeatThreshold); }

bool FullAutoFeederCommand::isFinished() const { return !feeder->isBarrelHeatAcceptable(acceptableHeatThreshold); }

StopFeederCommand::StopFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder) : drivers(drivers), feeder(feeder) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void StopFeederCommand::initialize() { feeder->setTargetRPM(0); }

void StopFeederCommand::execute() { feeder->setTargetRPM(0); }

void StopFeederCommand::end(bool interrupted) { UNUSED(interrupted); }

bool StopFeederCommand::isReady() { return true; }

bool StopFeederCommand::isFinished() const { return false; }

}  // namespace src::Feeder
#endif
