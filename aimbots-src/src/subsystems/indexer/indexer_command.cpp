#include "indexer_command.hpp"
#ifndef ENGINEER

namespace src::Indexer {
BurstIndexerCommand::BurstIndexerCommand(src::Drivers* drivers, IndexerSubsystem* indexer, float speed, float acceptableHeatThreshold, int burstLength)
    : drivers(drivers),
      indexer(indexer),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      startingTotalBallCount(0),
      burstLength(burstLength) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(indexer));
}

void BurstIndexerCommand::initialize() {
    startingTotalBallCount = indexer->getTotalLimitCount();
}

void BurstIndexerCommand::execute() {
    indexer->setTargetRPM(speed);
}

void BurstIndexerCommand::end(bool) {
    indexer->setTargetRPM(0);
}

bool BurstIndexerCommand::isReady() {
    return indexer->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

bool BurstIndexerCommand::isFinished() const {
    int elapsedTotal = indexer->getTotalLimitCount() - startingTotalBallCount;
    return (elapsedTotal >= burstLength) || !indexer->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
      targetRPM(0),
      desiredOutput(0),
      indexerVelPID(INDEXER_VELOCITY_PID_CONFIG),
      indexerMotor(drivers, INDEXER_ID, INDEX_BUS, INDEXER_DIRECTION, "Indexer Motor"),
      limitSwitchLeft(static_cast<std::string>("C6"), src::Informants::EdgeType::RISING)
#ifdef TARGET_SENTRY
      ,
      limitSwitchRight(static_cast<std::string>("C7"), src::Informants::EdgeType::RISING)
#endif
{
}

void IndexerSubsystem::initialize() {
    indexerMotor.initialize();
    limitSwitchLeft.initialize();
#ifdef TARGET_SENTRY
    limitSwitchRight.initialize();
#endif
}

// refreshes the velocity PID given the target RPM and the current RPM
void IndexerSubsystem::refresh() {
    updateMotorVelocityPID();
    setDesiredOutput();
    limitSwitchLeft.refresh();
#ifdef TARGET_SENTRY
    limitSwitchRight.refresh();
#endif
}

void IndexerSubsystem::updateMotorVelocityPID() {
    float err = targetRPM - indexerMotor.getShaftRPM();
    indexerVelPID.runControllerDerivateError(err);
    desiredOutput = indexerVelPID.getOutput();
}

float IndexerSubsystem::setTargetRPM(float rpm) {
    this->targetRPM = rpm;
    return targetRPM;
}

void IndexerSubsystem::setDesiredOutput() {  // takes the input from the velocity PID and sets the motor to that RPM
    indexerMotor.setDesiredOutput(static_cast<int32_t>(desiredOutput));
}

int IndexerSubsystem::getTotalLimitCount() const {
#ifndef TARGET_SENTRY
    return limitSwitchLeft.getCurrentCount();
#endif
#ifdef TARGET_SENTRY
    return limitSwitchLeft.getCurrentCount() + limitSwitchRight.getCurrentCount();
#endif
}

bool IndexerSubsystem::isBarrelHeatAcceptable(float maxPercentage) {
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

FullAutoIndexerCommand::FullAutoIndexerCommand(src::Drivers* drivers, IndexerSubsystem* indexer, float speed, float acceptableHeatThreshold)
    : drivers(drivers),
      indexer(indexer),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      unjamSpeed(-3000.0f)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(indexer));
}

void FullAutoIndexerCommand::initialize() {
    indexer->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

void FullAutoIndexerCommand::execute() {
    if (fabs(indexer->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
        indexer->setTargetRPM(unjamSpeed);
        unjamTimer.restart(175);
    }

    if (unjamTimer.execute()) {
        indexer->setTargetRPM(speed);
        startupThreshold.restart(500);
    }
}

void FullAutoIndexerCommand::end(bool) { indexer->setTargetRPM(0.0f); }

bool FullAutoIndexerCommand::isReady() { return indexer->isBarrelHeatAcceptable(acceptableHeatThreshold); }

bool FullAutoIndexerCommand::isFinished() const { return !indexer->isBarrelHeatAcceptable(acceptableHeatThreshold); }

StopIndexerCommand::StopIndexerCommand(src::Drivers* drivers, IndexerSubsystem* indexer) : drivers(drivers), indexer(indexer) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(indexer));
}

void StopIndexerCommand::initialize() { indexer->setTargetRPM(0); }

void StopIndexerCommand::execute() { indexer->setTargetRPM(0); }

void StopIndexerCommand::end(bool interrupted) { UNUSED(interrupted); }

bool StopIndexerCommand::isReady() { return true; }

bool StopIndexerCommand::isFinished() const { return false; }

}  // namespace src::Indexer
#endif
