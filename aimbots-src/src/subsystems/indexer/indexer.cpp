#include "indexer.hpp"

namespace src::Indexer {

IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, MotorID INDEXER_ID, CANBus INDEX_BUS, bool INDEXER_DIRECTION, SmoothPidConfig INDEXER_VELOCITY_PID_CONFIG)
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

}  // namespace src::Indexer
