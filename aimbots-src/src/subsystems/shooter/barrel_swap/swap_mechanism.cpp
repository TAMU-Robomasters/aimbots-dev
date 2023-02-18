#include "subsystems/shooter/barrel_swap/swap_mechanism.hpp"
#ifndef ENGINEER
namespace src::Shooter{

    SwapMechanismSubsystem::SwapMechanismSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
    targetRPM(0),
    desiredOutput(0),
    shooterVelPID(SHOOTER_VELOCITY_PID_CONFIG),
    shooterMotor(drivers, SHOOTER_1_ID, SHOOTER_BUS, SHOOTER_1_DIRECTION, "Shooter 1 Motor"),
    shooterMotor(drivers, SHOOTER_2_ID, SHOOTER_BUS, SHOOTER_2_DIRECTION, "Shooter 2 Motor"),
    limitSwitchLeft(static_cast<std::string>("C6"), src::Informants::EdgeType::RISING)
    limitSwitchRight(static_cast<std::string>("C7"), src::Informants::EdgeType::RISING)

    void SwapMechanismSubsystem::initialize() {
        shooterMotor.initialize();
        limitSwitchLeft.initialize();
        limitSwitchRight.initialize();
    }

    void SwapMechanismSubsystem::refresh() {
        updateMotorVelocityPID();
        setDesiredOutput();
        limitSwitchLeft.refresh();
        limitSwitchRight.refresh();
    }

    void SwapMechanismSubsystem::updateMotorVelocityPID() {
        float err = targetRPM - shooterMotor.getShaftRPM();
        shooterVelPID.runControllerDerivateError(err);
        desiredOutput = shooterVelPID.getOutput();
    }

    float SwapMechanismSubsystem::setTargetRPM(float rpm) {
        this->targetRPM = rpm;
        return targetRPM;
    }

    void SwapMechanismSubsystem::setDesiredOutput() {
        shooterMotor.setDesiredOutput(static_cast<int32_t>(desiredOutput));
    }

    int SwapMechanismSubsystem::getTotalLimitCount() const {
        return limitSwitchLeft.getCurrentCount() + limitSwitchRight.getCurrentCount();
    }

    bool SwapMechanismSubsystem::isBarrelHeatAcceptable(float maxPercentage) {
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
}
#endif