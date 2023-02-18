#include "subsystems/shooter/barrel_swap/swap_mechanism.hpp"
#ifndef ENGINEER
namespace src::Shooter{

    SwapMechanismSubsystem::SwapMechanismSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
    swapMotor(drivers, SWAP_MOTOR_ID, GIMBAL_BUS, SWAP_DIRECTION),
    barrelMotor(drivers, SHOOTER_1_ID, SHOOTER_BUS, SHOOTER_1_DIRECTION, "Shooter 1 Motor")

    void SwapMechanismSubsystem::initialize() {
        shooterMotor.initialize();
        limitSwitchLeft.initialize();
    }

    void SwapMechanismSubsystem::refresh() {
        updateMotorVelocityPID();
        setDesiredOutput();
        limitSwitchLeft.refresh();
    }

    void SwapMechanismSubsystem::updateMotorVelocityPID() {
        float err = targetRPM - shooterMotor.getShaftRPM();
        shooterVelPID.runControllerDerivateError(err);
        desiredOutput = shooterVelPID.getOutput();
    }

    void SwapMechanismSubsystem::setDesiredOutput() {
        // shooterMotor.setDesiredOutput(static_cast<int32_t>(desiredOutput));
        // below modified from gimbal.cpp:109
        uint16_t currentSwapEncoderPosition = swapMotor.getEncoderWrapped();
        currentSwapMotorRelativeAngle.setValue(wrappedEncoderValueToRadians(currentSwapEncoderPosition));

        swapRelativeDisplay = modm::toDegree(currentSwapMotorRelativeAngle.getValue());
        swapOutputDisplay = desiredSwapMotorOutput;

        swapMotor.setDesiredOutput(desiredSwapMotorOutput);
    }

    void SwapMechanismSubsystem::setSwapMotorOutput(float output) {
        // m2006 motor
        // this function modified from gimbal.cpp:129
        desiredSwapMotorOutput = tap::algorithms::limitVal(output, -M2006_MAX_OUTPUT, M2006_MAX_OUTPUT);
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