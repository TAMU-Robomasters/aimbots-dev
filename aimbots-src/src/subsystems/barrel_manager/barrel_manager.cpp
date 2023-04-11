#include "barrel_manager.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace src::Barrel_Manager {

BarrelSwapSubsytem::BarrelSwapSubsytem(tap::Drivers* drivers) : tap::control::Subsystem(drivers), swapMotor(drivers,
                   SWAP_MOTOR_ID,
                   BARREL_BUS,
                   SWAP_DIRECTION,
                   "Swap Motor") {
    
}


void BarrelSwapSubsytem::initialize() {
    swapMotor.initialize();
    swapMotor.setDesiredOutput(0);

}

void BarrelSwapSubsytem::refresh() {

    if (swapMotor.isMotorOnline()) {
        uint64_t swapMotorUnwrapedEncoder = swapMotor.getEncoderUnwrapped();
        currentSwapMotorPosition = swapMotorUnwrapedEncoder / LEAD_SCREW_TICKS_PER_MM;
        swapMotor.setDesiredOutput(desiredSwapMotorOutput);

    }

    //Continue to follow outline of gimbal.cpp refresh




    /*updateMotorVelocityPID();
    limitSwitchLeft.refresh();
    if (shooter.runShooterCommand.isFinished) {
        // calculate how long the shooter has been not firing, and if it is over a minimum threshold, switch barrels
        clock_t t = clock();
        using RefSerialData = tap::communication::serial::RefSerial::Rx;
        auto turretData = drivers -> refSerial.getRobotData().turret;

        uint16_t heat = 0;
        uint16_t MAX_HEAT = 0;

        // replace later once mechanism ID is known
        auto launcherID = turretData.launchMechanismID;
        switch (launcherID) {
            case RefSerialRxData::MechanismID::TURRET_17MM_1: {
                heat = turretData.heat17ID1;
                MAX_HEAT = turretData.heatLimit17ID1;
                break;
            }
            case RefSerialRxData::MechanismID::TURRET_17MM_2: {
                heat = turretData.heat17ID2;
                MAX_HEAT = turretData.heatLimit17ID2;
                break;
            }
            case RefSerialRxData::MechanismID::TURRET_42MM: {
                heat = turretData.heatLimit42;
                MAX_HEAT = turretData.heatLimit42;
                break;
            }
            default:
                break;
        }
        }*/
    }
    
}  // namespace src::Shooter

#endif