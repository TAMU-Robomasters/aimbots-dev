
#ifdef TARGET_STANDARD

#include "barrel_swap.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Shooter {
BarrelSwapSubsytem::BarrelSwapSubsytem(tap::Drivers*) : Subsystem(drivers) {}

BarrelSwapSubsytem::BarrelSwapSubsytem(src::Drivers* drivers, ShooterSubsystem* shooter) {
    this->drivers = drivers;
    this->shooter = shooter;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
    limitSwitchLeft(static_cast<std::string>("C6"), src::Informants::EdgeType::RISING)
}

void BarrelSwapSubsytem::initialize() {
    // move until hits limit switch
    // LimitSwitch.isFalling or something maybe
    // reset encoders
    // from field_relative_informant.cpp
    shooter.initialize();
    limitSwitchLeft.inialize();
    // motorRevolutionsUnwrapped = static_cast<float>(odomRailMotor->getEncoderUnwrapped()) /
    //                             static_cast<float>(odomRailMotor->ENC_RESOLUTION);

}

void BarrelSwapSubsytem::refresh() {
    updateMotorVelocityPID();
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
            default;
                break;
        }

        if (heat >= MAX_HEAT) {
            // current_shooter.stopShooterCommand
            // swap barrels
            // m2006 motor
            // swap_mechanism.cpp
            // SwapMechanismSubsystem
            // temp = other_shooter
            // other_shooter = current_shooter
            // current_shooter = temp
        }
        }
    }
    
}  // namespace src::Shooter

#endif