
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
}

void BarrelSwapSubsytem::initialize() {
    // move until hits limit switch
    // LimitSwitch.isFalling or something maybe
    // reset encoders (how?)
    // from field_relative_informant.cpp
    // motorRevolutionsUnwrapped = static_cast<float>(odomRailMotor->getEncoderUnwrapped()) /
    //                             static_cast<float>(odomRailMotor->ENC_RESOLUTION);

}

void BarrelSwapSubsytem::refresh() {
    
    if (shooter.runShooterCommand.isFinished) {
        // calculate how long the shooter has been not firing, and if it is over a minimum threshold, switch barrels
        clock_t t = clock();
        using RefSerialData = tap::communication::serial::RefSerial::Rx;
        auto turretData = drivers -> refSerial.getRobotData().turret;

        uint16_t heat = 0;
        uint16_t MAX_HEAT = 0;

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

        // if (clock() - t <(DEAD_TIME*CLOCKS_PER_SEC) && current_shooter.heat > MAX_HEAT) {
            // current_shooter.stopShooterCommand.execute();
            // something here to swap barrels until barrel is in proper spot
            // swap current shooter with other shooter
            // current_shooter = other_shooter
        }
    }
    
}  // namespace src::Shooter

#endif