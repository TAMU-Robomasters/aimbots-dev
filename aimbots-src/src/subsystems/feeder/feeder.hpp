#pragma once

#include "informants/limit_switch.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Feeder {

class FeederSubsystem : public tap::control::Subsystem {
   public:
    FeederSubsystem(
        src::Drivers* drivers);

    mockable void initialize() override;
    mockable void refresh() override;

    void updateMotorVelocityPID();

    mockable void setDesiredOutput();

    mockable float setTargetRPM(float rpm);

    int getTotalLimitCount() const;

    bool isBarrelHeatAcceptable(float maxPercentage) {
        using RefSerialRxData = tap::communication::serial::RefSerial::Rx;
        auto turretData = drivers->refSerial.getRobotData().turret;

        uint16_t lastHeat = 0;
        uint16_t heatLimit = 0;

        if (turretData.bulletType == RefSerialRxData::BulletType::AMMO_17) {
            auto mechID = turretData.launchMechanismID;
            if (mechID == RefSerialRxData::MechanismID::TURRET_17MM_1) {
                lastHeat = turretData.heat17ID1;
                heatLimit = turretData.heatLimit17ID1;
            } else if (mechID == RefSerialRxData::MechanismID::TURRET_17MM_2) {
                lastHeat = turretData.heat17ID2;
                heatLimit = turretData.heatLimit17ID2;
            }
        } else if (turretData.bulletType == RefSerialRxData::BulletType::AMMO_42) {
            lastHeat = turretData.heat42;
            heatLimit = turretData.heatLimit42;
        }

        return lastHeat <= (static_cast<float>(heatLimit) * maxPercentage);
    }

    SmoothPID feederVelPID;

#ifndef ENV_UNIT_TESTS
   private:
#else
   public:
#endif
    float targetRPM;
    float desiredOutput;

    DJIMotor feederMotor;

    LimitSwitch limitSwitchLeft;  // for single-barreled robots
#ifdef TARGET_SENTRY
    LimitSwitch limitSwitchRight;  // for double-barreled robots
#endif

    // commands
};

}  // namespace src::Feeder