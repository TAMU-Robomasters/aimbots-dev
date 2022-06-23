#pragma once

#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "drivers.hpp"
#include "informants/limit_switch.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Feeder {

class FeederSubsystem : public tap::control::Subsystem {
   public:
    FeederSubsystem(src::Drivers* drivers);

    mockable void initialize() override;
    mockable void refresh() override;

    void updateMotorVelocityPID();

    mockable void setDesiredOutput();

    mockable float setTargetRPM(float rpm);

    int getTotalLimitCount() const;

#ifndef ENV_UNIT_TESTS
   private:
#else
   public:
#endif

    float targetRPM;
    float desiredOutput;

    SmoothPID feederVelPID;
    DJIMotor feederMotor;

    src::Informants::LimitSwitch limitSwitchLeft;  // for single-barreled robots
#ifdef TARGET_SENTRY
    src::Informants::LimitSwitch limitSwitchRight;  // for double-barreled robots
#endif

    // commands
};

}  // namespace src::Feeder