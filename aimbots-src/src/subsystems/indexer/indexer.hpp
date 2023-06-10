#pragma once
#ifndef ENGINEER
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "informants/limit_switch.hpp"
#include "robots/hero/hero_constants.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Indexer {

// this contains the subsystem AND commands
class IndexerSubsystem : public tap::control::Subsystem {
public:
    IndexerSubsystem(src::Drivers* drivers);

    mockable void initialize() override;
    mockable void refresh() override;

    void updateMotorVelocityPID();

    mockable void setDesiredOutput();

    mockable float setTargetRPM(float rpm);

    float getTargetRPM() const { return targetRPM; }

    float getCurrentRPM() const { return indexerMotor.getShaftRPM(); }

    int getTotalLimitCount() const;

    bool isBarrelHeatAcceptable(float maxPercentage);

#ifndef ENV_UNIT_TESTS
private:
#else
public:
#endif

    float targetRPM;
    float desiredOutput;

    SmoothPID indexerVelPID;
    DJIMotor indexerMotor;

    src::Informants::LimitSwitch limitSwitchLeft;  // for single-barreled robots
#ifdef TARGET_SENTRY
    src::Informants::LimitSwitch limitSwitchRight;  // for double-barreled robots
#endif

    // commands
};

}  // namespace src::Indexer

#endif
