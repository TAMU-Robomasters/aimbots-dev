#pragma once
#ifdef ULTRASONIC

#include "subsystems/chassis/chassis.hpp"

#include "utils/common_types.hpp"

#include "drivers.hpp"

namespace src::Chassis {

enum ChassisMatchStates {
    NONE = 0,
    PATROL = 1,
    EVADE = 2,
};

class SentryMatchChassisControlCommand : public TapComprisedCommand {
public:
    SentryMatchChassisControlCommand(src::Drivers*, ChassisSubsystem*, ChassisMatchStates& chassisState);

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Chassis Control Command"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    src::Chassis::ChassisMatchStates& chassisState;

    ChassisRailBounceCommand patrolCommand;
    ChassisRailEvadeCommand evadeCommand;

    MilliTimeout evadeTimeout;
};

}  // namespace src::Chassis

#endif // ULTRASONIC