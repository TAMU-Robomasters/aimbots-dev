#pragma once
#ifdef TARGET_SENTRY

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/sentry_commands/chassis_rail_bounce_command.hpp"
#include "subsystems/chassis/sentry_commands/chassis_rail_evade_command.hpp"
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

#endif