#include "utils/tools/robot_specific_defines.hpp"

#pragma once

#ifdef ALL_SENTRIES

#include "subsystems/chassis/sentry_commands/sentry_match_chassis_control_command.hpp"
#include "subsystems/gimbal/complex_commands/gimbal_patrol_command.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

namespace src::Gimbal {

class SentryMatchGimbalControlCommand : public TapComprisedCommand {
public:
    SentryMatchGimbalControlCommand(
        src::Drivers*,
        GimbalSubsystem*,
        GimbalFieldRelativeController*,
        src::Utils::RefereeHelperTurreted*,
        src::Utils::Ballistics::BallisticsSolver*,
        GimbalPatrolConfig patrolConfig,
        src::Chassis::ChassisMatchStates& chassisState,
        int chaseTimeoutMillis);

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Gimbal Control Command"; }

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;

    GimbalFieldRelativeController* controller;

    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver;

    GimbalPatrolCommand patrolCommand;
    GimbalChaseCommand chaseCommand;

    src::Chassis::ChassisMatchStates& chassisState;

    MilliTimeout chaseTimeout;
    int chaseTimeoutMillis = 0;
};

}  // namespace src::Gimbal

#endif