#pragma once

#include "utils/ballistics/ballistics_solver.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

//
#include "subsystems/chassis/sentry_commands/sentry_match_chassis_control_command.hpp"
#include "subsystems/feeder/basic_commands/full_auto_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/stop_feeder_command.hpp"
#include "subsystems/feeder/control/feeder.hpp"
//
#include "subsystems/shooter/basic_commands/run_shooter_command.hpp"
#include "subsystems/shooter/control/shooter.hpp"
#include "subsystems/shooter/basic_commands/stop_shooter_command.hpp"
//
#include <subsystems/gimbal/control/gimbal_controller_interface.hpp>

namespace src::Control {

enum FeederMatchStates {
    ANNOYED = 0,
    AGGRESSIVE = 1,
    FURIOUS = 2,
};

using namespace src::Feeder;
using namespace src::Shooter;

static constexpr float SENTRY_SHOOTER_MAX_ACCEL = 2.5f;

class SentryMatchFiringControlCommand : public TapComprisedCommand {
public:
    SentryMatchFiringControlCommand(
        src::Drivers*,
        FeederSubsystem*,
        ShooterSubsystem*,
        src::Utils::RefereeHelperTurreted*,
        src::Utils::Ballistics::BallisticsSolver*,
        src::Gimbal::GimbalControllerInterface*,
        src::Chassis::ChassisMatchStates& chassisState);

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Firing Control Command"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    ShooterSubsystem* shooter;
    src::Utils::RefereeHelperTurreted* refHelper;

    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver;
    src::Gimbal::GimbalControllerInterface* fieldRelativeGimbalController;

    src::Chassis::ChassisMatchStates& chassisState;

    StopFeederCommand stopFeederCommand;
    FullAutoFeederCommand fullAutoFeederCommand;

    StopShooterCommand stopShooterCommand;
    RunShooterCommand runShooterCommand;

    MilliTimeout refireTimer;
    MilliTimeout shootMinimumTime;
};

}  // namespace src::Control