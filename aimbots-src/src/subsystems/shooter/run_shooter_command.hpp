#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/shooter/shooter.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Shooter {

class RunShooterCommand : public TapCommand {
public:
    RunShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter, src::Utils::RefereeHelperTurreted* refHelper);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "shooter subsystem"; }

private:
    src::Drivers* drivers;
    ShooterSubsystem* shooter;

    src::Utils::RefereeHelperTurreted* refHelper;
};

}  // namespace src::Shooter