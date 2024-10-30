#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/shooter/control/shooter.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

#ifdef SHOOTER_COMPATIBLE

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

    bool wasNeutral = false;
    int speedIncrement = 0;

    src::Utils::RefereeHelperTurreted* refHelper;
};

}  // namespace src::Shooter

#endif  //#ifdef SHOOTER_COMPATIBLE