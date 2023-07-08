#pragma once

#include "utils/motion/auto_nav/auto_navigator_holonomic.hpp"

#include "src/subsystems/chassis/chassis.hpp"
#include "src/subsystems/chassis/chassis_helper.hpp"
#include "drivers.hpp"

namespace src::Chassis {

class SentryChassisAutoNavEvadeCommand : public TapCommand {
public:
    SentryChassisAutoNavEvadeCommand(src::Drivers* drivers, ChassisSubsystem* chassis);

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Chassis Evade Command"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
};

}  // namespace src::Chassis