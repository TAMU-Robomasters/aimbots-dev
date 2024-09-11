#pragma once

#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/chassis/control/chassis_helper.hpp"
#include "utils/motion/auto_nav/auto_navigator_holonomic.hpp"

#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

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

#endif //#ifdef CHASSIS_COMPATIBLE