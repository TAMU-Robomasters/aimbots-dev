#pragma once

#include "drivers.hpp"
#include "subsystems/chassis/control/chassis.hpp"
#include "tap/control/command.hpp"
#include "utils/tools/common_types.hpp"


#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

class ChassisManualDriveCommand : public TapCommand {
   public:
    ChassisManualDriveCommand(src::Drivers*, ChassisSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Chassis Manual Drive"; }

   private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
};

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE