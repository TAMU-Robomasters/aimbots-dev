#pragma once

#include "drivers.hpp"
#include "subsystems/chassis/chassis.hpp"
#include "tap/control/command.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Chassis {

    enum chassisControlMode {
        MANUAL,
        PATROL,
        EVADE_SLOW,
        EVADE_FAST,
    };

    class ChassisDriveCommand : public TapCommand {
       public:
        ChassisDriveCommand(src::Drivers*, ChassisSubsystem*);
        void initialize() override;

        void execute() override;
        void end(bool interrupted) override;
        bool isReady() override;

        bool isFinished() const override;

        const char* getName() const override { return "chassis drive"; }

       private:
        src::Drivers* drivers;
        ChassisSubsystem* chassis;

        chassisControlMode currMode;
    };

}  // namespace src::Chassis