
#pragma once

#ifdef TARGET_SENTRY

#include "utils/tools/robot_specific_defines.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "subsystems/shooter/shooter_constants.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"



#ifdef CHASSIS_COMPATIBLE


namespace SentryControl{

class SentryIntelligenceCommand : public TapComprisedCommand {
public:
    SentryIntelligenceCommand(
        src::Drivers*);

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Intelligence Command"; }

private:
    src::Drivers* drivers;
    


};

}


#endif
#endif