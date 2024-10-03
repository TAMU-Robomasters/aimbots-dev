#include "subsystems/feeder/basic_commands/stop_feeder_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "drivers.hpp"


#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "utils/tools/common_types.hpp"



namespace src::Feeder{

    StopFeederCommand::StopFeederCommand(
        src::Drivers* drivers,
        FeederSubsystem* feeder)
    :   drivers(drivers),
        feeder(feeder)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    }

void StopFeederCommand::initialize() override;

    void StopFeederCommand::execute() override;
    void StopFeederCommand::end(bool interrupted) override;
    bool StopFeederCommand::isReady() { return true; };

    bool StopFeederCommand::isFinished() const override;

    const char* StopFeederCommand::getName() const override { return "stop feeder"; }

}

