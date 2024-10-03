#include "utils/tools/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "utils/tools/common_types.hpp"


namespace src::Feeder{
    
    FullAutoFeederCommand::FullAutoFeederCommand(
        src::Drivers* drivers,
        FeederSubsystem* feeder)
    :   drivers(drivers),
        feeder(feeder)
{
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FullAutoFeederCommand::initialize() override;

    void FullAutoFeederCommand::execute() override;
    void FullAutoFeederCommand::end(bool interrupted) override;
    bool FullAutoFeederCommand::isReady() override;

    bool FullAutoFeederCommand::isFinished() const override;

    const char* FullAutoFeederCommand::getName() const override { return "full auto feeder"; }



}
