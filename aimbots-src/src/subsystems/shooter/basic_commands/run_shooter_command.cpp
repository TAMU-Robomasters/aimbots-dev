#include "subsystems/shooter/basic_commands/run_shooter_command.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "subsystems/shooter/control/shooter.hpp"
#include "drivers.hpp"



namespace src::Shooter{

RunShooterCommand::RunShooterCommand(
    src::Drivers* drivers,
    ShooterSubsystem* shooter)
    : drivers(drivers)
      shooter(shooter)

{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void RunShooterCommand::initialize() {};

    void RunShooterCommand::execute() override;
    void RunShooterCommand::end(bool interrupted) override;
    bool RunShooterCommand::isReady() override;

    bool RunShooterCommand::isFinished() const override;

    const char* RunShooterCommand::getName() const override { return "run shooter"; }


}