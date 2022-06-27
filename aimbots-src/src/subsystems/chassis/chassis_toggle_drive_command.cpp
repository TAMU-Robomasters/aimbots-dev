#include "chassis_toggle_drive_command.hpp"

namespace src::Chassis {

ChassisToggleDriveCommand::ChassisToggleDriveCommand(src::Drivers* drivers, ChassisSubsystem* chassis, Gimbal::GimbalSubsystem* gimbal)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      chassis(chassis),
      followGimbalCommand(drivers, chassis, gimbal),
      tokyoCommand(drivers, chassis, gimbal) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
    comprisedCommandScheduler.registerSubsystem(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisToggleDriveCommand::initialize() {
    if (!comprisedCommandScheduler.isCommandScheduled(&tokyoCommand))
        comprisedCommandScheduler.removeCommand(&tokyoCommand, true);
    if (!comprisedCommandScheduler.isCommandScheduled(&followGimbalCommand))
        comprisedCommandScheduler.addCommand(&followGimbalCommand);
}

void ChassisToggleDriveCommand::execute() {
    if (drivers->remote.keyPressed(Remote::Key::F))
        wasFPressed = true;

    if (wasFPressed && !drivers->remote.keyPressed(Remote::Key::F)) {
        wasFPressed = false;

        if (comprisedCommandScheduler.isCommandScheduled(&followGimbalCommand)) {
            comprisedCommandScheduler.addCommand(&tokyoCommand);
        } else if (comprisedCommandScheduler.isCommandScheduled(&tokyoCommand)) {
            comprisedCommandScheduler.addCommand(&followGimbalCommand);
        }
    }
    comprisedCommandScheduler.run();
}

void ChassisToggleDriveCommand::end(bool interrupted) {
    UNUSED(interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &followGimbalCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoCommand, interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisToggleDriveCommand::isReady() { return true; }

bool ChassisToggleDriveCommand::isFinished() const { return false; }

}  // namespace src::Chassis