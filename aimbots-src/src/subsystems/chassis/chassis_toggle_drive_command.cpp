#include "chassis_toggle_drive_command.hpp"

namespace src::Chassis {

ChassisToggleDriveCommand::ChassisToggleDriveCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    Gimbal::GimbalSubsystem* gimbal)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      chassis(chassis),
      followGimbalCommand(drivers, chassis, gimbal),
      tokyoCommand(drivers, chassis, gimbal),
      tokyoLeftCommand(drivers, chassis, gimbal, -1),
      tokyoRightCommand(drivers, chassis, gimbal, 1) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
    comprisedCommandScheduler.registerSubsystem(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisToggleDriveCommand::initialize() {
    // TODO: Logic is backwards maybe?
    // if (!comprisedCommandScheduler.isCommandScheduled(&tokyoCommand))
    // comprisedCommandScheduler.removeCommand(&tokyoCommand, true);
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &followGimbalCommand);
    // if (!comprisedCommandScheduler.isCommandScheduled(&followGimbalCommand))
    // comprisedCommandScheduler.addCommand(&followGimbalCommand);
    qPressed.restart(0);
    ePressed.restart(0);
}

bool isQDone = false;
bool isEDone = false;

void ChassisToggleDriveCommand::execute() {
    // This needs to match the button in Gimbal Toggle Aiming!
    if (drivers->remote.keyPressed(Remote::Key::F)) wasFPressed = true;

    if (drivers->remote.keyPressed(Remote::Key::E)) {
        ePressed.restart(800);
    }
    if (drivers->remote.keyPressed(Remote::Key::Q)) {
        qPressed.restart(800);
    }

    isQDone = !qPressed.isExpired();
    isEDone = !ePressed.isExpired();

    if (wasFPressed && !drivers->remote.keyPressed(Remote::Key::F)) {
        wasFPressed = false;
        preferSpecificSpin = !ePressed.isExpired() || !qPressed.isExpired();

        if (comprisedCommandScheduler.isCommandScheduled(&followGimbalCommand)) {
            if (preferSpecificSpin) {
                if (!ePressed.isExpired()) {
                    scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoRightCommand);
                    // qPressed.stop();
                    // ePressed.stop();
                }
                if (!qPressed.isExpired()) {
                    scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoLeftCommand);
                    // qPressed.stop();
                    // ePressed.stop();
                }
            } else {
                scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoCommand);
            }
        } else if (
            comprisedCommandScheduler.isCommandScheduled(&tokyoCommand) ||
            comprisedCommandScheduler.isCommandScheduled(&tokyoLeftCommand) ||
            comprisedCommandScheduler.isCommandScheduled(&tokyoRightCommand)) {
            comprisedCommandScheduler.addCommand(&followGimbalCommand);
        }
    }
    comprisedCommandScheduler.run();
}

void ChassisToggleDriveCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &followGimbalCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoLeftCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoRightCommand, interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisToggleDriveCommand::isReady() { return true; }

bool ChassisToggleDriveCommand::isFinished() const { return false; }

}  // namespace src::Chassis