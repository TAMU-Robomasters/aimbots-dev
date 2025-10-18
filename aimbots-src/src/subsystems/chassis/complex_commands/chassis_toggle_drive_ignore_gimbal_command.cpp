#include "chassis_toggle_drive_ignore_gimbal_command.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

ChassisToggleDriveIgnoreGimbalCommand::ChassisToggleDriveIgnoreGimbalCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    Gimbal::GimbalSubsystem* gimbal,
    const TokyoConfig& tokyoConfig,
    bool randomizeSpinRate,
    const SpinRandomizerConfig& randomizerConfig)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      chassis(chassis),
      ignoreGimbalCommand(drivers, chassis, gimbal),
      tokyoCommand(drivers, chassis, gimbal, tokyoConfig, 0, randomizeSpinRate, randomizerConfig),
      tokyoLeftCommand(drivers, chassis, gimbal, tokyoConfig, -1, randomizeSpinRate, randomizerConfig),
      tokyoRightCommand(drivers, chassis, gimbal, tokyoConfig, 1, randomizeSpinRate, randomizerConfig)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
    comprisedCommandScheduler.registerSubsystem(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisToggleDriveIgnoreGimbalCommand::initialize() {
    // TODO: Logic is backwards maybe?
    // if (!comprisedCommandScheduler.isCommandScheduled(&tokyoCommand))
    // comprisedCommandScheduler.removeCommand(&tokyoCommand, true);
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &ignoreGimbalCommand);
    // if (!comprisedCommandScheduler.isCommandScheduled(&followGimbalCommand))
    // comprisedCommandScheduler.addCommand(&followGimbalCommand);
    qPressed.restart(0);
    ePressed.restart(0);
}


void ChassisToggleDriveIgnoreGimbalCommand::execute() {
    // This needs to match the button in Gimbal Toggle Aiming!
    if (drivers->remote.keyPressed(Remote::Key::F)) wasFPressed = true;

    if (drivers->remote.keyPressed(Remote::Key::E)) {
        ePressed.restart(800);
    }
    if (drivers->remote.keyPressed(Remote::Key::Q)) {
        qPressed.restart(800);
    }

    if (wasFPressed && !drivers->remote.keyPressed(Remote::Key::F)) {
        wasFPressed = false;
        preferSpecificSpin = !ePressed.isExpired() || !qPressed.isExpired();

        if (comprisedCommandScheduler.isCommandScheduled(&ignoreGimbalCommand)) {
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
            comprisedCommandScheduler.addCommand(&ignoreGimbalCommand);
        }
    }
    comprisedCommandScheduler.run();
}

void ChassisToggleDriveIgnoreGimbalCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &ignoreGimbalCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoLeftCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoRightCommand, interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisToggleDriveIgnoreGimbalCommand::isReady() { return true; }

bool ChassisToggleDriveIgnoreGimbalCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE