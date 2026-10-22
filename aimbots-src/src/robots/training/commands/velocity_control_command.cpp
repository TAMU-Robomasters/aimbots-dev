#include "velocity_control_command.hpp"

#ifdef TARGET_TRAINING

// MCUViewer watch variables.
float stickTargetRpmDisplay = 0.0f;   // the speed your stick mapping asks for
float stickCurrentRpmDisplay = 0.0f;  // what the motor actually does

namespace src::Training {

VelocityControlCommand::VelocityControlCommand(
    src::Drivers* drivers,
    TrainingMotorSubsystem* motor,
    VelocityMotorPid* controller)
    : drivers(drivers),
      motor(motor),
      controller(controller)  //
{
    addSubsystemRequirement(motor);
}

void VelocityControlCommand::initialize() {
    // TODO(week3): get the controller ready (it may still remember the last command that used it).
}

void VelocityControlCommand::execute() {
    // TODO(week3):
    // 1. Read the right stick's vertical axis from YOUR remote: drivers->dt7Remote.getRightVertical()
    //    (-1 = full down, +1 = full up).
    // 2. Map it to a speed between -MAX_SPEED_RPM and +MAX_SPEED_RPM.
    // 3. Give that target to the controller and run it.
    // 4. Update stickTargetRpmDisplay / stickCurrentRpmDisplay for MCUViewer.
}

void VelocityControlCommand::end(bool) {
    // TODO(week3): stop the motor. (What happens to a spinning motor if you leave this empty?)
}

bool VelocityControlCommand::isFinished() const { return false; }

}  // namespace src::Training

#endif  // TARGET_TRAINING
