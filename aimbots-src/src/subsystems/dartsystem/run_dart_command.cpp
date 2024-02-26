#include "subsystems/dartsystem/run_dart_command.hpp"

#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef DART_COMPATIBLE

namespace src::Dart {

RunDartCommand::RunDartCommand(
    src::Drivers* drivers,
    DartSubsystem* dart)
    : drivers(drivers),
      dart(dart) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(dart));
}

bool isCommandRunningDisplay = false;

void RunDartCommand::initialize() {
    // No initialization needed
    isCommandRunningDisplay = true;
}

uint16_t flywheelRPMDisplay = 0;
uint16_t flywheelCurrentRPMDisplay = 0;

float currentHeatDisplay = 0.0f;
float heatLimitDisplay = 0.0f;

void RunDartCommand::execute() {

    uint16_t flywheelRPM = 9000;

    dart->ForAllDartMotors(&DartSubsystem::setTargetRPM, static_cast<float>(flywheelRPM));
    dart->ForAllDartMotors(&DartSubsystem::updateMotorVelocityPID);
}

void RunDartCommand::end(bool) {
    // No cleanup needed
    isCommandRunningDisplay = false;
}

bool RunDartCommand::isReady() { return true; }

bool RunDartCommand::isFinished() const { return false; }
}  // namespace src::Shooter

#endif //#ifdef SHOOTER_COMPATIBLE