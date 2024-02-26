#include "subsystems/dartsystem/stop_dart_command.hpp"

#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef DART_COMPATIBLE

namespace src::Dart {

StopDartCommand::StopDartCommand(
    src::Drivers* drivers,
    DartSubsystem* dart)
    : drivers(drivers),
      dart(dart) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(dart));
}

void StopDartCommand::initialize() {
    // No initialization needed
}

bool isStopRunningDisplay = false;

void StopDartCommand::execute() {

    uint16_t flywheelRPM = 0;

    isStopRunningDisplay = true;

    // dart->ForAllDartMotors(&DartSubsystem::setDesiredOutput, static_cast<float>(flywheelRPM));
    dart->ForAllDartMotors(&DartSubsystem::setTargetRPM, static_cast<float>(flywheelRPM));
    dart->ForAllDartMotors(&DartSubsystem::updateMotorVelocityPID);

    dart->setLoadTargetSpeed(0.0f);
}

void StopDartCommand::end(bool) {
    // No cleanup needed
    isStopRunningDisplay = false;
}

bool StopDartCommand::isReady() { return true; }

bool StopDartCommand::isFinished() const { return false; }
}  // namespace src::Shooter

#endif //#ifdef SHOOTER_COMPATIBLE