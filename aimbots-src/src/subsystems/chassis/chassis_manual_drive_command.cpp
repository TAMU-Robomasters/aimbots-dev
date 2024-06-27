#include "chassis_manual_drive_command.hpp"

#include "subsystems/chassis/chassis_helper.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

ChassisManualDriveCommand::ChassisManualDriveCommand(src::Drivers* drivers, ChassisSubsystem* chassis)
    : drivers(drivers),
      chassis(chassis)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisManualDriveCommand::initialize() {}

void ChassisManualDriveCommand::execute() {
    float xDesiredWheelspeed = 0.0f;
    float yDesiredWheelspeed = 0.0f;
    float rotationDesiredWheelspeed = 0.0f;

    // gets desired user input from operator interface
    Helper::getUserDesiredInput(drivers, chassis, &xDesiredWheelspeed, &yDesiredWheelspeed, &rotationDesiredWheelspeed);

    // rescales desired input to power limited speed, also limits translational movement based on rotational movement
    Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &xDesiredWheelspeed, &yDesiredWheelspeed, &rotationDesiredWheelspeed);

    chassis->setTargetRPMs(xDesiredWheelspeed, yDesiredWheelspeed, rotationDesiredWheelspeed);
}

void ChassisManualDriveCommand::end(bool) { chassis->setZeroVelocity(); }

bool ChassisManualDriveCommand::isReady() { return true; }

bool ChassisManualDriveCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE