#include "chassis_auto_nav_velocity_command.hpp"

#if defined(CHASSIS_COMPATIBLE) && defined(GIMBAL_COMPATIBLE)

namespace src::Chassis {

ChassisAutoNavVelocityCommand::ChassisAutoNavVelocityCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    src::Gimbal::GimbalSubsystem* gimbal)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisAutoNavVelocityCommand::initialize() {}

// Tunable: scales the Jetson velocity command (m/s).
// Tune in Ozone until the robot's real speed matches the commanded m/s. Good enough for now
// ideally you would tune the actually motor pid's. 
float velocityToInputMultiplier = 0.60f;

// Debug / watch variables
float velCmdTurretXDisplay = 0.0f;
float velCmdTurretYDisplay = 0.0f;
float desiredChassisXInputDisplay = 0.0f;
float desiredChassisYInputDisplay = 0.0f;
bool jetsonOnlineForNavDisplay = false;

void ChassisAutoNavVelocityCommand::execute() {
    chassis->setTokyoDrift(false);

    jetsonOnlineForNavDisplay = drivers->cvCommunicator.isJetsonOnline();
    if (!jetsonOnlineForNavDisplay) {
        chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
        return;
    }

    // Turret-relative chassis velocity command (m/s) from nav2 on the Jetson.
    modm::Vector2f turretRelativeVelocity = drivers->cvCommunicator.getDesiredTurretRelativeVelocity();

    velCmdTurretXDisplay = turretRelativeVelocity.getX();
    velCmdTurretYDisplay = turretRelativeVelocity.getY();

    float desiredX = turretRelativeVelocity.getX() * velocityToInputMultiplier;
    float desiredY = turretRelativeVelocity.getY() * velocityToInputMultiplier;
    float desiredRotation = 0.0f;

    Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

    if (gimbal->isOnline()) {
        float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        tap::algorithms::rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);
    }

    desiredChassisXInputDisplay = desiredX;
    desiredChassisYInputDisplay = desiredY;

    chassis->setTargetRPMs(desiredX, desiredY, 0.0f);
}

void ChassisAutoNavVelocityCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisAutoNavVelocityCommand::isReady() { return true; }

bool ChassisAutoNavVelocityCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif  // defined(CHASSIS_COMPATIBLE) && defined(GIMBAL_COMPATIBLE)
