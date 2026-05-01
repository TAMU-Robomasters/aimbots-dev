#include "chassis_dui_command.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

ChassisDUICommand::ChassisDUICommand(
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

void ChassisDUICommand::initialize() {
    // TODO: Logic is backwards maybe?
    // if (!comprisedCommandScheduler.isCommandScheduled(&tokyoCommand))
    // comprisedCommandScheduler.removeCommand(&tokyoCommand, true);
  //  scheduleIfNotScheduled(this->comprisedCommandScheduler, &ignoreGimbalCommand);
    chassis->setTargetRPMs(10.0f, 0.0f, 0.0f);
    // if (!comprisedCommandScheduler.isCommandScheduled(&followGimbalCommand))
    // comprisedCommandScheduler.addCommand(&followGimbalCommand);
    drunkMotion.restart(3000);
    ePressed.restart(0);
}

float displayY = 0.0f;
void ChassisDUICommand::execute() {
    // This needs to match the button in Gimbal Toggle Aiming!
    // if (drivers->remote.keyPressed(Remote::Key::F)) wasFPressed = true;

    // if (drivers->remote.keyPressed(Remote::Key::E)) {
    //     ePressed.restart(800);
    // }
    // if (drivers->remote.keyPressed(Remote::Key::Q)) {
    //     qPressed.restart(800);
    // }

    if(drunkMotion.isExpired()){
        scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoRightCommand);
        chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
    }else{
        chassis->setTokyoDrift(false);
        float desiredX = 0.0f;
        float desiredY = 0.5f;
        float desiredRotation = 0.0f;
        // gets desired user input from operator interface
        //Chassis::Helper::getUserDesiredInput(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
        displayY = desiredY;
        //gimbalOnlineDisplay = gimbal->isOnline();getUserDesiredInput

        // if (gimbal->isOnline()) {
        //     float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);

        //     float chassisErrorAngle = yawAngleFromChassisCenter;

        //     //chassisErrorAngleDisplay = chassisErrorAngle;

        //     // Find rotation correction power
        //     rotationController.runController(
        //         chassisErrorAngle,
        //         -RADPS_TO_RPM(drivers->kinematicInformant.getChassisIMUAngularVelocity(
        //             src::Informants::AngularAxis::YAW_AXIS,
        //             AngleUnit::Radians)));
        //     // rotationController.runControllerDerivateError(chassisErrorAngle);
        //     //rotationControllerOutputDisplay = rotationController.getOutput();

        //     // overwrite desired rotation with rotation controller output, range [-1, 1]
        //     desiredRotation = rotationController.getOutput();

        //     Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

        //     tap::algorithms::rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);
        // } else {  // if the gimbal is offline, run the normal manual drive command
            Chassis::Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
       // }

        //chassis does not care about rotation
        chassis->setTargetRPMs(desiredX, desiredY, 0.0f);
    }
    

    // if (wasFPressed && !drivers->remote.keyPressed(Remote::Key::F)) {
    //     wasFPressed = false;
    //     preferSpecificSpin = !ePressed.isExpired() || !qPressed.isExpired();

    //     if (comprisedCommandScheduler.isCommandScheduled(&ignoreGimbalCommand)) {
    //         if (preferSpecificSpin) {
    //             if (!ePressed.isExpired()) {
    //                 scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoRightCommand);
    //                 // qPressed.stop();
    //                 // ePressed.stop();
    //             }
    //             if (!qPressed.isExpired()) {
    //                 scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoLeftCommand);
    //                 // qPressed.stop();
    //                 // ePressed.stop();
    //             }
    //         } else {
    //             scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoCommand);
    //         }
    //     } else if (
    //         comprisedCommandScheduler.isCommandScheduled(&tokyoCommand) ||
    //         comprisedCommandScheduler.isCommandScheduled(&tokyoLeftCommand) ||
    //         comprisedCommandScheduler.isCommandScheduled(&tokyoRightCommand)) {
    //         comprisedCommandScheduler.addCommand(&ignoreGimbalCommand);
    //     }
    // }
    comprisedCommandScheduler.run();
}

void ChassisDUICommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &ignoreGimbalCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoLeftCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoRightCommand, interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisDUICommand::isReady() { return true; }

bool ChassisDUICommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE