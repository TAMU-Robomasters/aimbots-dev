#include "utils/robot_specific_inc.hpp"
#ifndef ENGINEER
#ifdef TOKYO_COMPATIBLE

#include <subsystems/chassis/chassis_helper.hpp>

#include "chassis_tokyo_command.hpp"
#warning "tokyo compatible"

namespace src::Chassis {

ChassisTokyoCommand::ChassisTokyoCommand(src::Drivers* drivers, ChassisSubsystem* chassis, src::Gimbal::GimbalSubsystem* gimbal)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisTokyoCommand::initialize() {
    // picks a random direction to begin rotation
    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? 1 : -1;
    rotationSpeedRamp.reset(chassis->getDesiredRotation());
}

float yawAngleFromChassisCenterDisplay = 0.0f;

void ChassisTokyoCommand::execute() {
    float desiredX = 0.0f;
    float desiredY = 0.0f;
    float desiredRotation = 0.0f;

    // we overwrite desiredRotation later if tokyo drifting
    Helper::getUserDesiredInput(drivers, chassis, &desiredX, &desiredY, &desiredRotation);

    if (gimbal->isOnline()) {
        float yawAngleFromChassisCenter = gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Radians);
        // this is wrapped between -PI and PI

        yawAngleFromChassisCenterDisplay = modm::toDegree(yawAngleFromChassisCenter);

        // The maximum speed that we're realistically able to achieve with the current power limit
        const float maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        desiredX *= TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed;
        desiredY *= TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed;

        const float translationalSpeedThreshold =
            maxWheelSpeed * TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER * TOKYO_TRANSLATION_THRESHOLD_TO_DECREASE_ROTATION_SPEED;

        float rampTarget = maxWheelSpeed * rotationDirection * TOKYO_ROTATIONAL_SPEED_FRACTION_OF_MAX;

        // reduces rotation speed when translation speed is high
        if (fabsf(desiredX) > translationalSpeedThreshold || fabsf(desiredY) > translationalSpeedThreshold) {
            rampTarget *= TOKYO_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING;
        }

        rotationSpeedRamp.setTarget(rampTarget);
        rotationSpeedRamp.update(TOKYO_ROTATIONAL_SPEED_INCREMENT);
        desiredRotation = rotationSpeedRamp.getValue();

        rotateVector(&desiredX, &desiredY, -yawAngleFromChassisCenter);

    } else {
        Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredRotation);
    }

    chassis->setTargetRPMs(desiredX, desiredY, desiredRotation);
}

void ChassisTokyoCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisTokyoCommand::isReady() { return true; }

bool ChassisTokyoCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif
#endif