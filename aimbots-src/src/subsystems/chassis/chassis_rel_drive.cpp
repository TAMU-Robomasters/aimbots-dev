#include "chassis_rel_drive.hpp"

#include "utils/robot_specific_inc.hpp"

int8_t chassisYDesiredWheelspeedWatch = 0;
int8_t chassisXDesiredWheelspeedWatch = 0;

namespace src::Chassis::Helper {

void getUserDesiredInput(src::Drivers* drivers, ChassisSubsystem* chassis, float* desiredXInput, float* desiredYInput, float* desiredRotationInput) {
    if (drivers == nullptr || chassis == nullptr || desiredXInput == nullptr || desiredYInput == nullptr || desiredRotationInput == nullptr) {
        return;
    }

    *desiredXInput = drivers->controlOperatorInterface.getChassisXInput();
    *desiredYInput = drivers->controlOperatorInterface.getChassisYInput();
    *desiredRotationInput = drivers->controlOperatorInterface.getChassisRotationInput();
}

void rescaleDesiredInputToPowerLimitedSpeeds(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    float* desiredX,
    float* desiredY,
    float* desiredRotation) {
    if (desiredX == nullptr || desiredY == nullptr || desiredRotation == nullptr) {
        return;
    }

    // Gets the maximum speed that we're realistically able to achieve with the current power limit
    const float maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

    *desiredRotation *= maxWheelSpeed;

    // the maximum translational speed that we can achieve while maintaining the desired rotation speed
    float rTranslationalGain = chassis->calculateRotationLimitedTranslationalWheelspeed(*desiredRotation, maxWheelSpeed);

    *desiredX = limitVal<float>(*desiredX * maxWheelSpeed, -rTranslationalGain, rTranslationalGain);
    *desiredY = limitVal<float>(*desiredY * maxWheelSpeed, -rTranslationalGain, rTranslationalGain);
}

}  // namespace src::Chassis::Helper