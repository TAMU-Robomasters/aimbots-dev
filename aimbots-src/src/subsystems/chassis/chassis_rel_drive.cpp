#include "chassis_rel_drive.hpp"

#include "utils/robot_specific_inc.hpp"

float xFromRemote = 0.0f;

int8_t chassisXDesiredWheelspeedWatch = 0;
int8_t chassisYDesiredWheelspeedWatch = 0;

namespace src::Chassis::Movement::Relative {

void calculateUserDesiredMovement(src::Drivers* drivers,
                                  ChassisSubsystem* chassis,
                                  float* desiredXSpeed,
                                  float* desiredYSpeed,
                                  float desiredChassisRotation) {
    if (drivers == nullptr || chassis == nullptr || desiredXSpeed == nullptr || desiredYSpeed == nullptr) {
        return;
    }

    const float MAX_WHEEL_SPEED = ChassisSubsystem::getMaxUserWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain =
        chassis->calculateRotationTranslationalGain(desiredChassisRotation) * MAX_WHEEL_SPEED;

    *desiredXSpeed = limitVal<float>(
                         drivers->controlOperatorInterface.getChassisXInput(),
                         -rTranslationalGain,
                         rTranslationalGain) *
                     MAX_WHEEL_SPEED;

    *desiredYSpeed = limitVal<float>(
                         drivers->controlOperatorInterface.getChassisYInput(),
                         -rTranslationalGain,
                         rTranslationalGain) *
                     MAX_WHEEL_SPEED;
}

void onExecute(src::Drivers* drivers, ChassisSubsystem* chassis) {
    float chassisRotationDesiredWheelspeed = drivers->controlOperatorInterface.getChassisRotationInput();

    float chassisXDesiredWheelspeed = 0.0f;
    float chassisYDesiredWheelspeed = 0.0f;

    calculateUserDesiredMovement(
        drivers,
        chassis,
        &chassisXDesiredWheelspeed,
        &chassisYDesiredWheelspeed,
        chassisRotationDesiredWheelspeed);

    // set chassis targets using setDesiredOutputs
    chassis->setTargetRPMs(
        chassisXDesiredWheelspeed,
        chassisYDesiredWheelspeed,
        chassisRotationDesiredWheelspeed);

    chassisXDesiredWheelspeedWatch = (int8_t)(chassisXDesiredWheelspeed * 127);
    chassisYDesiredWheelspeedWatch = (int8_t)(chassisYDesiredWheelspeed * 127);
    // chassis->setDesiredOutputs(0, 0, 0);
}
}  // namespace src::Chassis::Movement::Relative