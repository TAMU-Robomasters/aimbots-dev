#include "chassis_rel_drive.hpp"

#include "utils/robot_specific_inc.hpp"

float xFromRemote = 0.0f;

int8_t chassisXDesiredWheelspeedWatch = 0;
int8_t chassisYDesiredWheelspeedWatch = 0;

namespace src::Chassis::Movement::Independent {

void calculateUserDesiredMovement(src::Drivers* drivers,
                                  ChassisSubsystem* chassis,
                                  float* desiredXSpeed,
                                  float* desiredYSpeed,
                                  float desiredChassisRotation) {
    if (drivers == nullptr || chassis == nullptr || desiredXSpeed == nullptr || desiredYSpeed == nullptr) {
        return;
    }

    const float maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain =
        chassis->calculateRotationTranslationalGain(desiredChassisRotation) * maxWheelSpeed;

    *desiredXSpeed = limitVal<float>(
                         drivers->controlOperatorInterface.getChassisXInput(),
                         -rTranslationalGain,
                         rTranslationalGain) *
                     maxWheelSpeed;

    *desiredYSpeed = limitVal<float>(
                         drivers->controlOperatorInterface.getChassisYInput(),
                         -rTranslationalGain,
                         rTranslationalGain) *
                     maxWheelSpeed;
}

float chassisRotationInputDisplay = 0.0f;
float chassisYInputDisplay = 0.0f;
float chassisXInputDisplay = 0.0f;

float chassisXDesiredWheelspeedDisplay = 0.0f;
float chassisYDesiredWheelspeedDisplay = 0.0f;
float chassisRotationDesiredWheelspeedDisplay = 0.0f;

void onExecute(src::Drivers* drivers, ChassisSubsystem* chassis) {
    const float maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

    float chassisRotationDesiredWheelspeed =
        drivers->controlOperatorInterface.getChassisRotationInput() * maxWheelSpeed;

    chassisRotationInputDisplay = drivers->controlOperatorInterface.getChassisRotationInput();

    chassisXInputDisplay = drivers->controlOperatorInterface.getChassisXInput();
    chassisYInputDisplay = drivers->controlOperatorInterface.getChassisYInput();

    float chassisXDesiredWheelspeed = 0.0f;
    float chassisYDesiredWheelspeed = 0.0f;

    calculateUserDesiredMovement(
        drivers,
        chassis,
        &chassisXDesiredWheelspeed,
        &chassisYDesiredWheelspeed,
        chassisRotationDesiredWheelspeed);

    chassisXDesiredWheelspeedDisplay = chassisXDesiredWheelspeed;
    chassisYDesiredWheelspeedDisplay = chassisYDesiredWheelspeed;
    chassisRotationDesiredWheelspeedDisplay = chassisRotationDesiredWheelspeed;

    // set chassis targets using setDesiredOutputs
    chassis->setTargetRPMs(
        chassisXDesiredWheelspeed,
        chassisYDesiredWheelspeed,
        chassisRotationDesiredWheelspeed);
}
}  // namespace src::Chassis::Movement::Independent