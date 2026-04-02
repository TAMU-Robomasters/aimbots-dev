#include "power_can_communicator.hpp"

namespace src::Informants::PowerComms
{
PowerSensor::PowerSensor(tap::Drivers* drivers, tap::can::CanBus canBus)
    : tap::can::CanRxListener(drivers, CHASSIS_SENSOR_CAN_ID, canBus),
    sendDataTimer(500) // temp 500 ms make constant later ZHENG-HAO
    { }


float voltageDisplay = 67.0;
float currentDisplay = 69.0;
void PowerSensor::processMessage(const modm::can::Message& message) {
    this->heartbeat.restart(100);
    this->voltage = message.data[1] << 8 | message.data[0];
    this->current = message.data[3] << 8 | message.data[2];

    voltageDisplay = this->voltage;
    currentDisplay = this->current;
}

void PowerSensor::initialize() {
    this->attachSelfToRxHandler();
    this->heartbeat.restart(0);
    
}

// void TurretMCBCanComm::sendData()
// {
//     if (sendMcbDataTimer.execute())
//     {
//         modm::can::Message txMsg(TURRET_MCB_TX_CAN_ID, 1);
//         txMsg.setExtended(false);
//         txMsg.data[0] = txCommandMsgBitmask.value;
//         drivers->can.sendMessage(canBus, txMsg);

//         if (txCommandMsgBitmask.any(TxCommandMsgBitmask::RECALIBRATE_IMU))
//         {
//             yawRevolutions = 0;
//             pitchRevolutions = 0;
//         }

//         // set this calibrate flag to false so the calibrate command is only sent once
//         txCommandMsgBitmask.reset(TxCommandMsgBitmask::RECALIBRATE_IMU);
//     }

//     if (!isConnected())
//     {
//         yawRevolutions = 0;
//         pitchRevolutions = 0;
//     }
// }

int sendSuccDisplay = 69;
void PowerSensor::requestTest() {
    
     if (sendDataTimer.execute()) {
        modm::can::Message txMsg(CHASSIS_SENSOR_CAN_ID, 1, 0b01000101);
        txMsg.setExtended(false);
       // txMsg.data[0] = 0b01000101;
        sendSuccDisplay = drivers->can.sendMessage(canBus, txMsg); // 68 +- 1 HAHAHAHAHAHAHAHAHAHAHAHAHAHAHAHAHAHA
      //  sendSuccDisplay = -5;
     }
    // else{
       // sendSuccDisplay = 0;
    // }
    
}

// void PowerSensor::update() {
//     // bit message 0b 0000 0000 0000 0000 0000 0000 0000
    

// }

}