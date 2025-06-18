/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "dji_motor.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"

#ifdef PLATFORM_HOSTED
#include <iostream>

#include "tap/communication/tcp-server/json_messages.hpp"
#include "tap/communication/tcp-server/tcp_server.hpp"

#include "modm/architecture/interface/can_message.hpp"
#endif

namespace tap
{
namespace motor
{
DjiMotor::~DjiMotor() { drivers->djiMotorTxHandler.removeFromMotorManager(*this); }

DjiMotor::DjiMotor(
    Drivers* drivers,
    MotorId desMotorIdentifier,
    tap::can::CanBus motorCanBus,
    bool isInverted,
    const char* name,
    uint16_t encoderWrapped,
    int64_t encoderRevolutions)
    : CanRxListener(drivers, static_cast<uint32_t>(desMotorIdentifier), motorCanBus),
      motorName(name),
      drivers(drivers),
      motorIdentifier(desMotorIdentifier),
      motorCanBus(motorCanBus),
      desiredOutput(0),
      shaftRPM(0),
      temperature(0),
      torque(0),
      motorInverted(isInverted),
      encoderWrapped(encoderWrapped),
      encoderRevolutions(encoderRevolutions)
{
    motorDisconnectTimeout.stop();
}

void DjiMotor::initialize()
{
    drivers->djiMotorTxHandler.addMotorToManager(this);
    attachSelfToRxHandler();
}

void DjiMotor::parseCanRxData(const modm::can::Message& message)
{
    if (message.getIdentifier() != DjiMotor::getMotorIdentifier())
    {
        return;
    }
    uint16_t encoderActual =
        static_cast<uint16_t>(message.data[0] << 8 | message.data[1]);        // encoder value
    shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
    shaftRPM = motorInverted ? -shaftRPM : shaftRPM;
    torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // torque
    torque = motorInverted ? -torque : torque;
    temperature = static_cast<int8_t>(message.data[6]);  // temperature

    // restart disconnect timer, since you just received a message from the motor
    motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

    // invert motor if necessary
    encoderActual = motorInverted ? ENC_RESOLUTION - 1 - encoderActual : encoderActual;
    updateEncoderValue(encoderActual);

#ifdef PLATFORM_HOSTED
    /* So the trace of this function to main() goes through a lot, but inside of main
     * this function is eventually called through a sequence of functions by
     * canRxHandler.pollCanData(). In fact this seems to be the only driver that
     * extends the can_rx_listener class... so it's the only thing that uses CAN? */
    using namespace tap::communication;
    std::string jsonMessage = json::makeMotorMessage(*this);
    const char* jsonCString = jsonMessage.c_str();
    TCPServer::MainServer()->writeToClient(jsonCString, strlen(jsonCString));
#endif
}

void DjiMotor::setDesiredOutput(int32_t desiredOutput)
{
    int16_t desOutputNotInverted =
        static_cast<int16_t>(tap::algorithms::limitVal<int32_t>(desiredOutput, SHRT_MIN, SHRT_MAX));
    this->desiredOutput = motorInverted ? -desOutputNotInverted : desOutputNotInverted;
}

bool DjiMotor::isMotorOnline() const
{
    /*
     * motor online if the disconnect timout has not expired (if it received message but
     * somehow got disconnected) and the timeout hasn't been stopped (initially, the timeout
     * is stopped)
     */
    return !motorDisconnectTimeout.isExpired() && !motorDisconnectTimeout.isStopped();
}

void DjiMotor::serializeCanSendData(modm::can::Message* txMessage) const
{
    int id = DJI_MOTOR_TO_NORMALIZED_ID(this->getMotorIdentifier());  // number between 0 and 7
    // this method assumes you have choosen the correct message
    // to send the data in. Is blind to message type and is a private method
    // that I use accordingly.
    id %= 4;
    txMessage->data[2 * id] = this->getOutputDesired() >> 8;
    txMessage->data[2 * id + 1] = this->getOutputDesired() & 0xFF;
}

// getter functions
int16_t DjiMotor::getOutputDesired() const { return desiredOutput; }

uint32_t DjiMotor::getMotorIdentifier() const { return motorIdentifier; }

int8_t DjiMotor::getTemperature() const { return temperature; }

int16_t DjiMotor::getTorque() const { return torque; }

int16_t DjiMotor::getShaftRPM() const { return shaftRPM; }

bool DjiMotor::isMotorInverted() const { return motorInverted; }

tap::can::CanBus DjiMotor::getCanBus() const { return motorCanBus; }

const char* DjiMotor::getName() const { return motorName; }

int64_t DjiMotor::getEncoderUnwrapped() const
{
    return static_cast<int64_t>(encoderWrapped) +
           static_cast<int64_t>(ENC_RESOLUTION) * encoderRevolutions;
}

uint16_t DjiMotor::getEncoderWrapped() const { return encoderWrapped; }

void DjiMotor::updateEncoderValue(uint16_t newEncWrapped)
{
    int16_t enc_dif = newEncWrapped - encoderWrapped;
    if (enc_dif < -ENC_RESOLUTION / 2)
    {
        encoderRevolutions++;
    }
    else if (enc_dif > ENC_RESOLUTION / 2)
    {
        encoderRevolutions--;
    }
    encoderWrapped = newEncWrapped;
}
}  // namespace motor

}  // namespace tap
