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

#include "dji_motor_tx_handler.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/architecture/interface/assert.h"
#include "modm/architecture/interface/can_message.hpp"

#define CAN_DJI_MESSAGE_SEND_LENGTH 8
#define CAN_DJI_LOW_IDENTIFIER 0X200
#define CAN_DJI_HIGH_IDENTIFIER 0X1FF

namespace tap
{
namespace motor
{
void DjiMotorTxHandler::addMotorToManager(DjiMotor** canMotorStore, DjiMotor* const motor)
{
    int16_t idIndex = DJI_MOTOR_NORMALIZED_ID(motor->getMotorIdentifier());
    bool motorOverloaded = canMotorStore[idIndex] != nullptr;
    bool motorOutOfBounds = (idIndex < 0) || (idIndex >= DJI_MOTORS_PER_CAN);
    // kill start
    modm_assert(!motorOverloaded && !motorOutOfBounds, "DjiMotorTxHandler:can", "overloading", 1);
    canMotorStore[idIndex] = motor;
}

void DjiMotorTxHandler::addMotorToManager(DjiMotor* motor)
{
    // add new motor to either the can1 or can2 motor store
    // because we checked to see if the motor is overloaded, we will
    // never have to worry about overfilling the CanxMotorStore array
    if (motor->getCanBus() == tap::can::CanBus::CAN_BUS1)
    {
        addMotorToManager(can1MotorStore, motor);
    }
    else
    {
        addMotorToManager(can2MotorStore, motor);
    }
}

void DjiMotorTxHandler::processCanSendData()
{
    // set up new can messages to be sent via CAN bus 1 and 2
    modm::can::Message can1MessageLow(CAN_DJI_LOW_IDENTIFIER, CAN_DJI_MESSAGE_SEND_LENGTH);
    can1MessageLow.setExtended(false);

    modm::can::Message can1MessageHigh(CAN_DJI_HIGH_IDENTIFIER, CAN_DJI_MESSAGE_SEND_LENGTH);
    can1MessageHigh.setExtended(false);

    modm::can::Message can2MessageLow(CAN_DJI_LOW_IDENTIFIER, CAN_DJI_MESSAGE_SEND_LENGTH);
    can2MessageLow.setExtended(false);

    modm::can::Message can2MessageHigh(CAN_DJI_HIGH_IDENTIFIER, CAN_DJI_MESSAGE_SEND_LENGTH);
    can2MessageHigh.setExtended(false);

    zeroTxMessage(&can1MessageLow);
    zeroTxMessage(&can1MessageHigh);
    zeroTxMessage(&can2MessageLow);
    zeroTxMessage(&can2MessageHigh);

    serializeMotorStoreSendData(can1MotorStore, &can1MessageLow, &can1MessageHigh);
    serializeMotorStoreSendData(can2MotorStore, &can2MessageLow, &can2MessageHigh);

    bool messageFailure = false;
    if (drivers->can.isReadyToSend(can::CanBus::CAN_BUS1))
    {
        messageFailure |= drivers->can.sendMessage(can::CanBus::CAN_BUS1, can1MessageLow);
        messageFailure |= drivers->can.sendMessage(can::CanBus::CAN_BUS1, can1MessageHigh);
    }
    if (drivers->can.isReadyToSend(can::CanBus::CAN_BUS2))
    {
        messageFailure |= drivers->can.sendMessage(can::CanBus::CAN_BUS2, can2MessageLow);
        messageFailure |= drivers->can.sendMessage(can::CanBus::CAN_BUS2, can2MessageHigh);
    }
    if (messageFailure)
    {
        RAISE_ERROR(drivers, "sendMessage failure");
    }
}

void DjiMotorTxHandler::serializeMotorStoreSendData(
    DjiMotor** canMotorStore,
    modm::can::Message* messageLow,
    modm::can::Message* messageHigh)
{
    for (int i = 0; i < DJI_MOTORS_PER_CAN; i++)
    {
        const DjiMotor* const motor = canMotorStore[i];
        if (motor != nullptr)
        {
            if (motor->getMotorIdentifier() - 0x200 <= 4)
            {
                motor->serializeCanSendData(messageLow);
            }
            else
            {
                motor->serializeCanSendData(messageHigh);
            }
        }
    }
}

void DjiMotorTxHandler::removeFromMotorManager(const DjiMotor& motor)
{
    if (motor.getCanBus() == tap::can::CanBus::CAN_BUS1)
    {
        removeFromMotorManager(motor, can1MotorStore);
    }
    else
    {
        removeFromMotorManager(motor, can2MotorStore);
    }
}

void DjiMotorTxHandler::removeFromMotorManager(const DjiMotor& motor, DjiMotor** motorStore)
{
    uint32_t id = DJI_MOTOR_NORMALIZED_ID(motor.getMotorIdentifier());
    if (motorStore[id] == nullptr)
    {
        // error, trying to remove something that doesn't exist!
        RAISE_ERROR(drivers, "trying to remove something that doesn't exist");
        return;
    }
    motorStore[id] = nullptr;
}

void DjiMotorTxHandler::zeroTxMessage(modm::can::Message* message)
{
    for (int i = 0; i < message->length; i++)
    {
        message->data[i] = 0;
    }
}

DjiMotor const* DjiMotorTxHandler::getCan1Motor(MotorId motorId)
{
    return can1MotorStore[DJI_MOTOR_NORMALIZED_ID(motorId)];
}

DjiMotor const* DjiMotorTxHandler::getCan2Motor(MotorId motorId)
{
    return can2MotorStore[DJI_MOTOR_NORMALIZED_ID(motorId)];
}
}  // namespace motor

}  // namespace tap
