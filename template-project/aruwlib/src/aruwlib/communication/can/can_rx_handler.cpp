/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "can_rx_handler.hpp"

#include "aruwlib/drivers.hpp"
#include "aruwlib/errors/create_errors.hpp"

#include "modm/architecture/interface/assert.h"
#include "modm/architecture/interface/can.hpp"

#include "can_rx_listener.hpp"

namespace aruwlib
{
namespace can
{
CanRxHandler::CanRxHandler(Drivers* drivers)
    : drivers(drivers),
      messageHandlerStoreCan1(),
      messageHandlerStoreCan2()
{
}

void CanRxHandler::attachReceiveHandler(CanRxListener* const listener)
{
    if (listener->canBus == can::CanBus::CAN_BUS1)
    {
        attachReceiveHandler(listener, messageHandlerStoreCan1, MAX_RECEIVE_UNIQUE_HEADER_CAN1);
    }
    else
    {
        attachReceiveHandler(listener, messageHandlerStoreCan2, MAX_RECEIVE_UNIQUE_HEADER_CAN2);
    }
}

void CanRxHandler::attachReceiveHandler(
    CanRxListener* const CanRxHndl,
    CanRxListener** messageHandlerStore,
    int messageHandlerStoreSize)
{
    int32_t id = DJI_MOTOR_NORMALIZED_ID(CanRxHndl->canIdentifier);
    bool receiveInterfaceOverloaded = messageHandlerStore[id] != nullptr;
    bool receiveAttachSuccess =
        !receiveInterfaceOverloaded || (id >= 0 && id < messageHandlerStoreSize);
    modm_assert(receiveAttachSuccess, "can1", "overloading", 1);

    messageHandlerStore[id] = CanRxHndl;
}

void CanRxHandler::pollCanData()
{
    modm::can::Message rxMessage;
    // handle incoming CAN 1 messages
    if (drivers->can.getMessage(CanBus::CAN_BUS1, &rxMessage))
    {
        processReceivedCanData(rxMessage, messageHandlerStoreCan1, MAX_RECEIVE_UNIQUE_HEADER_CAN1);
    }
    // handle incoming CAN 2 messages
    if (drivers->can.getMessage(CanBus::CAN_BUS2, &rxMessage))
    {
        processReceivedCanData(rxMessage, messageHandlerStoreCan2, MAX_RECEIVE_UNIQUE_HEADER_CAN2);
    }
}

void CanRxHandler::processReceivedCanData(
    const modm::can::Message& rxMessage,
    CanRxListener* const* messageHandlerStore,
    int messageHandlerStoreSize)
{
    int32_t handlerStoreId = DJI_MOTOR_NORMALIZED_ID(rxMessage.getIdentifier());
    if (handlerStoreId >= 0 && handlerStoreId < messageHandlerStoreSize)
    {
        if (messageHandlerStore[handlerStoreId] != nullptr)
        {
            messageHandlerStore[handlerStoreId]->processMessage(rxMessage);
        }
    }
    else
    {
        RAISE_ERROR(
            drivers,
            "Invalid can id received - not between 0x200 and 0x208",
            aruwlib::errors::Location::CAN_RX,
            aruwlib::errors::CanRxErrorType::MOTOR_ID_OUT_OF_BOUNDS);
    }
}

void CanRxHandler::removeReceiveHandler(const CanRxListener& rxListner)
{
    if (rxListner.canBus == CanBus::CAN_BUS1)
    {
        removeReceiveHandler(rxListner, messageHandlerStoreCan1, MAX_RECEIVE_UNIQUE_HEADER_CAN1);
    }
    else
    {
        removeReceiveHandler(rxListner, messageHandlerStoreCan2, MAX_RECEIVE_UNIQUE_HEADER_CAN2);
    }
}

void CanRxHandler::removeReceiveHandler(
    const CanRxListener& listener,
    CanRxListener** messageHandlerStore,
    int messageHandlerStoreSize)
{
    int id = DJI_MOTOR_NORMALIZED_ID(listener.canIdentifier);
    if (id < 0 || id >= messageHandlerStoreSize)
    {
        RAISE_ERROR(
            drivers,
            "index out of bounds",
            aruwlib::errors::CAN_RX,
            aruwlib::errors::CanRxErrorType::INVALID_REMOVE);
        return;
    }
    messageHandlerStore[id] = nullptr;
}

aruwlib::can::CanRxListener** CanRxHandler::getHandlerStore(aruwlib::can::CanBus bus)
{
    if (bus == aruwlib::can::CanBus::CAN_BUS1)
    {
        return this->messageHandlerStoreCan1;
    }
    return this->messageHandlerStoreCan2;
}
}  // namespace can

}  // namespace aruwlib
