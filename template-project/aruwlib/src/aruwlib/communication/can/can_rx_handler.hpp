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

#ifndef CAN_RX_HANDLER_HPP_
#define CAN_RX_HANDLER_HPP_

#include "aruwlib/util_macros.hpp"

#include "can_bus.hpp"

namespace modm::can
{
class Message;
}

namespace aruwlib
{
class Drivers;

namespace can
{
class CanRxListener;

/**
 * A handler that stores pointers to CanRxListener and that watches
 * CAN 1 and CAN 2 for messages. If messages are received, it checks
 * its internal maps for a `CanRxListener` that matches the message
 * identifier and CAN bus and calls the listener's `processMessage`
 * function.
 *
 * Interfaces with modm receive data from CAN1 and CAN2 buses.
 *
 * To use, extend CanRxListener class and create a method called
 * processMessage. Next, call the function attachReceiveHandler,
 * which will add the class you instantiated to a list of classes
 * that will be handled on receive. The class you created and
 * attached will be called by the pollCanData function
 * every time there is a message available that has the CAN identifier
 * matching the identifier specified in the CanRxListener constructor.
 *
 * For proper closed loop motor control, it is necessary to have the
 * pollCanData function be called at a very high frequency,
 * so call this in a high frequency thread.
 *
 * @see `CanRxListener` for information about how to properly create
 *      add a listener to the handler.
 * @see `Can` for modm CAN wrapper functions.
 */
class CanRxHandler
{
public:
    CanRxHandler(Drivers* drivers);
    mockable ~CanRxHandler() = default;
    DISALLOW_COPY_AND_ASSIGN(CanRxHandler)

    /**
     * Call this function to add a CanRxListener to the list of CanRxListener's
     * that are referenced when a new CAN message is received.
     *
     * @note do not call this function in an object that is globally (i.e. not on
     *      that stack or heap) constructed. The map that the handler uses to
     *      store listeners may or not be properly allocated if you do and
     *      undefined behavior will follow.
     * @note if you attempt to add a listener with an identifier identical to
     *      something already in the `CanRxHandler`, an error is thrown and
     *      the handler does not add the listener.
     * @see `CanRxListener`
     * @param[in] listener the listener to be attached ot the handler.
     * @return true if listener successfully added, false otherwise.
     */
    mockable void attachReceiveHandler(CanRxListener* const listener);

    /**
     * Function handles receiving messages and calling the appropriate
     * processMessage function given the CAN bus and can identifier.
     *
     * @attention you should call this function as frequently as you receive
     *      messages if you want to receive the most up to date messages.
     *      modm's IQR puts CAN messages in a queue, and this function
     *      clears out the queue once it is called.
     */
    mockable void pollCanData();

    /**
     * Removes the passed in `CanRxListener` from the `CanRxHandler`. If the
     * listener isn't in the handler, the
     */
    mockable void removeReceiveHandler(const CanRxListener& rxListener);

private:
    static const int MAX_RECEIVE_UNIQUE_HEADER_CAN1 = 8;
    static const int MAX_RECEIVE_UNIQUE_HEADER_CAN2 = 8;
    static const int LOWEST_RECEIVE_ID = 0x201;

    Drivers* drivers;

    /**
     * Stores pointers to the `CanRxListeners` for CAN 1, referenced when
     * a new message is received.
     */
    CanRxListener* messageHandlerStoreCan1[MAX_RECEIVE_UNIQUE_HEADER_CAN1];

    /**
     * Stores pointers to the `CanRxListeners` for CAN 2, referenced when
     * a new message is received.
     */
    CanRxListener* messageHandlerStoreCan2[MAX_RECEIVE_UNIQUE_HEADER_CAN2];

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
#endif

    void attachReceiveHandler(
        CanRxListener* const CanRxHndl,
        CanRxListener** messageHandlerStore,
        int messageHandlerStoreSize);

    void processReceivedCanData(
        const modm::can::Message& rxMessage,
        CanRxListener* const* messageHandlerStore,
        int messageHandlerStoreSize);

    void removeReceiveHandler(
        const CanRxListener& listener,
        CanRxListener** messageHandlerStore,
        int messageHandlerStoreSize);

    aruwlib::can::CanRxListener** getHandlerStore(aruwlib::can::CanBus bus);
};  // class CanRxHandler

}  // namespace can

}  // namespace aruwlib

#endif  // CAN_RX_HANDLER_HPP_
