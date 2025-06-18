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

#ifndef TAPROOT_CAN_RX_LISTENER_HPP_
#define TAPROOT_CAN_RX_LISTENER_HPP_

#include <cstdint>

#include "can.hpp"
#include "can_bus.hpp"

namespace modm::can
{
class Message;
}

namespace tap
{
class Drivers;
namespace can
{
/**
 * A class that when extended allows you to interface with the `can_rx_handler`.
 *
 * You must extend this class in order to use its features. So to use,
 * extend this class and implement `processMessage`. This pure virtual
 * function will be called in `can_rx_handler`'s processReceivedCanData.
 * Additionally, you must call `initialize`.
 *
 * @attention It is critical that you call `attachSelfToRxHandler`, otherwise
 *      the rx listener will not receive data from the rx handler.
 *
 * Below is a barebones example implementation of a `CanRxListener`.
 *
 * ```
 * class CanWatcher : public CanRxListener
 * {
 *  public:
 *     CanWatcher(uint32_t id, CanBus cB) : CanRxListener(id, cB) {}
 *
 *     void processMessage(const modm::can::Message& message) override
 *     {
 *          myMessage = message;
 *     }
 *
 *  private:
 *     modm::can::Message myMessage;
 * };
 * ```
 *
 * Then in `main.cpp`.
 *
 * ```
 * CanWatcher cw;
 * cw.attachSelfToRxHandler();
 *
 * int main(int argv, char **argc)
 * {
 *     while (true)
 *     {
 *         DoNotUse__getDrivers()->canRxHandler.pollCanData();
 *     }
 * }
 * ```
 *
 * @see `CanRxHandler`
 * @see `DjiMotor` for a usecase implementation of a `CanRxListener`.
 */
class CanRxListener
{
public:
    /**
     * Construct a new CanRxListener, must specify the can identifier
     * and the can bus the receive handler will be watching.
     *
     * @param[in] id the message identifier to be associated with this
     *      rx listener.
     * @param[in] cB the CanBus that you would like to watch.
     */
    CanRxListener(Drivers* drivers, uint32_t id, CanBus cB);

    /**
     * Here we remove the listener from receive interface.
     */
    ~CanRxListener();

    DISALLOW_COPY_AND_ASSIGN(CanRxListener)

    /**
     * Adds itself to the CanRxHandler.
     */
    mockable void attachSelfToRxHandler();

    /**
     * Called when a message is received with the particular id and
     * CAN bus.
     *
     * Pure virtual function: When you extend this class declare this
     * function.
     *
     * @param[in] message a new message received on the CAN bus.
     */
    virtual void processMessage(const modm::can::Message& message) = 0;

    /**
     * A variable necessary for the receive handler to determine
     * which message corresponds to which CanRxListener child class.
     */
    const uint32_t canIdentifier;

    /**
     * A variable necessary for the receive handler to determine
     * which message corresponds to which CanRxListener child class.
     */
    const CanBus canBus;

    Drivers* drivers;
};  // class CanRxListener

}  // namespace can

}  // namespace tap

#endif  // TAPROOT_CAN_RX_LISTENER_
