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

#include "can.hpp"

#include "modm/architecture/interface/can_message.hpp"
#include "modm/platform.hpp"

#ifdef PLATFORM_HOSTED
#include "aruwlib/motor/motorsim/sim_handler.hpp"
#endif

#include "aruwlib/rm-dev-board-a/board.hpp"

#ifndef PLATFORM_HOSTED
using namespace modm::platform;
#endif
using namespace modm::literals;

void aruwlib::can::Can::initialize()
{
#ifndef PLATFORM_HOSTED
    CanFilter::setStartFilterBankForCan2(14);
    // initialize CAN 1
    Can1::connect<GpioD0::Rx, GpioD1::Tx>(Gpio::InputType::PullUp);
    Can1::initialize<Board::SystemClock, 1000_kbps>(9);
    // receive every message for CAN 1
    CanFilter::setFilter(
        0,
        CanFilter::FIFO0,
        CanFilter::StandardIdentifier(0),
        CanFilter::StandardFilterMask(0));
    Can2::connect<GpioB12::Rx, GpioB13::Tx>(Gpio::InputType::PullUp);
    Can2::initialize<Board::SystemClock, 1000_kbps>(12);
    // receive every message for CAN 2
    CanFilter::setFilter(
        14,
        CanFilter::FIFO0,
        CanFilter::StandardIdentifier(0),
        CanFilter::StandardFilterMask(0));
#endif
}

bool aruwlib::can::Can::isMessageAvailable(aruwlib::can::CanBus bus) const
{
#ifdef PLATFORM_HOSTED
    return aruwlib::motorsim::SimHandler::readyToSend(bus);
#else
    switch (bus)
    {
        case CanBus::CAN_BUS1:
            return Can1::isMessageAvailable();
        case CanBus::CAN_BUS2:
            return Can2::isMessageAvailable();
        default:
            return false;
    }
#endif
}

bool aruwlib::can::Can::getMessage(aruwlib::can::CanBus bus, modm::can::Message* message)
{
#ifdef PLATFORM_HOSTED
    return aruwlib::motorsim::SimHandler::sendMessage(bus, message);
#else
    switch (bus)
    {
        case CanBus::CAN_BUS1:
            return Can1::getMessage(*message);
        case CanBus::CAN_BUS2:
            return Can2::getMessage(*message);
        default:
            return false;
    }
#endif
}

bool aruwlib::can::Can::isReadyToSend(CanBus bus) const
{
#ifdef PLATFORM_HOSTED
    return true;
#else
    switch (bus)
    {
        case CanBus::CAN_BUS1:
            return Can1::isReadyToSend();
        case CanBus::CAN_BUS2:
            return Can2::isReadyToSend();
        default:
            return false;
    }
#endif
}

bool aruwlib::can::Can::sendMessage(CanBus bus, const modm::can::Message& message)
{
#ifdef PLATFORM_HOSTED
    return aruwlib::motorsim::SimHandler::getMessage(bus, message);
#else
    switch (bus)
    {
        case CanBus::CAN_BUS1:
            return Can1::sendMessage(message);
        case CanBus::CAN_BUS2:
            return Can2::sendMessage(message);
        default:
            return false;
    }
#endif
}
