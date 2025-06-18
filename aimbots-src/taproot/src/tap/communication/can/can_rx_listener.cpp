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

#include "can_rx_listener.hpp"

#include "tap/drivers.hpp"

#include "modm/architecture/interface/can_message.hpp"

namespace tap
{
namespace can
{
CanRxListener::CanRxListener(Drivers *drivers, uint32_t id, CanBus cB)
    : canIdentifier(id),
      canBus(cB),
      drivers(drivers)
{
}

void CanRxListener::attachSelfToRxHandler() { drivers->canRxHandler.attachReceiveHandler(this); }

CanRxListener::~CanRxListener() { drivers->canRxHandler.removeReceiveHandler(*this); }
}  // namespace can

}  // namespace tap
