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

#include "imu_rx_listener.hpp"

#include "tap/architecture/endianness_wrappers.hpp"

#include "modm/architecture/interface/can.hpp"

namespace tap::can
{
ImuRxListener::ImuRxListener(Drivers* drivers)
    : drivers(drivers),
      angleGyroMessageHandler(
          drivers,
          ANGLE_GYRO_MESSAGE_CAN_ID,
          IMU_MSG_CAN_BUS,
          this,
          &ImuRxListener::handleAngleGyroMessage)
{
}

ImuRxListener::ImuRxHandler::ImuRxHandler(
    Drivers* drivers,
    uint32_t id,
    CanBus cB,
    ImuRxListener* msgHandler,
    ImuRxListenerFunc funcToCall)
    : CanRxListener(drivers, id, cB),
      msgHandler(msgHandler),
      funcToCall(funcToCall)
{
}

void ImuRxListener::init() { angleGyroMessageHandler.attachSelfToRxHandler(); }

void ImuRxListener::ImuRxHandler::processMessage(const modm::can::Message& message)
{
    (msgHandler->*funcToCall)(message);
}

void ImuRxListener::handleAngleGyroMessage(const modm::can::Message& message)
{
    arch::convertFromLittleEndian(&yaw, message.data);
    arch::convertFromLittleEndian(&rawGz, message.data + 4);

    imuConnectedTimeout.restart(DISCONNECT_TIMEOUT_PERIOD);
}
}  // namespace tap::can
