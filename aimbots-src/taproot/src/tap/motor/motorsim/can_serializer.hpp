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

#ifndef TAPROOT_CAN_SERIALIZER_HPP_
#define TAPROOT_CAN_SERIALIZER_HPP_

#ifdef PLATFORM_HOSTED

#include <array>
#include <cstdint>

#include "tap/motor/dji_motor.hpp"

namespace modm::can
{
class Message;
}

namespace tap
{
namespace motorsim
{
class CanSerializer
{
public:
    /**
     * Parse a given CAN motor message into 4 motor input values.
     * Returns a 16-bit int array containing the 4 integer input values.
     */
    static std::array<int16_t, 4> parseMessage(const modm::can::Message* message);

    /**
     * Serialize the given motor feedback data into a CAN Message.
     * Returns a pointer to the created message.
     */
    static modm::can::Message serializeFeedback(
        int16_t angle,
        int16_t rpm,
        int16_t current,
        uint8_t port);

    /**
     * Return the corresponding port number on the can (0-7)
     * for the given MotorId (uint32_t alias).
     */
    static int8_t idToPort(tap::motor::MotorId id);

private:
    /* Constants */
    static constexpr uint8_t FEEDBACK_MESSAGE_SEND_LENGTH = 8;
};
}  // namespace motorsim

}  // namespace tap

#endif  // PLATFORM_HOSTED

#endif  // TAPROOT_CAN_SERIALIZER_HPP_
