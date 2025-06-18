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

#ifdef PLATFORM_HOSTED

#include "json_messages.hpp"

#include <iostream>
#include <string>

#include "tap/motor/dji_motor.hpp"

namespace tap
{
namespace communication
{
/**
 * The JSON namespace provides methods for working with JSON strings
 * which the MCB-simulator will be sending to the Windows simulator
 * and potentially vice versa.
 */
namespace json
{
using std::cout;
using std::string;

/**
 * returns JSON string representing motor message.
 */
string makeMotorMessage(const tap::motor::DjiMotor& motor)
{
    string jsonMessage =
        "{\"messageType\":\"motor\","
        "\"canBus\":" +
        std::to_string(static_cast<int>(motor.getCanBus()) + 1) + "," +
        "\"motorID\":" + std::to_string(motor.getMotorIdentifier()) + "," +
        "\"shaftRPM\":" + std::to_string(motor.getShaftRPM()) + "," +
        "\"torque\":" + std::to_string(motor.getTorque()) + "," +
        "\"encoderValue\":" + std::to_string(motor.getEncoderUnwrapped()) + "}";
    return jsonMessage;
}

}  // namespace json

}  // namespace communication

}  // namespace tap

#endif  // PLATFORM_HOSTED
