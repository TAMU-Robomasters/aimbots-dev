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

#ifndef TAPROOT_JSON_MESSAGES_HPP_
#define TAPROOT_JSON_MESSAGES_HPP_

#ifdef PLATFORM_HOSTED

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
/**
 * Pre: motorValues should be at least size 4 and contain valid
 * current values.
 * Post: Returns a string in valid JSON format that says that it is a
 * motor message, whether or not it is for the upper 4 motors based
 * on "upperMessage", and with the four current values in motorValues.
 */
std::string makeMotorMessage(const tap::motor::DjiMotor& motor);

}  // namespace json

}  // namespace communication

}  // namespace tap

#endif  // PLATFORM_HOSTED

#endif  // TAPROOT_JSON_MESSAGES_HPP_
