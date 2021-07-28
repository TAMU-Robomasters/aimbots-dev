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

#ifndef SYSTEM_ERROR_HPP_
#define SYSTEM_ERROR_HPP_

namespace tap
{
namespace errors
{
/// Location of errors; subject to change
enum Location
{
    CAN_RX = 0,
    MOTOR_CONTROL,
    MPU6500,
    DJI_SERIAL,
    COMMAND_SCHEDULER,
    SUBSYSTEM,
    CONTROLLER_MAPPER,
    TURRET,
    SERVO,
    OLED_DISPLAY,
    LOCATION_AMOUNT,
    DJI_MOTOR_TX_HANDLER,
};

class SystemError
{
public:
    static const uint8_t ERROR_LOCATION_SIZE = 5;

    static const uint8_t ERROR_TYPE_SIZE = 3;

    constexpr SystemError()
        : lineNumber(0),
          description("default"),
          filename("none"),
          location(LOCATION_AMOUNT),
          errorType(ERROR_TYPE_AMOUNT)
    {
        static_assert(
            LOCATION_AMOUNT <= ERROR_LOCATION_SIZE * ERROR_LOCATION_SIZE,
            "You have declared too many locations!");
        static_assert(
            ERROR_TYPE_AMOUNT <= ERROR_TYPE_SIZE * ERROR_TYPE_SIZE,
            "You have declared too many error types!");
    }

    constexpr SystemError(const char *desc, int line, const char *file, Location l, uint8_t et)
        : lineNumber(line),
          description(desc),
          filename(file),
          location(l),
          errorType(et)
    {
        static_assert(
            LOCATION_AMOUNT <= ERROR_LOCATION_SIZE * ERROR_LOCATION_SIZE,
            "You have declared too many locations!");
        static_assert(
            ERROR_TYPE_AMOUNT <= ERROR_TYPE_SIZE * ERROR_TYPE_SIZE,
            "You have declared too many error types!");
    }

    constexpr int getLineNumber() const { return lineNumber; }

    const char *getDescription() const { return description; }

    const char *getFilename() const { return filename; }

    constexpr Location getLocation() const { return location; }

    constexpr uint8_t getErrorType() const { return errorType; }

private:
    int lineNumber;

    const char *description;

    const char *filename;

    static const uint8_t ERROR_TYPE_AMOUNT = 8;

    Location location;

    uint8_t errorType;
};  // class SystemError
}  // namespace errors
}  // namespace tap

#endif  // SYSTEM_ERROR_HPP_
