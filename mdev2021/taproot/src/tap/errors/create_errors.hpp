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
#ifndef CREATE_ERRORS_HPP_
#define CREATE_ERRORS_HPP_

#include "system_error.hpp"

namespace tap
{
namespace errors
{
enum class OLEDErrors : uint8_t
{
    INVALID_VERT_SCROLL_SIZE = 0,
    INVAILD_VERT_SCROLL_SMALLEST_AND_LARGEST_INDEX = 1,
    INVALID_VERT_SCROLL_MAX_ENTRIES = 2,
    NULLPTR_DJI_MOTOR_IN_MOTOR_SPECIFIC_MENU = 3,
};

enum class CanRxErrorType : uint8_t
{
    MOTOR_ID_OUT_OF_BOUNDS = 0,
    INVALID_REMOVE
};

enum class MotorControlErrorType : uint8_t
{
    NULL_MOTOR_ID = 0
};

enum class Mpu6500ErrorType : uint8_t
{
    IMU_DATA_NOT_INITIALIZED = 0,
    IMU_NOT_RECEIVING_PROPERLY
};

enum class DjiSerialErrorType : uint8_t
{
    MESSAGE_LENGTH_OVERFLOW = 0,
    INVALID_MESSAGE_LENGTH,
    CRC_FAILURE
};

enum class CommandSchedulerErrorType : uint8_t
{
    ADDING_NULLPTR_COMMAND = 0,
    ADDING_NULLPTR_SUBSYSTEM,
    ADDING_ALREADY_ADDED_SUBSYSTEM,
    MASTER_SCHEDULER_ALREADY_EXISTS,
    ADD_COMMAND_WHILE_TESTING,
    ADD_COMMAND_WITHOUT_REGISTERED_SUB,
    RUN_TIME_OVERFLOW,
    REMOVE_NULLPTR_COMMAND
};

enum class SubsystemErrorType : uint8_t
{
    MOTOR_OFFLINE = 0,
    ZERO_DESIRED_AGITATOR_ROTATE_TIME
};

enum class ControllerMapperErrorType : uint8_t
{
    INVALID_ADD = 0
};

enum class TurretErrorType : uint8_t
{
    MOTOR_OFFLINE = 0,
    INVALID_MOTOR_OUTPUT
};

enum class ServoErrorType : uint8_t
{
    INVALID_ADD = 0
};

enum class DjiMotorTxHandlerErrorType : uint8_t
{
    SEND_MESSAGE_FAILURE = 0
};

/**
 * Example for how to create and add an error. `drivers` is a pointer to an
 * `tap::Drivers`, which contains an instance of an `ErrorController`.
 *
 * @see ErrorController
 * @see SystemError
 *
 * ```cpp
 * RAISE_ERROR(
 *     drivers
 *     "Error in DJI Serial",
 *     tap::errors::Location::DJI_SERIAL,
 *     tap::errors::ErrorType::INVALID_CRC);
 * ```
 */
#define RAISE_ERROR(drivers, desc, l, et)                     \
    do                                                        \
    {                                                         \
        tap::errors::SystemError stringError(                 \
            desc,                                             \
            __LINE__,                                         \
            __FILE__,                                         \
            l,                                                \
            static_cast<uint8_t>(et));                        \
        drivers->errorController.addToErrorList(stringError); \
    } while (0);

}  // namespace errors

}  // namespace tap

#endif  // CREATE_ERRORS_HPP_
