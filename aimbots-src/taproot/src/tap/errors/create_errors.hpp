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

#ifndef TAPROOT_CREATE_ERRORS_HPP_
#define TAPROOT_CREATE_ERRORS_HPP_

#include "system_error.hpp"

namespace tap::errors
{
/**
 * Example for how to create and add an error. `drivers` is a pointer to an
 * `tap::Drivers`, which contains an instance of an `ErrorController`.
 *
 * @see ErrorController
 * @see SystemError
 *
 * ```cpp
 * RAISE_ERROR(drivers, "CRC8 failure");
 * ```
 */
#define RAISE_ERROR(drivers, desc)                                      \
    do                                                                  \
    {                                                                   \
        tap::errors::SystemError stringError(desc, __LINE__, __FILE__); \
        drivers->errorController.addToErrorList(stringError);           \
    } while (0);

}  // namespace tap::errors

#endif  // TAPROOT_TAPROOT_CREATE_ERRORS_HPP_
