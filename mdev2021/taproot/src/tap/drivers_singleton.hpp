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

#ifndef ENV_UNIT_TESTS

#ifndef DRIVERS_SINGLETON_HPP_
#define DRIVERS_SINGLETON_HPP_

#include "drivers.hpp"

namespace tap
{
/**
 * @return The singleton instance of the Drivers class. This is the only instance of the
 *      Drivers class that should be created anywhere in the non-unit test framework.
 * @note It is likely that you will never have to use this. There are only two files you
 *      should be calling this function from -- `main.cpp` and `*_control.cpp`, either to
 *      run I/O stuff and to add a Drivers pointer to an instance of a Subsystem or Command.
 */
Drivers *DoNotUse_getDrivers();
using driversFunc = tap::Drivers *(*)();
}  // namespace tap

#endif  // DRIVERS_SINGLETON_HPP_

#endif
