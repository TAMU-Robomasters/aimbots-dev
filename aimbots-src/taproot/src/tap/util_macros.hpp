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

#ifndef TAPROOT_UTIL_MACROS_HPP_
#define TAPROOT_UTIL_MACROS_HPP_

#define DISALLOW_COPY_AND_ASSIGN(Typename) \
    Typename(const Typename &) = delete;   \
    Typename &operator=(const Typename &) = delete;

#ifdef PLATFORM_HOSTED
/// Wrap class functions that are not already virtual in this function if you wish to mock them.
#define mockable virtual
/// Use this instead of final if you want to mock a function when unit testing.
#define final_mockable
#else
/// Wrap class functions that are not already virtual in this function if you wish to mock them.
#define mockable
/// Use this instead of final if you want to mock a function when unit testing.
#define final_mockable final
#endif

#define UNUSED(var) (void)(var)

#endif  // TAPROOT_UTIL_MACROS_HPP_
