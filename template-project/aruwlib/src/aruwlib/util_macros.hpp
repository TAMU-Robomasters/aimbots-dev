/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef UTIL_MACROS_HPP_
#define UTIL_MACROS_HPP_

#define DISALLOW_COPY_AND_ASSIGN(Typename) \
    Typename(const Typename &) = delete;   \
    Typename &operator=(const Typename &) = delete;

#ifdef PLATFORM_HOSTED
/// Wrap class functions that are not already virtual in this function if you wish to mock them.
#define mockable virtual
#else
/// Wrap class functions that are not already virtual in this function if you wish to mock them.
#define mockable
#endif

#endif  // UTIL_MACROS_HPP_
