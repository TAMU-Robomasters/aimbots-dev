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

#ifndef COMMAND_SCHEDULER_TYPES_HPP_
#define COMMAND_SCHEDULER_TYPES_HPP_

#include <cinttypes>

namespace aruwlib::control
{
typedef uint64_t command_scheduler_bitmap_t;
typedef uint64_t subsystem_scheduler_bitmap_t;
}  // namespace aruwlib::control

#endif  // COMMAND_SCHEDULER_TYPES_HPP_
