/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aimbots-dev.
 *
 * aimbots-dev is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aimbots-dev is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aimbots-dev.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "my_first_file.hpp"

TEST(hello, world) { EXPECT_EQ(3, sum(1, 2)); }
