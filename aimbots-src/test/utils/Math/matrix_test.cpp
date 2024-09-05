/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aimbots-src.
 *
 * aimbots-src is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aimbots-src is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aimbots-src.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <gtest/gtest.h>

#include "utils/tools/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

using ::testing::NiceMock;

TEST(MatrixConstructorTest, rotation_matrix_creates_rot_matricies) {
    // ON_CALL(c.leftBackWheel, getEncoderUnwrapped);

    // clang-format off
    //define identity matrix
    float rotation_array[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    //check if any identity returns
    EXPECT_EQ(Matrix<float, 3, 3>(rotation_array), rotation_matrix(AngleUnit::Degrees, 45, 3));
            //define Rx
            rotation_array[0] = 1.0f; rotation_array[1] = 0.0f; rotation_array[2] = 0.0f;
            rotation_array[3] = 0.0f; rotation_array[4] = c; rotation_array[5] = -s;
            rotation_array[6] = 0.0f; rotation_array[7] = s; rotation_array[8] = c;
     EXPECT_EQ(Matrix<float, 3, 3>(rotation_array),rotation_matrix(AngleUnit::Degrees, 45, 0));
            //define Ry
            rotation_array[0] = c; rotation_array[1] = 0.0f; rotation_array[2] = s;
            rotation_array[3] = 0.0f; rotation_array[4] = 1.0; rotation_array[5] = 0.0f;
            rotation_array[6] = -s; rotation_array[7] = 0.0f; rotation_array[8] = c;
     EXPECT_EQ(Matrix<float, 3, 3>(rotation_array),rotation_matrix(AngleUnit::Degrees, 45, 1));
            //define Rz
            rotation_array[0] = c; rotation_array[1] = -s; rotation_array[2] = 0.0f;
            rotation_array[3] = s; rotation_array[4] = c; rotation_array[5] = 0.0f;
            rotation_array[6] = 0.0f; rotation_array[7] = 0.0f; rotation_array[8] = 1.0f;
     EXPECT_EQ(Matrix<float, 3, 3>(rotation_array),rotation_matrix(AngleUnit::Degrees, 45, 2));
    // clang-format on
}

// TEST(hello, world) { EXPECT_EQ(3, sum(1, 2)); }
