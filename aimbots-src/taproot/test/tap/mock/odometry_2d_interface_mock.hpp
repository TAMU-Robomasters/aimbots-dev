/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_ODOMETRY_2D_INTERFACE_MOCK_HPP_
#define TAPROOT_ODOMETRY_2D_INTERFACE_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"

namespace tap::mock
{
class Odometry2DInterfaceMock : public algorithms::odometry::Odometry2DInterface
{
public:
    MOCK_METHOD(modm::Location2D<float>, getCurrentLocation2D, (), (const override));
    MOCK_METHOD(modm::Vector2f, getCurrentVelocity2D, (), (const override));
};
}  // namespace tap::mock

#endif  // TAPROOT_ODOMETRY_2D_INTERFACE_MOCK_HPP_
