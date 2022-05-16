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

#ifndef TAPROOT_ODOMETRY_2D_INTERFACE_HPP_
#define TAPROOT_ODOMETRY_2D_INTERFACE_HPP_

// Include instead of forward declare because template uses default arguments
#include "modm/math/geometry/location_2d.hpp"

namespace tap::algorithms::odometry
{
/**
 * @brief Interface for retrieving the position of a robot chassis in 2 dimensions.
 *
 * The coordinates measured are relative to some reference coordinate frame which has a
 * fixed position and fixed orientation. The reference coordinate frame's position and
 * orientation are implementation defined.
 *
 * As to why this interface exists separately from a more generic 3D implementation:
 * 2D odometry requires less sensor data (all it needs is chassis yaw in the world
 * frame), and performs simpler computations than a 3D odometry system would require.
 */
class Odometry2DInterface
{
public:
    /**
     * @return The current location (x and y coordinate) and orientation (in radians).
     */
    virtual modm::Location2D<float> getCurrentLocation2D() const = 0;

    /**
     * @return The current x and y velocity (in m/s).
     */
    virtual modm::Vector2f getCurrentVelocity2D() const = 0;
};

}  // namespace tap::algorithms::odometry

#endif  // TAPROOT_ODOMETRY_2D_INTERFACE_HPP_
