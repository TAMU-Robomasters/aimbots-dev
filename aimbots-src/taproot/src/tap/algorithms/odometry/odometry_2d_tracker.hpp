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

#ifndef TAPROOT_ODOMETRY_2D_TRACKER_HPP_
#define TAPROOT_ODOMETRY_2D_TRACKER_HPP_

#include "modm/math/geometry/location_2d.hpp"

#include "odometry_2d_interface.hpp"

namespace tap::algorithms::odometry
{
// Forward declarations
class ChassisWorldYawObserverInterface;
class ChassisDisplacementObserverInterface;

/**
 * Class for tracking the 2D position of an object over time.
 *
 * The faster update() is called the better.
 */
class Odometry2DTracker : public Odometry2DInterface
{
public:
    /**
     * @param[in] chassisYawObserver pointer to an object which implements the
     *      ChassisWorldYawObserverInterface. Should return the angle of the chassis
     *      forward vector relative to the x-axis of the field.
     * @param[in] chassisDisplacementObserver pointer to an object which implements the
     *      ChassisDisplacementObserverInterface. Used for getting the chassis displacement
     *
     * @note It is essential that the chassisYawObserver and chassisDisplacementObserver
     *      use the same positive z-axis. The getter interfaces should enforce that they
     *      they use positive z-axis is up, but it's worth noting here again.
     */
    Odometry2DTracker(
        ChassisWorldYawObserverInterface* chassisYawObserver,
        ChassisDisplacementObserverInterface* chassisDisplacementObserver)
        : chassisYawObserver(chassisYawObserver),
          chassisDisplacementObserver(chassisDisplacementObserver)
    {
    }

    /**
     * Run logic and update tracked chassis position. Call frequently for better
     * results.
     */
    void update();

    /**
     * @return The current odometry frame.
     * @see Odometry2DInterface::getCurrentLocation2D()
     */
    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

private:
    ChassisWorldYawObserverInterface* chassisYawObserver;
    ChassisDisplacementObserverInterface* chassisDisplacementObserver;
    // Location in reference frame
    modm::Location2D<float> location;
    // Velocity in reference frame
    modm::Vector2f velocity;
    // Previous chassis absolute displacement in chassis frame
    modm::Vector<float, 3> prevChassisAbsoluteDisplacement;
    // `true` iff `this` has been updated with valid chassis data at least once.
    bool displacementPrimed = false;
};

}  // namespace tap::algorithms::odometry

#endif  // TAPROOT_ODOMETRY_2D_TRACKER_HPP_
