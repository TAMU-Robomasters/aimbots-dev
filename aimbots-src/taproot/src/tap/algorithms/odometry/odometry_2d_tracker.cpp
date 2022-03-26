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
#include "odometry_2d_tracker.hpp"

#include <cmath>

#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "modm/math/geometry/angle.hpp"
#include "modm/math/geometry/vector.hpp"

#include "chassis_displacement_observer_interface.hpp"
#include "chassis_world_yaw_observer_interface.hpp"

namespace tap::algorithms::odometry
{
void Odometry2DTracker::update()
{
    modm::Vector<float, 3> chassisAbsoluteDisplacement;
    float chassisYaw = 0.0f;

    bool validDisplacementAvailable =
        chassisDisplacementObserver->getChassisDisplacement(&chassisAbsoluteDisplacement);
    bool validOrientationAvailable = chassisYawObserver->getChassisWorldYaw(&chassisYaw);

    if (validDisplacementAvailable)
    {
        if (!displacementPrimed)
        {
            // if this is first time valid displacement data was available we skip main logic
            displacementPrimed = true;
        }
        else if (displacementPrimed && validOrientationAvailable)
        {
            // Differentiate the absolute chassis displacement
            modm::Vector<float, 3> displacementChassisRelative =
                chassisAbsoluteDisplacement - prevChassisAbsoluteDisplacement;

            // Spec for `Location2D` seems to suggest it should only use normalized angles.
            // chassisYawObserver is specified to return normalized angles
            location.setOrientation(chassisYaw);
            location.move(
                modm::Vector2f(displacementChassisRelative.x, displacementChassisRelative.y));
        }

        prevChassisAbsoluteDisplacement = chassisAbsoluteDisplacement;
    }
}

}  // namespace tap::algorithms::odometry
