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

#include "ballistics.hpp"

#include "math_user_utils.hpp"

namespace tap::algorithms::ballistics
{
bool computeTravelTime(
    const modm::Vector3f &targetPosition,
    float bulletVelocity,
    float *travelTime,
    float *turretPitch)
{
    float horizontalDist = hypot(targetPosition.x, targetPosition.y);
    float bulletVelocitySquared = powf(bulletVelocity, 2);
    float sqrtTerm = powf(bulletVelocitySquared, 2) -
                     ACCELERATION_GRAVITY * (ACCELERATION_GRAVITY * powf(horizontalDist, 2) +
                                             2 * targetPosition.z * bulletVelocitySquared);

    if (sqrtTerm < 0)
    {
        return false;
    }

    // Equation obtained from the wikipedia page on projectile motion
    *turretPitch =
        -atan2(bulletVelocitySquared - sqrt(sqrtTerm), (ACCELERATION_GRAVITY * horizontalDist));

    // For vertical aiming, y_f = v_0*t - 0.5*g*t^2 -> t = (v_0 - sqrt((v_0)^2 - 2*g*y_f))/g
    // We use the negative root since the collision will happen on the first instance that the
    // trajectory reaches y_f
    if (compareFloatClose(*turretPitch, 0, 1E-2))
    {
        float sqrtTerm = powf(bulletVelocity, 2.0f) - 2 * ACCELERATION_GRAVITY * targetPosition.z;

        // If there isn't a real-valued root, there is no time where we can reach the target with
        // the given assumptions
        if (sqrtTerm < 0)
        {
            return false;
        }

        *travelTime = (bulletVelocity - sqrt(sqrtTerm)) / ACCELERATION_GRAVITY;
        return true;
    }

    // Equation obtained from the wikipedia page on projectile motion
    *travelTime = horizontalDist / (bulletVelocity * cos(*turretPitch));

    return !isnan(*turretPitch) && !isnan(*travelTime);
}

bool findTargetProjectileIntersection(
    MeasuredKinematicState targetInitialState,
    float bulletVelocity,
    uint8_t numIterations,
    float *turretPitch,
    float *turretYaw)
{
    modm::Vector3f projectedTargetPosition = targetInitialState.position;
    float projectedTravelTime;

    if (projectedTargetPosition.x == 0 && projectedTargetPosition.y == 0 &&
        projectedTargetPosition.z == 0)
    {
        return false;
    }

    for (int i = 0; i < numIterations; i++)
    {
        if (!computeTravelTime(
                projectedTargetPosition,
                bulletVelocity,
                &projectedTravelTime,
                turretPitch))
        {
            return false;
        }
        projectedTargetPosition = targetInitialState.projectForward(projectedTravelTime);
    }

    *turretYaw = atan2f(projectedTargetPosition.y, projectedTargetPosition.x);

    return !isnan(*turretPitch) && !isnan(*turretYaw);
}

}  // namespace tap::algorithms::ballistics
