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

#ifndef DISTANCE_SENSOR_H_
#define DISTANCE_SENSOR_H_

namespace tap
{
namespace sensors
{
class DistanceSensor
{
public:
    /**
     * Constructor to init boundaries.
     *
     * @param[in] minDistance the min valid distance.
     * @param[in] maxDistance the max valid distance.
     */
    DistanceSensor(float minDistance, float maxDistance);

    virtual ~DistanceSensor() = default;

    virtual void init() = 0;

    /**
     * Read sensor and updates current distance.
     */
    virtual float read() = 0;

    /**
     * Checks if current reading is within bounds.
     */
    virtual bool validReading() const = 0;

    /**
     * Get minumum distance boundary.
     */
    float getMinDistance() const;

    /**
     * Get maximun distance boundary.
     */
    float getMaxDistance() const;

    /**
     * Get the current distance.
     */
    float getDistance() const;

protected:
    /// Distance from sensor
    float distance;

    /// Lower boundary for reliable readings
    float minDistance;

    /// Upper boundary for reliable readings
    float maxDistance;
};  // class DistanceSensor

}  // namespace sensors

}  // namespace tap

#endif  // DISTANCE_SENSOR_H_
