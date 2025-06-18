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

#include "distance_sensor.hpp"

namespace tap
{
namespace sensors
{
// Constructor to init boundaries
DistanceSensor::DistanceSensor(float minDistance, float maxDistance)
    : distance(0),
      minDistance(minDistance),
      maxDistance(maxDistance)
{
}

float DistanceSensor::getMinDistance() const { return minDistance; }

float DistanceSensor::getMaxDistance() const { return maxDistance; }

float DistanceSensor::getDistance() const { return distance; }

}  // namespace sensors

}  // namespace tap
