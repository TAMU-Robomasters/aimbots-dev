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

#include "analog_distance_sensor.hpp"

#include "tap/drivers.hpp"

namespace tap
{
namespace sensors
{
AnalogDistanceSensor::AnalogDistanceSensor(
    Drivers *drivers,
    float minDistance,
    float maxDistance,
    float m,
    float b,
    float offset,
    gpio::Analog::Pin pin)
    : DistanceSensor(minDistance, maxDistance),
      drivers(drivers),
      m(m),
      b(b),
      offset(offset),
      pin(pin)
{
}

float AnalogDistanceSensor::read()
{
    // Read analog pin and convert to volts
    float reading = drivers->analog.read(pin);

    // Linear model
    float linear = m * reading / 1000.0f + b;

    // Convert to cm distance
    distance = 1.0f / linear + offset;

    return validReading() ? distance : -1.0f;
}

bool AnalogDistanceSensor::validReading() const
{
    return (distance > minDistance) && (distance < maxDistance);
}
}  // namespace sensors

}  // namespace tap
