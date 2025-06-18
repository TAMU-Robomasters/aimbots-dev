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

#ifndef TAPROOT_ANALOG_DISTANCE_SENSOR_HPP_
#define TAPROOT_ANALOG_DISTANCE_SENSOR_HPP_

#include "tap/communication/gpio/analog.hpp"

#include "distance_sensor.hpp"

namespace tap
{
class Drivers;
namespace sensors
{
/**
 * Basic analog IR Sensor.
 * - The distance conversion can be tweaked depending on the sensor.
 * - Min and max distance are in cm.
 *
 * See here (https://i.stack.imgur.com/babQg.png) for a graph of what
 * an IR sensor should look like. Datasheets will have specifics for
 * what the distance curve should look like. Thsi class gives a general
 * curve fit that first calculates a linear fit from the raw voltage
 * of the form \f$ y=mx+b \f$
 * and then the output is put into the equation \f$ dist = \frac{1}{linear} + offset \f$.
 */
class AnalogDistanceSensor : public DistanceSensor
{
public:
    /**
     * Constructor to initialize the analog IR boundary,
     * distance conversion, and analog pin.
     *
     * @param[in] minDistance the sensor's min valid distance
     * @param[in] maxDistance the sensor's max valid distance
     * @param[in] m the slope of the linear model that describes
     *      the relationship between analog input and distance.
     * @param[in] b the y intercept of the linear model that
     *      describes the relationship between analog input and
     *      distance.
     * @param[in] offset the offset in the non-linear portion of
     *      the model.
     * @param[in] pin the analog pin that the sensor is connected to.
     */
    AnalogDistanceSensor(
        Drivers *drivers,
        float minDistance,
        float maxDistance,
        float m,
        float b,
        float offset,
        gpio::Analog::Pin pin);

    /**
     * Reads the sensor, updates the current distance, and returns this reading.
     *
     * @return the updated value.  May or may not be valid. If it is not valid,
     *      -1 is returned.
     */
    float read() override;

    /**
     * Checks if current reading is within bounds.
     *
     * @return `true` if the reading is within the min and max distance, exclusive.
     *      Returns `false` otherwise.
     */
    bool validReading() const override;

private:
    Drivers *drivers;

    float m;  ///< Slope of linear model that converts raw analog values to distance values.
    float b;  ///< Y-intercept of linear model taht converts raw analog values to distance values.

    float offset;  ///< Offset value of inverse of the model.

    gpio::Analog::Pin pin;  ///< The analog pin which the sensor is connected to.
};

}  // namespace sensors

}  // namespace tap

#endif  // TAPROOT_ANALOG_DISTANCE_SENSOR_HPP_
