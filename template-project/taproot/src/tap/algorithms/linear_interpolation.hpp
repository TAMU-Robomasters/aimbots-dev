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

#ifndef LINEAR_INTERPOLATION_HPP_
#define LINEAR_INTERPOLATION_HPP_

#include <cstdint>

namespace tap
{
namespace algorithms
{
/**
 * A class that allows you to linearly interpolate discrete measurements.
 * Useful for increasing the resolution of data measurements. For example,
 * if you receive input data at 10 ms intervals and are running a controller
 * ever 1 ms, you can use this class so that the data you receive won't be in
 * 13 ms steps, but rather will interpolate using past data to predict
 * the data until new data is received.
 *
 * Usecase example, pseudocode:
 *
 * \code
 * LinearInterpolation li;
 * while (true) {
 *     if (new value received) {
 *         li.update(value);
 *     }
 *     runcontroller(li.getInterpolated(tap::arch::clock::getTimeMilliseconds()));
 * }
 * \endcode
 */
class LinearInterpolation
{
public:
    LinearInterpolation();

    /**
     * Updates the interpolation using the newValue.
     *
     * @note only call this when you receive a new value (use remote rx
     *      counter to tell when there is new data from the remote, for
     *      example).
     * @param[in] newValue the new data used in the interpolation.
     * @param[in] currTime The time that this function was called.
     */
    void update(float newValue, uint32_t currTime);

    /**
     * Returns the current value, that is: \f$y\f$ in the equation
     * \f$y=slope\cdot (currTime - lastUpdateCallTime) + previousValue\f$.
     *
     * @note use a millisecond-resolution timer, e.g.
     *      tap::arch::clock::getTimeMilliseconds()
     * @param[in] currTime the current clock time, in ms.
     * @return the interpolated value.
     */
    float getInterpolatedValue(uint32_t currTime);

private:
    uint32_t lastUpdateCallTime;  /// The previous timestamp from when update was called.
    float previousValue;          /// The previous data value.
    float slope;  /// The current slope, calculated using the previous and most current data.
};                // class LinearInterpolation

}  // namespace algorithms

}  // namespace tap

#endif  // LINEAR_INTERPOLATION_HPP_
