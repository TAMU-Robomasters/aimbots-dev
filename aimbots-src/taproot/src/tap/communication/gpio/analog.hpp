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

#ifndef TAPROOT_ANALOG_HPP_
#define TAPROOT_ANALOG_HPP_

#include <cstdint>

#ifndef PLATFORM_HOSTED
#include "modm/platform/adc/adc_1.hpp"
#endif

#include "tap/util_macros.hpp"

namespace tap
{
namespace gpio
{
/**
 * To read from a pin call Read and pass the function a pin from the
 * analog Pin enum.
 */
class Analog
{
public:
    Analog() = default;
    DISALLOW_COPY_AND_ASSIGN(Analog)
    mockable ~Analog() = default;

    // Analog pins
    enum Pin
    {
        S,
        T,
        U,
        V,
        OledJoystick,
    };

    /**
     * Initializes the ADC and connects the configured analog pins to it.
     */
    mockable void init();

    /**
     * Reads voltage across the specified pin. Units in mV.
     */
    mockable uint16_t read(Analog::Pin pin) const;
};  // class Analog
}  // namespace gpio

}  // namespace tap

#endif  // TAPROOT_ANALOG_HPP_
