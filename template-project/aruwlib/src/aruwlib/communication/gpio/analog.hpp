/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ANALOG_HPP_
#define ANALOG_HPP_

#include <cstdint>

#ifndef PLATFORM_HOSTED
#include "modm/platform/adc/adc_1.hpp"
#endif

#include "aruwlib/util_macros.hpp"

namespace aruwlib
{
namespace gpio
{
/**
 * Analog output pins are pins W, X, Y, and Z (board pins PI5, PI6, PI7, PI2)
 * as referenced by the pin naming on the RoboMaster type A board.
 *
 * To read from a pin call Read and pass the function a value (S - V) from the
 * analog inPin enum.
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
        S = 1,
        T,
        U,
        V,
        OLED_JOYSTICK,
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

}  // namespace aruwlib

#endif  // ANALOG_HPP_
