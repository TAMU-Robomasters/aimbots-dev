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

#ifndef TAPROOT_DIGITAL_HPP_
#define TAPROOT_DIGITAL_HPP_

#ifndef PLATFORM_HOSTED
#include "modm/platform.hpp"
#endif

#include <cstdint>

#include "tap/util_macros.hpp"

namespace tap
{
namespace gpio
{
/**
 * Similar to the Analog class, wraps input and output digital pins.
 *
 * @see InputPin for the input pins configured (pin names correspond
 *      to RoboMaster dev board definitions).
 * @see OutputPin for the output pins configured (pin names correspond
 *      to RoboMaster dev board definitions).
 */
class Digital
{
public:
    Digital() = default;
    DISALLOW_COPY_AND_ASSIGN(Digital)
    mockable ~Digital() = default;

    /**
     * Currently enabled digital input pins.
     */
    enum InputPin
    {
        A,
        B,
        C,
        D,
    };

    /**
     * Currently enabled digital output pins.
     */
    enum OutputPin
    {
        E,
        F,
        G,
        H,
        Laser,
    };

#ifdef PLATFORM_HOSTED
    enum InputPullMode
    {
        Floating,
        PullUp,
        PullDown
    };
#else
    /**
     * This references a struct defined by modm.  Can either be floating, pull-up, or pull-down.
     */
    using InputPullMode = modm::platform::Gpio::InputType;
#endif

    /**
     * Initializes all pins as output/input pins. Does not handle configuring
     * pin types (@see configureInputPullMode).
     */
    mockable void init();

    /**
     * By default input pins are floating. Configure them to have a pull-up
     * or pull-down resistor here.
     *
     * @param[in] pin the InputPin to configure.
     * @param[in] mode the pull mode to be enabled.
     */
    mockable void configureInputPullMode(InputPin pin, InputPullMode mode);

    /**
     * Sets the digital OutputPin either high or low.
     *
     * @param[in] pin the OutputPin to set.
     * @param[in] isSet `true` to send high, `false` to send low.
     */
    mockable void set(OutputPin pin, bool isSet);

    /**
     * Reads from an InputPin.
     *
     * @param[in] pin the InputPin to read from.
     * @return `true` if the pin is pulled high and `false` otherwise.
     */
    mockable bool read(InputPin pin) const;
};  // class Digital

}  // namespace gpio

}  // namespace tap

#endif  // TAPROOT_DIGITAL_HPP_
