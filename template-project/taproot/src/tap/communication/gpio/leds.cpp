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

#include "leds.hpp"

#include "tap/board/board.hpp"
#include "tap/util_macros.hpp"

using namespace Board;

namespace tap
{
namespace gpio
{
void Leds::init()
{
#ifndef PLATFORM_HOSTED
    // init Leds
    LedsPort::setOutput(modm::Gpio::Low);
#endif
}

void Leds::set(Leds::LedPin pin, bool isSet)
{
#ifdef PLATFORM_HOSTED
    UNUSED(pin);
    UNUSED(isSet);
#else
    switch (pin)
    {
        case Leds::LedPin::A:
            LedA::set(isSet);
            break;

        case Leds::LedPin::B:
            LedB::set(isSet);
            break;

        case Leds::LedPin::C:
            LedC::set(isSet);
            break;

        case Leds::LedPin::D:
            LedD::set(isSet);
            break;

        case Leds::LedPin::E:
            LedE::set(isSet);
            break;

        case Leds::LedPin::F:
            LedF::set(isSet);
            break;

        case Leds::LedPin::G:
            LedG::set(isSet);
            break;

        case Leds::LedPin::H:
            LedH::set(isSet);
            break;

        case Leds::LedPin::Green:
            LedGreen::set(isSet);
            break;

        case Leds::LedPin::Red:
            LedRed::set(isSet);
            break;
    }
#endif
}
}  // namespace gpio

}  // namespace tap
