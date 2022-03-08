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

#include "digital.hpp"

#include "tap/board/board.hpp"
#include "tap/util_macros.hpp"

using namespace Board;

namespace tap
{
namespace gpio
{
void Digital::init()
{
#ifndef PLATFORM_HOSTED
    // init digital out pins
    DigitalOutPins::setOutput(modm::Gpio::Low);
    // init digital in pins
    // interrupts disabled
    DigitalInPins::setInput();
#endif
}

void Digital::configureInputPullMode(Digital::InputPin pin, Digital::InputPullMode mode)
{
#ifdef PLATFORM_HOSTED
    UNUSED(pin);
    UNUSED(mode);
#else
    switch (pin)
    {
        case Digital::InputPin::A:
            DigitalInPinA::configure(mode);
            break;
        case Digital::InputPin::B:
            DigitalInPinB::configure(mode);
            break;
        case Digital::InputPin::C:
            DigitalInPinC::configure(mode);
            break;
        case Digital::InputPin::D:
            DigitalInPinD::configure(mode);
            break;
    }
#endif
}

void Digital::set(Digital::OutputPin pin, bool isSet)
{
#ifdef PLATFORM_HOSTED
    UNUSED(pin);
    UNUSED(isSet);
#else
    switch (pin)
    {
        case Digital::OutputPin::E:
            DigitalOutPinE::set(isSet);
            break;
        case Digital::OutputPin::F:
            DigitalOutPinF::set(isSet);
            break;
        case Digital::OutputPin::G:
            DigitalOutPinG::set(isSet);
            break;
        case Digital::OutputPin::H:
            DigitalOutPinH::set(isSet);
            break;
        case Digital::OutputPin::Laser:
            DigitalOutPinLaser::set(isSet);
            break;
    }
#endif
}

bool Digital::read(Digital::InputPin pin) const
{
#ifdef PLATFORM_HOSTED
    UNUSED(pin);
    return false;
#else
    switch (pin)
    {
        case Digital::InputPin::A:
            return DigitalInPinA::read();
        case Digital::InputPin::B:
            return DigitalInPinB::read();
        case Digital::InputPin::C:
            return DigitalInPinC::read();
        case Digital::InputPin::D:
            return DigitalInPinD::read();
        default:
            return false;
    }
#endif
}
}  // namespace gpio

}  // namespace tap
