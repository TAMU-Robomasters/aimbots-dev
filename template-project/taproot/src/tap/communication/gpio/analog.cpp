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

#include "analog.hpp"

#include "tap/board/board.hpp"
#include "tap/util_macros.hpp"

using namespace Board;

namespace tap
{
namespace gpio
{
void Analog::init()
{
#ifndef PLATFORM_HOSTED
    AnalogInPins::setAnalogInput();

    // Initial ADC/Timer setup
    Adc1::connect<AnalogInPinS::In0, AnalogInPinT::In1, AnalogInPinU::In2, AnalogInPinV::In3, AnalogInPinOledJoystick::In6>();
    Adc1::initialize<SystemClock, 22500000_Bd>();

    Adc1::setPinChannel<AnalogInPinS>();
    Adc1::setPinChannel<AnalogInPinT>();
    Adc1::setPinChannel<AnalogInPinU>();
    Adc1::setPinChannel<AnalogInPinV>();
    Adc1::setPinChannel<AnalogInPinOledJoystick>();

#endif
}

uint16_t Analog::read(Pin pin) const
{
#ifdef PLATFORM_HOSTED
    UNUSED(pin);
    return 0;
#else
    switch (pin)
    {
        case Pin::S:
            return Adc1::readChannel(Adc1::getPinChannel<AnalogInPinS>());
        case Pin::T:
            return Adc1::readChannel(Adc1::getPinChannel<AnalogInPinT>());
        case Pin::U:
            return Adc1::readChannel(Adc1::getPinChannel<AnalogInPinU>());
        case Pin::V:
            return Adc1::readChannel(Adc1::getPinChannel<AnalogInPinV>());
        case Pin::OledJoystick:
            return Adc1::readChannel(Adc1::getPinChannel<AnalogInPinOledJoystick>());
        default:
            return 0;
    }
#endif
}
}  // namespace gpio

}  // namespace tap
