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

#ifndef DIGITAL_MOCK_HPP_
#define DIGITAL_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwlib/communication/gpio/digital.hpp"

namespace aruwlib
{
namespace mock
{
class DigitalMock : public aruwlib::gpio::Digital
{
public:
    DigitalMock();
    virtual ~DigitalMock();

    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(
        void,
        configureInputPullMode,
        (aruwlib::gpio::Digital::InputPin pin, aruwlib::gpio::Digital::InputPullMode mode),
        (override));
    MOCK_METHOD(void, set, (aruwlib::gpio::Digital::OutputPin pin, bool isSet), (override));
    MOCK_METHOD(bool, read, (aruwlib::gpio::Digital::InputPin pin), (const override));
};  // class DigitalMock
}  // namespace mock
}  // namespace aruwlib

#endif  // DIGITAL_MOCK_HPP_
