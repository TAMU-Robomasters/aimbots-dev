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

#ifndef CONTROL_OPERATOR_INTERFACE_MOCK_HPP_
#define CONTROL_OPERATOR_INTERFACE_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwlib/control/command.hpp"
#include "aruwlib/control/control_operator_interface.hpp"
#include "aruwlib/control/subsystem.hpp"

namespace aruwlib
{
namespace mock
{
class ControlOperatorInterfaceMock : public aruwlib::control::ControlOperatorInterface
{
public:
    ControlOperatorInterfaceMock(aruwlib::Drivers *drivers);
    virtual ~ControlOperatorInterfaceMock();

    MOCK_METHOD(float, getChassisXInput, (), (override));
    MOCK_METHOD(float, getChassisYInput, (), (override));
    MOCK_METHOD(float, getChassisRInput, (), (override));
    MOCK_METHOD(float, getTurretYawInput, (), (override));
    MOCK_METHOD(float, getTurretPitchInput, (), (override));
    MOCK_METHOD(float, getSentinelSpeedInput, (), (override));
};  // class ControlOperatorInterfaceMock
}  // namespace mock
}  // namespace aruwlib

#endif  // CONTROL_OPERATOR_INTERFACE_MOCK_HPP_
