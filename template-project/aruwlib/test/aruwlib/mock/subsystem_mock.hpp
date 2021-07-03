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

#ifndef SUBSYSTEM_MOCK_HPP_
#define SUBSYSTEM_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwlib/control/subsystem.hpp"

namespace aruwlib
{
namespace mock
{
class SubsystemMock : public control::Subsystem
{
public:
    SubsystemMock(Drivers *drivers);
    virtual ~SubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, setDefaultCommand, (control::Command * defaultCommand), (override));
    MOCK_METHOD(control::Command *, getDefaultCommand, (), (const override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(bool, isHardwareTestComplete, (), (const override));
    MOCK_METHOD(void, setHardwareTestsComplete, (), (override));
    MOCK_METHOD(void, setHardwareTestsIncomplete, (), (override));
    MOCK_METHOD(void, runHardwareTests, (), (override));
    MOCK_METHOD(const char *, getName, (), (override));
};  // class SubsystemMock
}  // namespace mock
}  // namespace aruwlib

#endif  // SUBSYSTEM_MOCK_HPP_
