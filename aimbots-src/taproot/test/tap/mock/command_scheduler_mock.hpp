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

#ifndef TAPROOT_COMMAND_SCHEDULER_MOCK_HPP_
#define TAPROOT_COMMAND_SCHEDULER_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/control/command.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/subsystem.hpp"

namespace tap
{
namespace mock
{
class CommandSchedulerMock : public control::CommandScheduler
{
public:
    CommandSchedulerMock(Drivers *drivers);
    virtual ~CommandSchedulerMock();

    MOCK_METHOD(void, run, (), (override));
    MOCK_METHOD(void, addCommand, (control::Command *), (override));
    MOCK_METHOD(void, removeCommand, (control::Command *, bool), (override));
    MOCK_METHOD(bool, isCommandScheduled, (const control::Command *), (const override));
    MOCK_METHOD(void, registerSubsystem, (control::Subsystem *), (override));
    MOCK_METHOD(bool, isSubsystemRegistered, (const control::Subsystem *), (const override));
    MOCK_METHOD(void, startHardwareTests, (), (override));
    MOCK_METHOD(void, stopHardwareTests, (), (override));
    MOCK_METHOD(int, subsystemListSize, (), (const override));
    MOCK_METHOD(int, commandListSize, (), (const override));
    MOCK_METHOD(CommandIterator, cmdMapBegin, (), (override));
    MOCK_METHOD(CommandIterator, cmdMapEnd, (), (override));
    MOCK_METHOD(SubsystemIterator, subMapBegin, (), (override));
    MOCK_METHOD(SubsystemIterator, subMapEnd, (), (override));
    MOCK_METHOD(
        control::subsystem_scheduler_bitmap_t,
        getRegisteredSubsystemBitmap,
        (),
        (const override));
    MOCK_METHOD(control::command_scheduler_bitmap_t, getAddedCommandBitmap, (), (const override));
};  // class CommandSchedulerMock
}  // namespace mock
}  // namespace tap

#endif  // TAPROOT_COMMAND_SCHEDULER_MOCK_HPP_
