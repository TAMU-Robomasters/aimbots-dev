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

#ifndef SCHEDULER_TERMINAL_HANDLER_MOCK_HPP_
#define SCHEDULER_TERMINAL_HANDLER_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwlib/control/scheduler_terminal_handler.hpp"

namespace aruwlib
{
namespace mock
{
class SchedulerTerminalHandlerMock : public control::SchedulerTerminalHandler
{
public:
    SchedulerTerminalHandlerMock(Drivers *drivers);
    virtual ~SchedulerTerminalHandlerMock();

    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(bool, terminalSerialCallback, (char *, modm::IOStream &, bool), (override));
    MOCK_METHOD(void, terminalSerialStreamCallback, (modm::IOStream &), (override));
};
}  // namespace mock
}  // namespace aruwlib

#endif  //  SCHEDULER_TERMINAL_HANDLER_MOCK_HPP_
