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

#ifndef ERROR_CONTROLLER_MOCK_HPP_
#define ERROR_CONTROLLER_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwlib/errors/error_controller.hpp"

namespace aruwlib
{
namespace mock
{
class ErrorControllerMock : public aruwlib::errors::ErrorController
{
public:
    ErrorControllerMock(aruwlib::Drivers* drivers);
    virtual ~ErrorControllerMock();

    MOCK_METHOD(void, addToErrorList, (const aruwlib::errors::SystemError& error), (override));
    MOCK_METHOD(void, updateLedDisplay, (), (override));
};  // class ErrorControllerMock
}  // namespace mock
}  // namespace aruwlib

#endif  // ERROR_CONTROLLER_MOCK_HPP_
