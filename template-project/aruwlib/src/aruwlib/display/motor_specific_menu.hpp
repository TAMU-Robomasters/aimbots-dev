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

#ifndef MOTOR_SPECIFIC_MENU_HPP_
#define MOTOR_SPECIFIC_MENU_HPP_

#include "modm/ui/menu/abstract_menu.hpp"

namespace aruwlib
{
namespace motor
{
class DjiMotor;
}
class Drivers;
namespace display
{
class MotorSpecificMenu : public modm::AbstractMenu
{
public:
    MotorSpecificMenu(
        modm::ViewStack* stack,
        Drivers* drivers,
        const aruwlib::motor::DjiMotor* motor);

    void draw() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    void update() override;

    bool hasChanged() override;

private:
    Drivers* drivers;
    const aruwlib::motor::DjiMotor* associatedMotor;

    int16_t currDesiredOutput = 0;
    bool currIsInverted = false;
    uint16_t currEncoderWrapped;
    int16_t currRPM = 0;
};
}  // namespace display
}  // namespace aruwlib

#endif  // MOTOR_SPECIFIC_MENU_HPP_
