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

#ifndef MOTOR_MENU_HPP_
#define MOTOR_MENU_HPP_

#include "aruwlib/communication/can/can_bus.hpp"

#include "modm/ui/menu/abstract_menu.hpp"

#include "vertical_scroll_logic_handler.hpp"

namespace aruwlib
{
namespace motor
{
class DjiMotor;
}
class Drivers;

namespace display
{
class MotorMenu : public modm::AbstractMenu
{
public:
    MotorMenu(modm::ViewStack *stack, Drivers *drivers);

    virtual ~MotorMenu() = default;

    void draw() override;

    void update() override;

    bool hasChanged() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    static const char *getMenuName() { return "Motor Menu"; }

private:
    static constexpr int MOTOR_MENU_ID = 5;
    static constexpr int DISPLAY_MAX_ENTRIES = 7;

    Drivers *drivers;

    VerticalScrollLogicHandler verticalScroll;

    uint8_t can1PrevDisplayedStatus;
    uint8_t can2PrevDisplayedStatus;

    void drawMotor(aruwlib::can::CanBus canBus, int normalizedMotorId);
};
}  // namespace display
}  // namespace aruwlib
#endif  // MOTOR_MENU_HPP_
