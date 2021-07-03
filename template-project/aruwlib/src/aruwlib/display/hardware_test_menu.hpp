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

#ifndef HARDWARE_TEST_MENU_HPP_
#define HARDWARE_TEST_MENU_HPP_

#include "aruwlib/control/command_scheduler_types.hpp"

#include "modm/ui/menu/abstract_menu.hpp"

#include "vertical_scroll_logic_handler.hpp"

namespace aruwlib
{
class Drivers;

namespace display
{
class HardwareTestMenu : public modm::AbstractMenu
{
public:
    HardwareTestMenu(modm::ViewStack *vs, Drivers *drivers);

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Hardware Test Menu"; }

private:
    static constexpr int HARDWARE_TEST_MENU_ID = 4;
    static constexpr int MAX_ENTRIES_DISPLAYED = 6;

    Drivers *drivers;

    control::subsystem_scheduler_bitmap_t completeSubsystems = 0;

    VerticalScrollLogicHandler vertScrollHandler;
};  // class HardwareTestMenu
}  // namespace display
}  // namespace aruwlib

#endif  // HARDWARE_TEST_MENU_HPP_
