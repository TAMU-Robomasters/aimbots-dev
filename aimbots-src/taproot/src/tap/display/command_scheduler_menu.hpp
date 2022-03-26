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

#ifndef TAPROOT_COMMAND_SCHEDULER_MENU_HPP_
#define TAPROOT_COMMAND_SCHEDULER_MENU_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/control/command_scheduler_types.hpp"

#include "modm/ui/menu/abstract_menu.hpp"

#include "dummy_allocator.hpp"
#include "vertical_scroll_logic_handler.hpp"

namespace tap
{
class Drivers;

namespace control
{
class Command;
class Subsystem;
}  // namespace control

namespace display
{
class CommandSchedulerMenu : public modm::AbstractMenu<DummyAllocator<modm::IAbstractView>>
{
public:
    CommandSchedulerMenu(
        modm::ViewStack<DummyAllocator<modm::IAbstractView>> *stack,
        Drivers *drivers);

    ~CommandSchedulerMenu() = default;

    void draw() override;

    void update() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Command Scheduler"; }

private:
    static constexpr int MAX_ENTRIES_DISPLAYED = 5;

    Drivers *drivers;
    VerticalScrollLogicHandler vertScrollHandler;
    bool firstDrawTime;
    control::subsystem_scheduler_bitmap_t prevRegisteredSubsystems = 0;
    control::command_scheduler_bitmap_t prevAddedCommands = 0;
};  // class CommandSchedulerMenu
}  // namespace display
}  // namespace tap

#endif  // TAPROOT_COMMAND_SCHEDULER_MENU_HPP_
