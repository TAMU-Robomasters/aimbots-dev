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

#include "command_scheduler_menu.hpp"

#include <algorithm>

#include "tap/control/command.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

namespace tap
{
namespace display
{
CommandSchedulerMenu::CommandSchedulerMenu(
    modm::ViewStack<DummyAllocator<modm::IAbstractView> > *stack,
    Drivers *drivers)
    : modm::AbstractMenu<DummyAllocator<modm::IAbstractView> >(stack, 1),
      drivers(drivers),
      vertScrollHandler(drivers, 0, MAX_ENTRIES_DISPLAYED),
      firstDrawTime(true)
{
}

void CommandSchedulerMenu::update() {}

void CommandSchedulerMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
    else
    {
        vertScrollHandler.onShortButtonPress(button);
    }
}

bool CommandSchedulerMenu::hasChanged()
{
    if (firstDrawTime)
    {
        firstDrawTime = false;
        return true;
    }

    bool schedulerChanged =
        (prevRegisteredSubsystems != drivers->commandScheduler.getRegisteredSubsystemBitmap()) ||
        (prevAddedCommands != drivers->commandScheduler.getAddedCommandBitmap());
    prevRegisteredSubsystems = drivers->commandScheduler.getRegisteredSubsystemBitmap();
    prevAddedCommands = drivers->commandScheduler.getAddedCommandBitmap();

    return vertScrollHandler.acknowledgeCursorChanged() || schedulerChanged;
}

void CommandSchedulerMenu::draw()
{
    int numSubsystems = drivers->commandScheduler.subsystemListSize();
    int numCommands = drivers->commandScheduler.commandListSize();
    if (numSubsystems + numCommands != vertScrollHandler.getSize())
    {
        // Must check and update size every time in case commands have been added/removed
        vertScrollHandler.setSize(numSubsystems + numCommands);
    }

    modm::GraphicDisplay &display = getViewStack()->getDisplay();

    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    // TODO in the future display relationship between subsystems/commands (due to time constraints
    // this is not done)
    int8_t index = 0;

    // Only print "Subsystems:" when there are subsystems within the valid screen range to print
    if (numSubsystems > vertScrollHandler.getSmallestIndexDisplayed())
    {
        display << "Subsystems:" << modm::endl;
    }

    std::for_each(
        drivers->commandScheduler.subMapBegin(),
        drivers->commandScheduler.subMapEnd(),
        [&](control::Subsystem *sub) {
            if (index >= vertScrollHandler.getSmallestIndexDisplayed() &&
                index <= vertScrollHandler.getLargestIndexDisplayed())
            {
                display << (index == vertScrollHandler.getCursorIndex() ? ">" : " ")
                        << sub->getName() << modm::endl;
            }
            index++;
        });

    if (vertScrollHandler.getLargestIndexDisplayed() >= numSubsystems)
    {
        display << "Commands:" << modm::endl;
    }
    std::for_each(
        drivers->commandScheduler.cmdMapBegin(),
        drivers->commandScheduler.cmdMapEnd(),
        [&](control::Command *cmd) {
            // Only display stuff within bounds of the scroll handler.
            if (index >= vertScrollHandler.getSmallestIndexDisplayed() &&
                index <= vertScrollHandler.getLargestIndexDisplayed())
            {
                display << (index == vertScrollHandler.getCursorIndex() ? ">" : " ")
                        << cmd->getName() << modm::endl;
            }
            index++;
        });
}
}  // namespace display
}  // namespace tap
