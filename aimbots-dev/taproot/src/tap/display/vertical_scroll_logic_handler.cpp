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

#include "vertical_scroll_logic_handler.hpp"

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

namespace tap
{
namespace display
{
VerticalScrollLogicHandler::VerticalScrollLogicHandler(
    Drivers *drivers,
    int8_t size,
    int8_t maxEntries)
    : drivers(drivers),
      maxEntries(maxEntries <= 0 ? 1 : maxEntries),
      size(size),
      cursorIndex(0),
      smallestIndexDisplayed(0),
      cursorChanged(false)
{
    // Max entries won't be changed so we can check for validity
    // here.
    if (maxEntries <= 0)
    {
        RAISE_ERROR(drivers, "maxEntries < 0");
    }
}

void VerticalScrollLogicHandler::onShortButtonPress(modm::MenuButtons::Button button)
{
    if (size < 0)
    {
        RAISE_ERROR(drivers, "size < 0");
        return;
    }
    else if (size == 0 || maxEntries == 0)
    {
        // These are cases where you don't have to do anything but they aren't considered errors
        return;
    }

    if (button == modm::MenuButtons::DOWN)  // move cursor down
    {
        if (cursorIndex < size - 1)  // only move when not above specified size
        {
            // shift display indices, the window of indices that may be displayed at one time
            setCursor(cursorIndex + 1);
            if (cursorIndex > getLargestIndexDisplayed())
            {
                smallestIndexDisplayed++;
            }
        }
    }
    else if (button == modm::MenuButtons::UP)
    {
        if (cursorIndex > 0)
        {
            setCursor(cursorIndex - 1);
            if (cursorIndex < smallestIndexDisplayed)
            {
                smallestIndexDisplayed--;
            }
        }
    }
}

void VerticalScrollLogicHandler::setSize(int8_t size)
{
    if (size < 0)
    {
        return;
    }

    // Check if the cursor and smallestIndexDisplayed must be updated
    if (cursorIndex >= size)
    {
        // Cursor is always going to point to last element of newly specified size
        // if it previously pointed past the new size
        setCursor(size - 1);
    }

    if (getLargestIndexDisplayed() >= size)
    {
        // the smallestIndexDisplayed must be updated so the window represents a valid
        // range of indices. Either the smallestIndexDisplayed is 0 or is size - maxEntries.
        smallestIndexDisplayed = std::max(size - maxEntries, 0);
    }

    this->size = size;
}

bool VerticalScrollLogicHandler::acknowledgeCursorChanged()
{
    if (cursorChanged)
    {
        cursorChanged = false;
        return true;
    }
    return false;
}
}  // namespace display
}  // namespace tap
