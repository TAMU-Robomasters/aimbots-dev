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

#ifndef TAPROOT_VERTICAL_SCROLL_LOGIC_HANDLER_HPP_
#define TAPROOT_VERTICAL_SCROLL_LOGIC_HANDLER_HPP_

#include <inttypes.h>

#include "modm/ui/menu/menu_buttons.hpp"

namespace tap
{
class Drivers;
namespace display
{
/**
 * Encapsulates the logic necessary for determining the location of the cursor
 * and the smallest and largset indices to be displayed on the OLED display.
 *
 * For example, suppose we have a menu with 10 entries but only 5 fit on the
 * screen at one time. Initially, the user's cursor starts at index 0 and indices
 * 0 and 4 are the min and max indices displayed by the OLED display. When the user
 * presses the down button a few times such that the cursor index is 5, the min and
 * max indices must be shifted by 1 so they are now 1 and 5. This class handles
 * this sort of logic for you. Call this function in a menu's `shortButtonPress`
 * function and use the known `cursorIndex`, `smallestIndexDisplayed`, and
 * `largestIndexDisplayed` when drawing.
 *
 * @note Zero indexing is used.
 */
class VerticalScrollLogicHandler
{
public:
    /**
     * @param[in] size The number of elements that the scroll logic handler will enumerate
     *      through. If less than 0, an error will be raised and this object will not operate
     *      properly. If 0, no errors will be raised but the object will not do anything.
     * @param[in] maxEntries The maximum number of entries you can display on the OLED at one time.
     *      Must be greater than 0. If <= 0, error is raised and maxEntries set to 1.
     */
    VerticalScrollLogicHandler(Drivers *drivers, int8_t size, int8_t maxEntries);

    void setSize(int8_t size);

    void onShortButtonPress(modm::MenuButtons::Button button);

    bool acknowledgeCursorChanged();

    inline int8_t getCursorIndex() const { return cursorIndex; }

    inline int8_t getSmallestIndexDisplayed() const { return smallestIndexDisplayed; }

    inline int8_t getLargestIndexDisplayed() const
    {
        return smallestIndexDisplayed + maxEntries - 1;
    }

    inline int8_t getSize() const { return size; }

private:
    Drivers *drivers;

    /**
     * The max number of displayable entries on the menu.
     */
    const int8_t maxEntries;

    /**
     * The size of the menu that you are indexing through.
     */
    int8_t size;

    /**
     * The 0-based index that the user's cursor is located at.
     */
    int8_t cursorIndex;

    /**
     * The smallest index being displayed on the menu
     */
    int8_t smallestIndexDisplayed;

    /**
     * Flag set by the `onShortButtonPress` function to indicate whether or not the cursor
     * has changed positions
     */
    bool cursorChanged;

    inline void setCursor(int8_t newCursor)
    {
        cursorIndex = newCursor;
        cursorChanged = true;
    }
};  // class VerticalScrollLogicHandler
}  // namespace display
}  // namespace tap

#endif  // TAPROOT_VERTICAL_SCROLL_LOGIC_HANDLER_HPP_
