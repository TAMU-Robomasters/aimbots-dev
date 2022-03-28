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

#ifndef TAPROOT_OLED_BUTTON_HANDLER_HPP_
#define TAPROOT_OLED_BUTTON_HANDLER_HPP_

#include "modm/math/filter/debounce.hpp"
#include "modm/ui/menu/view_stack.hpp"

namespace tap
{
class Drivers;
namespace display
{
/**
 * Class designed to convert analog button signal from the OLED
 * display into usable button states.
 */
class OledButtonHandler
{
public:
    enum Button
    {
        LEFT = modm::MenuButtons::LEFT,
        RIGHT = modm::MenuButtons::RIGHT,
        UP = modm::MenuButtons::UP,
        DOWN = modm::MenuButtons::DOWN,
        OK = modm::MenuButtons::OK,
        NONE,
    };

    OledButtonHandler(tap::Drivers *drivers);

    /**
     * Updates the status of the current button and returns the updated button.
     *
     * @return The current state of the button, which corresponds to a
     *      `modm::MenuButton::Button` state, or `modm::MenuButtons::NONE` if no button
     *      is pressed.
     */
    Button getCurrentButtonState();

private:
    static constexpr int BUTTON_DEBOUNCE_SAMPLES = 10;
    static constexpr int ADC_PRESSED_RANGE = 100;
    static constexpr int OK_ADC_VAL = 0;
    static constexpr int LEFT_ADC_VAL = 900;
    static constexpr int RIGHT_ADC_VAL = 1700;
    static constexpr int UP_ADC_VAL = 2500;
    static constexpr int DOWN_ADC_VAL = 3300;

    tap::Drivers *drivers;

    modm::filter::Debounce<int> downButtonPressed;
    modm::filter::Debounce<int> upButtonPressed;
    modm::filter::Debounce<int> leftButtonPressed;
    modm::filter::Debounce<int> rightButtonPressed;
    modm::filter::Debounce<int> okButtonPressed;
};  // class OledButtonHandler
}  // namespace display
}  // namespace tap

#endif  // TAPROOT_BUTTON_HANDLER_HPP_
