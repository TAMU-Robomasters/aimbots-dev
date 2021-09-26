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

#include "oled_button_handler.hpp"

#include "tap/drivers.hpp"

namespace tap
{
namespace display
{
OledButtonHandler::OledButtonHandler(tap::Drivers *drivers)
    : drivers(drivers),
      downButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      upButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      leftButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      rightButtonPressed(BUTTON_DEBOUNCE_SAMPLES),
      okButtonPressed(BUTTON_DEBOUNCE_SAMPLES)
{
}

OledButtonHandler::Button OledButtonHandler::getCurrentButtonState()
{
    int buttonADC = drivers->analog.read(gpio::Analog::Pin::OledJoystick);

    downButtonPressed.update(abs(buttonADC - DOWN_ADC_VAL) < ADC_PRESSED_RANGE);
    upButtonPressed.update(abs(buttonADC - UP_ADC_VAL) < ADC_PRESSED_RANGE);
    leftButtonPressed.update(abs(buttonADC - LEFT_ADC_VAL) < ADC_PRESSED_RANGE);
    rightButtonPressed.update(abs(buttonADC - RIGHT_ADC_VAL) < ADC_PRESSED_RANGE);
    okButtonPressed.update(abs(buttonADC - OK_ADC_VAL) < ADC_PRESSED_RANGE);

    if (downButtonPressed.getValue())
    {
        return Button::DOWN;
    }
    else if (upButtonPressed.getValue())
    {
        return Button::UP;
    }
    else if (rightButtonPressed.getValue())
    {
        return Button::RIGHT;
    }
    else if (leftButtonPressed.getValue())
    {
        return Button::LEFT;
    }
    else if (okButtonPressed.getValue())
    {
        return Button::OK;
    }
    else
    {
        return Button::NONE;
    }
}
}  // namespace display
}  // namespace tap
