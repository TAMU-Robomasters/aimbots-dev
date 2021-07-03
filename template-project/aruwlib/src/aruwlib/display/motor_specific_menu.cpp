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

#include "motor_specific_menu.hpp"

#include "aruwlib/drivers.hpp"
#include "aruwlib/errors/create_errors.hpp"
#include "aruwlib/motor/dji_motor.hpp"
#include "aruwlib/motor/dji_motor_tx_handler.hpp"

using namespace aruwlib::motor;

namespace aruwlib
{
namespace display
{
MotorSpecificMenu::MotorSpecificMenu(
    modm::ViewStack *stack,
    Drivers *drivers,
    const DjiMotor *motor)
    : modm::AbstractMenu(stack, 1),
      drivers(drivers),
      associatedMotor(motor)
{
}

void MotorSpecificMenu::update() {}

void MotorSpecificMenu::draw()
{
    if (associatedMotor == nullptr)
    {
        RAISE_ERROR(
            drivers,
            "MotorSpecificMenu has nullptr associated motor",
            errors::OLED_DISPLAY,
            errors::OLEDErrors::NULLPTR_DJI_MOTOR_IN_MOTOR_SPECIFIC_MENU);
        return;
    }

    modm::GraphicDisplay &display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << associatedMotor->getName() << modm::endl << modm::endl;

    currDesiredOutput = associatedMotor->getOutputDesired();
    currIsInverted = associatedMotor->isMotorInverted();
    currEncoderWrapped = associatedMotor->getEncoderWrapped();
    currRPM = associatedMotor->getShaftRPM();

    display << "  Motor ID: " << associatedMotor->getMotorIdentifier() << modm::endl
            << "  Des. Output: " << currDesiredOutput << modm::endl
            << "  Enc. Wrapped: " << currEncoderWrapped << modm::endl
            << "  RPM: " << currRPM << modm::endl
            << "  Inverted: " << currIsInverted;
}

void MotorSpecificMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    if (button == modm::MenuButtons::LEFT)
    {
        this->remove();
    }
}

bool MotorSpecificMenu::hasChanged()
{
    bool sameOutputDesired = (associatedMotor->getOutputDesired() == currDesiredOutput);
    bool sameInverted = (associatedMotor->isMotorInverted() == currIsInverted);
    bool sameEncoderWrapped = (associatedMotor->getEncoderWrapped() == currEncoderWrapped);

    return !(sameOutputDesired && sameInverted && sameEncoderWrapped);
}
}  // namespace display
}  // namespace aruwlib
