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

#include "motor_specific_menu.hpp"

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/dji_motor_tx_handler.hpp"

using namespace tap::motor;

namespace tap
{
namespace display
{
MotorSpecificMenu::MotorSpecificMenu(
    modm::ViewStack<DummyAllocator<modm::IAbstractView> > *stack,
    Drivers *drivers,
    const DjiMotor *motor)
    : modm::AbstractMenu<DummyAllocator<modm::IAbstractView> >(stack, 1),
      drivers(drivers),
      associatedMotor(motor)
{
}

void MotorSpecificMenu::update() {}

void MotorSpecificMenu::draw()
{
    if (associatedMotor == nullptr)
    {
        RAISE_ERROR(drivers, "MotorSpecificMenu has nullptr associated motor");
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
}  // namespace tap
