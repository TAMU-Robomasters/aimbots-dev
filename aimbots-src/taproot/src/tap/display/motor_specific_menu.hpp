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

#ifndef TAPROOT_MOTOR_SPECIFIC_MENU_HPP_
#define TAPROOT_MOTOR_SPECIFIC_MENU_HPP_

#include "modm/ui/menu/abstract_menu.hpp"

#include "dummy_allocator.hpp"

namespace tap
{
namespace motor
{
class DjiMotor;
}
class Drivers;
namespace display
{
class MotorSpecificMenu : public modm::AbstractMenu<DummyAllocator<modm::IAbstractView> >
{
public:
    MotorSpecificMenu(
        modm::ViewStack<DummyAllocator<modm::IAbstractView> >* stack,
        Drivers* drivers,
        const motor::DjiMotor* motor);

    void draw() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    void update() override;

    bool hasChanged() override;

private:
    Drivers* drivers;
    const tap::motor::DjiMotor* associatedMotor;

    int16_t currDesiredOutput = 0;
    bool currIsInverted = false;
    uint16_t currEncoderWrapped;
    int16_t currRPM = 0;
};
}  // namespace display
}  // namespace tap

#endif  // TAPROOT_MOTOR_SPECIFIC_MENU_HPP_
