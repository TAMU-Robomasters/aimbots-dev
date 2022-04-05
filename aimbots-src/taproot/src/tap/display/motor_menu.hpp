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

#ifndef TAPROOT_MOTOR_MENU_HPP_
#define TAPROOT_MOTOR_MENU_HPP_

#include "tap/communication/can/can_bus.hpp"

#include "modm/ui/menu/abstract_menu.hpp"

#include "dummy_allocator.hpp"
#include "vertical_scroll_logic_handler.hpp"

namespace tap
{
namespace motor
{
class DjiMotor;
}
class Drivers;

namespace display
{
class MotorMenu : public modm::AbstractMenu<DummyAllocator<modm::IAbstractView> >
{
public:
    MotorMenu(modm::ViewStack<DummyAllocator<modm::IAbstractView> > *stack, Drivers *drivers);

    virtual ~MotorMenu() = default;

    void draw() override;

    void update() override;

    bool hasChanged() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    static const char *getMenuName() { return "Motor Menu"; }

private:
    static constexpr int MOTOR_MENU_ID = 5;
    static constexpr int DISPLAY_MAX_ENTRIES = 7;

    Drivers *drivers;

    VerticalScrollLogicHandler verticalScroll;

    uint8_t can1PrevDisplayedStatus;
    uint8_t can2PrevDisplayedStatus;

    void drawMotor(tap::can::CanBus canBus, int normalizedMotorId);
};
}  // namespace display
}  // namespace tap

#endif  // TAPROOT_MOTOR_MENU_HPP_
