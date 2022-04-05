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

#ifndef TAPROOT_REF_SERIAL_MENU_HPP_
#define TAPROOT_REF_SERIAL_MENU_HPP_

#include "tap/architecture/periodic_timer.hpp"

#include "modm/ui/menu/abstract_menu.hpp"

#include "dummy_allocator.hpp"
#include "vertical_scroll_logic_handler.hpp"

namespace tap
{
class Drivers;
}

namespace tap::display
{
/**
 * A menu that displays various information about the referee system. The information is designed to
 * mirror the referee system's MCM's OLED, which will allow you to diagnose if the refereee system
 * is working properly.
 *
 * Current information drawn:
 * - Robot Type/ID
 * - HP
 * - 17mmSpeed
 * - 17mmHeat
 * - 42mmSpeed
 * - 42mmHeat
 * - PowerBuf
 * - Power
 */
class RefSerialMenu : public modm::AbstractMenu<DummyAllocator<modm::IAbstractView> >
{
public:
    /// Time between calls to `draw`, which will redraw the referee serial menu.
    static constexpr uint32_t DISPLAY_DRAW_PERIOD = 500;
    static constexpr int REF_SERIAL_INFO_LINES = 8;
    static constexpr int DISPLAY_MAX_ENTRIES = 8;

    RefSerialMenu(modm::ViewStack<DummyAllocator<modm::IAbstractView> > *stack, Drivers *drivers);

    void draw() override;

    void shortButtonPress(modm::MenuButtons::Button button) override;

    void update() override;

    bool hasChanged() override;

    static const char *getMenuName() { return "Ref Serial Menu"; }

private:
    using PrintRefSerialDataFncPtr = void (RefSerialMenu::*)(modm::IOStream &);

    Drivers *drivers;
    VerticalScrollLogicHandler verticalScroll;

    PrintRefSerialDataFncPtr printRefSerialDataFncPtrs[REF_SERIAL_INFO_LINES];

    arch::PeriodicMilliTimer updatePeriodicTimer{DISPLAY_DRAW_PERIOD};

    void printRobotTypeId(modm::IOStream &stream);
    void printHp(modm::IOStream &stream);
    void print17mmSpeed(modm::IOStream &stream);
    void print17mmHeat(modm::IOStream &stream);
    void print42mmSpeed(modm::IOStream &stream);
    void print42mmHeat(modm::IOStream &stream);
    void printPowerBuf(modm::IOStream &stream);
    void printPower(modm::IOStream &stream);
};
}  // namespace tap::display

#endif  // TAPROOT_REF_SERIAL_MENU_HPP_
