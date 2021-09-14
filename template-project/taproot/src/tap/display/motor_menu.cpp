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

#include "motor_menu.hpp"

#include <algorithm>
#include <cmath>

#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/dji_motor_tx_handler.hpp"

#include "motor_specific_menu.hpp"

using namespace tap::motor;
using namespace tap::can;

namespace tap
{
namespace display
{
MotorMenu::MotorMenu(modm::ViewStack<DummyAllocator<modm::IAbstractView> >* stack, Drivers* drivers)
    : modm::AbstractMenu<DummyAllocator<modm::IAbstractView> >(stack, MOTOR_MENU_ID),
      drivers(drivers),
      verticalScroll(drivers, DjiMotorTxHandler::DJI_MOTORS_PER_CAN * 2, DISPLAY_MAX_ENTRIES)
{
}

void MotorMenu::drawMotor(CanBus canBus, int normalizedMotorId)
{
    const DjiMotor* motor = nullptr;
    const char* canBusName = "CAN?";
    bool printCursor = false;
    switch (canBus)
    {
        case CanBus::CAN_BUS1:
            motor = drivers->djiMotorTxHandler.getCan1Motor(
                NORMALIZED_ID_TO_DJI_MOTOR(normalizedMotorId));
            canBusName = "CAN1";
            printCursor = (normalizedMotorId == verticalScroll.getCursorIndex());
            break;
        case CanBus::CAN_BUS2:
            motor = drivers->djiMotorTxHandler.getCan2Motor(
                NORMALIZED_ID_TO_DJI_MOTOR(normalizedMotorId));
            canBusName = "CAN2";
            printCursor =
                ((normalizedMotorId + DjiMotorTxHandler::DJI_MOTORS_PER_CAN) ==
                 verticalScroll.getCursorIndex());
            break;
    }

    getViewStack()->getDisplay() << (printCursor ? ">" : " ") << canBusName << " Motor "
                                 << (normalizedMotorId + 1);

    if (motor != nullptr)
    {
        if (motor->isMotorOnline())
        {
            getViewStack()->getDisplay() << ": ON";
        }
        else
        {
            getViewStack()->getDisplay() << ": OFF";
        }
    }
    else
    {
        getViewStack()->getDisplay() << ": NULL";
    }
    getViewStack()->getDisplay() << modm::endl;
}

void MotorMenu::draw()
{
    modm::GraphicDisplay& display = getViewStack()->getDisplay();
    display.clear();
    display.setCursor(0, 2);
    display << getMenuName() << modm::endl;

    auto can1MinIndex = verticalScroll.getSmallestIndexDisplayed();
    auto can1MaxIndex = std::min(
        DjiMotorTxHandler::DJI_MOTORS_PER_CAN - 1,
        static_cast<int>(verticalScroll.getLargestIndexDisplayed()));
    for (int8_t motorId = can1MinIndex; motorId <= can1MaxIndex; ++motorId)
    {
        drawMotor(CanBus::CAN_BUS1, motorId);
    }

    auto can2MinIndex = std::max(
        0,
        verticalScroll.getSmallestIndexDisplayed() - DjiMotorTxHandler::DJI_MOTORS_PER_CAN);
    auto can2MaxIndex = std::min(
        DjiMotorTxHandler::DJI_MOTORS_PER_CAN - 1,
        verticalScroll.getLargestIndexDisplayed() - DjiMotorTxHandler::DJI_MOTORS_PER_CAN);
    for (int motorId = can2MinIndex; motorId <= can2MaxIndex; ++motorId)
    {
        drawMotor(CanBus::CAN_BUS2, motorId);
    }
}

void MotorMenu::update() {}

bool MotorMenu::hasChanged()
{
    uint8_t newCan1MotorStatus = 0;
    uint8_t newCan2MotorStatus = 0;
    for (int motorId = MotorId::MOTOR1; motorId <= MotorId::MOTOR8; ++motorId)
    {
        DjiMotor const* currMotor = NULL;

        currMotor = drivers->djiMotorTxHandler.getCan1Motor(static_cast<MotorId>(motorId));
        if (currMotor != nullptr && currMotor->isMotorOnline())
        {
            newCan1MotorStatus |= (1 << DJI_MOTOR_NORMALIZED_ID(motorId));
        }

        currMotor = drivers->djiMotorTxHandler.getCan2Motor(static_cast<MotorId>(motorId));
        if (currMotor != nullptr && currMotor->isMotorOnline())
        {
            newCan2MotorStatus |= (1 << DJI_MOTOR_NORMALIZED_ID(motorId));
        }
    }

    bool motorStatusChanged = (can1PrevDisplayedStatus != newCan1MotorStatus) ||
                              (can2PrevDisplayedStatus != newCan2MotorStatus);

    can1PrevDisplayedStatus = newCan1MotorStatus;
    can2PrevDisplayedStatus = newCan2MotorStatus;

    return verticalScroll.acknowledgeCursorChanged() || motorStatusChanged;
}

void MotorMenu::shortButtonPress(modm::MenuButtons::Button button)
{
    switch (button)
    {
        case modm::MenuButtons::LEFT:
            this->remove();
            break;
        case modm::MenuButtons::RIGHT:
        {
            int8_t idx = verticalScroll.getCursorIndex();
            if (idx < DjiMotorTxHandler::DJI_MOTORS_PER_CAN)  // idx between [0, 8)
            {
                const DjiMotor* motor =
                    drivers->djiMotorTxHandler.getCan1Motor(NORMALIZED_ID_TO_DJI_MOTOR(idx));
                if (motor != nullptr)
                {
                    this->getViewStack()->push(
                        new MotorSpecificMenu(getViewStack(), drivers, motor));
                }
            }
            else  // idx between [8, 16)
            {
                const DjiMotor* motor = drivers->djiMotorTxHandler.getCan2Motor(
                    NORMALIZED_ID_TO_DJI_MOTOR(idx - DjiMotorTxHandler::DJI_MOTORS_PER_CAN));
                if (motor != nullptr)
                {
                    this->getViewStack()->push(
                        new MotorSpecificMenu(getViewStack(), drivers, motor));
                }
            }
            break;
        }
        case modm::MenuButtons::DOWN:
            verticalScroll.onShortButtonPress(modm::MenuButtons::DOWN);
            break;
        case modm::MenuButtons::UP:
            verticalScroll.onShortButtonPress(modm::MenuButtons::UP);
            break;
        case modm::MenuButtons::OK:
            break;
    }
}
}  // namespace display
}  // namespace tap
