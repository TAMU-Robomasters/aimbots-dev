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

#ifndef REF_SERIAL_MOCK_HPP_
#define REF_SERIAL_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwlib/communication/serial/ref_serial.hpp"

namespace aruwlib
{
namespace mock
{
class RefSerialMock : public serial::RefSerial
{
public:
    RefSerialMock(Drivers* drivers);
    virtual ~RefSerialMock();

    MOCK_METHOD(
        void,
        messageReceiveCallback,
        (const aruwlib::serial::DJISerial::SerialMessage&),
        (override));
    MOCK_METHOD(bool, getRefSerialReceivingData, (), (const override));
    MOCK_METHOD(const RobotData&, getRobotData, (), (const override));
    MOCK_METHOD(const GameData&, getGameData, (), (const override));
    MOCK_METHOD(
        void,
        configGraphicGenerics,
        (GraphicData*, const uint8_t*, AddGraphicOperation, uint8_t, GraphicColor),
        (override));
    MOCK_METHOD(
        void,
        configLine,
        (uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, GraphicData*),
        (override));
    MOCK_METHOD(
        void,
        configRectangle,
        (uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, GraphicData*),
        (override));
    MOCK_METHOD(
        void,
        configCircle,
        (uint16_t, uint16_t, uint16_t, uint16_t, GraphicData*),
        (override));
    MOCK_METHOD(
        void,
        configEllipse,
        (uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, GraphicData*),
        (override));
    MOCK_METHOD(
        void,
        configArc,
        (uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, GraphicData*),
        (override));
    MOCK_METHOD(
        void,
        configFloatingNumber,
        (uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, float, GraphicData*),
        (override));
    MOCK_METHOD(
        void,
        configInteger,
        (uint16_t, uint16_t, uint16_t, uint16_t, int32_t, GraphicData*),
        (override));
    MOCK_METHOD(void, updateInteger, (int32_t, GraphicData*), (override));
    MOCK_METHOD(
        void,
        configCharacterMsg,
        (uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, const char*, GraphicCharacterMessage*),
        (override));
    MOCK_METHOD(void, deleteGraphicLayer, (DeleteGraphicOperation, uint8_t), (override));
};  // class RefSerialMock
}  // namespace mock
}  // namespace aruwlib

#endif  // REF_SERIAL_MOCK_HPP_
