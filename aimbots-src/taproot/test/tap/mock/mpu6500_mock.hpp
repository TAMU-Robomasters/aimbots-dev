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

#ifndef MPU6500_MOCK_HPP_
#define MPU6500_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/sensors/mpu6500/mpu6500.hpp"

namespace tap
{
namespace mock
{
class Mpu6500Mock : public tap::sensors::Mpu6500
{
public:
    Mpu6500Mock(tap::Drivers *drivers);
    virtual ~Mpu6500Mock();

    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(void, periodicIMUUpdate, (), (override));
    MOCK_METHOD(bool, read, (), (override));
    MOCK_METHOD(bool, initialized, (), (const override));
    MOCK_METHOD(float, getAx, (), (const override));
    MOCK_METHOD(float, getAy, (), (const override));
    MOCK_METHOD(float, getAz, (), (const override));
    MOCK_METHOD(float, getGx, (), (const override));
    MOCK_METHOD(float, getGy, (), (const override));
    MOCK_METHOD(float, getGz, (), (const override));
    MOCK_METHOD(float, getTemp, (), (const override));
    MOCK_METHOD(float, getYaw, (), (override));
    MOCK_METHOD(float, getPitch, (), (override));
    MOCK_METHOD(float, getRoll, (), (override));
    MOCK_METHOD(float, getTiltAngle, (), (override));
};  // Mpu6500Mock
}  // namespace mock
}  // namespace tap

#endif  //  MPU6500_MOCK_HPP_
