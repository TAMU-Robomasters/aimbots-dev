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

#ifndef TAPROOT_BMI088_MOCK_HPP_
#define TAPROOT_BMI088_MOCK_HPP_

#include <gmock/gmock.h>

#include "tap/communication/sensors/imu/bmi088/bmi088.hpp"

namespace tap::mock
{
class Bmi088Mock : public tap::communication::sensors::imu::bmi088::Bmi088
{
public:
    Bmi088Mock(tap::Drivers *drivers);
    virtual ~Bmi088Mock();

    MOCK_METHOD(void, initialize, (float, float, float), (override));
    MOCK_METHOD(void, periodicIMUUpdate, (), (override));
    MOCK_METHOD(ImuState, getImuState, (), (const override));
    MOCK_METHOD(void, requestRecalibration, (), (override));
    MOCK_METHOD(float, getYaw, (), (override));
    MOCK_METHOD(float, getPitch, (), (override));
    MOCK_METHOD(float, getRoll, (), (override));
    MOCK_METHOD(float, getGx, (), (override));
    MOCK_METHOD(float, getGy, (), (override));
    MOCK_METHOD(float, getGz, (), (override));
    MOCK_METHOD(float, getAx, (), (override));
    MOCK_METHOD(float, getAy, (), (override));
    MOCK_METHOD(float, getAz, (), (override));
    MOCK_METHOD(float, getTemp, (), (override));
    MOCK_METHOD(uint32_t, getPrevIMUDataReceivedTime, (), (const override));
};
}  // namespace tap::mock

#endif  // TAPROOT_BMI088_MOCK_HPP_
