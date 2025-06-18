/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_BMI088_HAL_HPP_
#define TAPROOT_BMI088_HAL_HPP_

#if defined(ENV_UNIT_TESTS)
#include <deque>
#endif

#include "tap/board/board.hpp"
#include "tap/util_macros.hpp"

#include "bmi088_data.hpp"

namespace tap::communication::sensors::imu::bmi088
{
class Bmi088Hal
{
private:
#if defined(ENV_UNIT_TESTS)
    /// Data that is set by to the input to bmi088ReadWriteByte
    static std::deque<uint8_t> rxData;
#endif

    static inline void chipSelectAccelLow()
    {
#if !defined(PLATFORM_HOSTED)
        Board::ImuCS1Accel::setOutput(modm::GpioOutput::Low);
#endif
    }

    static inline void chipSelectAccelHigh()
    {
#if !defined(PLATFORM_HOSTED)
        Board::ImuCS1Accel::setOutput(modm::GpioOutput::High);
#endif
    }

    static inline void chipSelectGyroLow()
    {
#if !defined(PLATFORM_HOSTED)
        Board::ImuCS1Gyro::setOutput(modm::GpioOutput::Low);
#endif
    }

    static inline void chipSelectGyroHigh()
    {
#if !defined(PLATFORM_HOSTED)
        Board::ImuCS1Gyro::setOutput(modm::GpioOutput::High);
#endif
    }

    static inline uint8_t bmi088ReadWriteByte(uint8_t tx)
    {
#if !defined(PLATFORM_HOSTED)
        uint8_t rx = 0;
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        return rx;
#elif defined(ENV_UNIT_TESTS)
        UNUSED(tx);
        if (rxData.size() != 0)
        {
            uint8_t data = rxData.front();
            rxData.pop_front();
            return data;
        }
        else
        {
            return 255;
        }
#else
        UNUSED(tx);
        return 255;
#endif
    }

    /**
     * Primitive for writing some data to a register reg to the bmi088
     */
    static inline void bmi088WriteSingleReg(uint8_t reg, uint8_t data)
    {
        bmi088ReadWriteByte(reg & ~Bmi088Data::BMI088_READ_BIT);
        bmi088ReadWriteByte(data);
    }

    /**
     * Primitive for reading a single register's value from the bmi088
     */
    static inline uint8_t bmi088ReadSingleReg(uint8_t reg)
    {
        bmi088ReadWriteByte(reg | Bmi088Data::BMI088_READ_BIT);
        return bmi088ReadWriteByte(0x55);
    }

    static inline void bmi088ReadMultiReg(
        uint8_t reg,
        uint8_t *rxBuff,
        uint8_t len,
        bool readExtraByte)
    {
        bmi088ReadWriteByte(reg | Bmi088Data::BMI088_READ_BIT);

        if (readExtraByte)
        {
            bmi088ReadWriteByte(0x55);
        }

        while (len != 0)
        {
            *rxBuff = bmi088ReadWriteByte(0x55);
            rxBuff++;
            len--;
        }
    }

public:
    static inline void bmi088AccWriteSingleReg(
        Bmi088Data::Acc::Register reg,
        Bmi088Data::Acc::Registers_t data)
    {
        chipSelectAccelLow();
        bmi088WriteSingleReg(static_cast<uint8_t>(reg), data.value);
        chipSelectAccelHigh();
    }

    /**
     * From page 45 of the bmi088 datasheet: "In case of read operations, the SPI interface of the
     * accelerometer does not send the requested information directly after the master has sent the
     * corresponding register address, but sends a dummy byte first, whose content is not
     * predictable."
     *
     * Because of this, call `bmi088ReadWriteByte` one time extra to get valid data.
     */
    static inline uint8_t bmi088AccReadSingleReg(Bmi088Data::Acc::Register reg)
    {
        chipSelectAccelLow();
        bmi088ReadSingleReg(static_cast<uint8_t>(reg));
        uint8_t res = bmi088ReadWriteByte(0x55);
        chipSelectAccelHigh();
        return res;
    }

    static inline void bmi088AccReadMultiReg(
        Bmi088Data::Acc::Register reg,
        uint8_t *rxBuff,
        uint8_t len)
    {
        chipSelectAccelLow();
        bmi088ReadMultiReg(static_cast<uint8_t>(reg), rxBuff, len, true);
        chipSelectAccelHigh();
    }

    static inline void bmi088GyroWriteSingleReg(
        Bmi088Data::Gyro::Register reg,
        Bmi088Data::Gyro::Registers_t data)
    {
        chipSelectGyroLow();
        bmi088WriteSingleReg(static_cast<uint8_t>(reg), data.value);
        chipSelectGyroHigh();
    }

    static inline uint8_t bmi088GyroReadSingleReg(Bmi088Data::Gyro::Register reg)
    {
        chipSelectGyroLow();
        uint8_t res = bmi088ReadSingleReg(static_cast<uint8_t>(reg));
        chipSelectGyroHigh();
        return res;
    }

    static inline void bmi088GyroReadMultiReg(
        Bmi088Data::Gyro::Register reg,
        uint8_t *rxBuff,
        uint8_t len)
    {
        chipSelectGyroLow();
        bmi088ReadMultiReg(static_cast<uint8_t>(reg), rxBuff, len, false);
        chipSelectGyroHigh();
    }

#if defined(ENV_UNIT_TESTS)
    /**
     * Insert some data into the queue so the next time bmi088ReadWriteByte is called,
     * it will be returned by the function.
     */
    static void setRxData(uint8_t data) { rxData.push_back(data); }

    static void expectAccWriteSingleReg()
    {
        setRxData(0);
        setRxData(0);
    }

    static void expectAccReadSingleReg(uint8_t data)
    {
        setRxData(0);
        setRxData(0);
        setRxData(data);
    }

    static void expectAccMultiRead(uint8_t *data, uint8_t len)
    {
        setRxData(0);
        setRxData(0);
        for (uint8_t i = 0; i < len; i++)
        {
            setRxData(data[i]);
        }
    }

    static void expectGyroWriteSingleReg()
    {
        setRxData(0);
        setRxData(0);
    }

    static void expectGyroReadSingleReg(uint8_t data)
    {
        setRxData(0);
        setRxData(data);
    }

    static void expectGyroMultiRead(uint8_t *data, uint8_t len)
    {
        setRxData(0);
        for (uint8_t i = 0; i < len; i++)
        {
            setRxData(data[i]);
        }
    }

    static void clearData() { rxData.clear(); }
#endif
};

}  // namespace tap::communication::sensors::imu::bmi088

#endif  // TAPROOT_BMI088_HAL_HPP_
