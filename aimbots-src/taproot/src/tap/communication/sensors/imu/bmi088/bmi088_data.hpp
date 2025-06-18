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

#ifndef TAPROOT_BMI088_DATA_HPP_
#define TAPROOT_BMI088_DATA_HPP_

#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"

namespace tap::communication::sensors::imu::bmi088
{
/**
 * For register tables and descriptions, refer to the bmi088 datasheet:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
 */
class Bmi088Data
{
public:
    /**
     * Bit appended or removed from a register while reading/writing.
     */
    static constexpr uint8_t BMI088_READ_BIT = 0x80;

    struct Gyro
    {
        enum Register : uint8_t
        {
            GYRO_CHIP_ID = 0x00,
            RATE_X_LSB = 0X02,
            RATE_X_MSB = 0X03,
            RATE_Y_LSB = 0X04,
            RATE_Y_MSB = 0X05,
            RATE_Z_LSB = 0X06,
            RATE_Z_MSB = 0X07,
            GYRO_INT_STAT_1 = 0x0a,
            FIFO_STATUS = 0x0e,
            GYRO_RANGE = 0x0f,
            GYRO_BANDWIDTH = 0x10,
            GYRO_LPM1 = 0x11,
            GYRO_SOFTRESET = 0x14,
            GYRO_INT_CTRL = 0x15,
            INT3_INT4_IO_CONF = 0x16,
            INT3_INT4_IO_MAP = 0x18,
            FIFO_WM_EN = 0x1e,
            FIFO_EXT_INT_S = 0x34,
            GYRO_SELF_TEST = 0x3c,
            FIFO_CONFIG_0 = 0x3d,
            FIFO_CONFIG_1 = 0x3e,
            FIFO_DATA = 0x3f,
        };

        /// The id of the gyroscope that will is stored in address `GYRO_CHIP_ID`.
        static constexpr uint8_t GYRO_CHIP_ID_VALUE = 0x0f;

        enum class GyroIntStat1 : uint8_t
        {
            GYRO_DRDY = modm::Bit7,
            FIFO_INT = modm::Bit4,
        };
        MODM_FLAGS8(GyroIntStat1);

        enum class FifoStatus : uint8_t
        {
            FIFO_OVERRUN = modm::Bit7,
            FIFO_FRAME_COUNTER = static_cast<uint8_t>(~modm::Bit7),
        };
        MODM_FLAGS8(FifoStatus);

        enum class GyroRange : uint8_t
        {
            DPS2000 = 0x00,
            DPS1000 = 0x01,
            DPS500 = 0x02,
            DPS250 = 0x03,
            DPS125 = 0x04,
        };
        MODM_FLAGS8(GyroRange);

        enum class GyroBandwidth : uint8_t
        {
            ODR2000_BANDWIDTH532 = 0x00,
            ODR2000_BANDWIDTH230 = 0x01,
            ODR1000_BANDWIDTH116 = 0x02,
            ODR400_BANDWIDTH47 = 0x03,
            ODR200_BANDWIDTH23 = 0x04,
            ODR100_BANDWIDTH12 = 0x05,
            ODR200_BANDWIDTH64 = 0x06,
            ODR100_BANDWIDTH32 = 0x07,
        };
        MODM_FLAGS8(GyroBandwidth);

        enum class GyroLpm1
        {
            PWRMODE_NORMAL = 0x00,
            PWRMODE_SUSPEND = 0x80,
            PWRMODE_DEEP_SUSPEND = 0x20
        };
        MODM_FLAGS8(GyroLpm1);

        enum class GyroSoftreset : uint8_t
        {
            RESET_SENSOR = 0xb6
        };
        MODM_FLAGS8(GyroSoftreset);

        enum class GyroIntCtrl : uint8_t
        {
            EnableNewDataInt_Mask = modm::Bit7,
            EnableFifoInt_Mask = modm::Bit6,
        };
        MODM_FLAGS8(GyroIntCtrl);

        enum class EnableNewDataInt : uint8_t
        {
            DISABLED = 0x00,
            ENABLED = 0x01,
        };
        MODM_FLAGS_CONFIG(GyroIntCtrl, EnableNewDataInt);

        enum class EnableFifoInt : uint8_t
        {
            DISABLED = 0x00,
            ENABLED = 0x01,
        };
        MODM_FLAGS_CONFIG(GyroIntCtrl, EnableFifoInt);

        enum class Int3Int4IoConf : uint8_t
        {
            Int4Od_Mask = modm::Bit3,
            Int4Lvl_Mask = modm::Bit2,
            Int3Od_Mask = modm::Bit1,
            Int3Lvl_Mask = modm::Bit0,
        };
        MODM_FLAGS8(Int3Int4IoConf);

        enum class Int4Od : uint8_t
        {
            PUSH_PULL = 0,
            OPEN_DRAIN = 1,
        };
        MODM_FLAGS_CONFIG(Int3Int4IoConf, Int4Od);

        enum class Int4Lvl : uint8_t
        {
            ACTIVE_LOW = 0,
            ACTIVE_HIGH = 1,
        };
        MODM_FLAGS_CONFIG(Int3Int4IoConf, Int4Lvl);

        enum class Int3Od : uint8_t
        {
            PUSH_PULL = 0,
            OPEN_DRAIN = 1,
        };
        MODM_FLAGS_CONFIG(Int3Int4IoConf, Int3Od);

        enum class Int3Lvl : uint8_t
        {
            ACTIVE_LOW = 0,
            ACTIVE_HIGH = 1,
        };
        MODM_FLAGS_CONFIG(Int3Int4IoConf, Int3Lvl);

        enum class Int3Int4IoMap : uint8_t
        {
            DATA_READY_INT4 = modm::Bit7,
            FIFO_INT4 = modm::Bit5,
            FIFO_INT3 = modm::Bit2,
            DATA_READY_INT3 = modm::Bit0,
        };
        MODM_FLAGS8(Int3Int4IoMap);

        enum class FifoWmEnable : uint8_t
        {
            FIFO_WATERMARK_LVL_INT_DISABLED = 0x08,
            FIFO_WATERMARK_LVL_INT_ENABLED = 0x88,
        };
        MODM_FLAGS8(FifoWmEnable);

        enum class FifoExtIntS : uint8_t
        {
            ENABLE_EXTERNAL_FIFO_SYNCH_MODE = modm::Bit5,
            EXT_FIFO_S_SEL = modm::Bit4,
        };
        MODM_FLAGS8(FifoExtIntS);

        enum class GyroSelfTest : uint8_t
        {
            RATE_OK = modm::Bit4,
            BIST_FAIL = modm::Bit2,
            BIST_RDY = modm::Bit1,
            TRIG_BIST = modm::Bit0,
        };
        MODM_FLAGS8(GyroSelfTest);

        /**
         * defines the FIFO watermark level. An interrupt will be generated, when the number of
         * entries in the FIFO exceeds fifo_water_mark_level_trigger_retain<6:0>. Writing to this
         * register clears the FIFO buffer.
         */
        enum class FifoConfig0 : uint8_t
        {
            FIFO_WATER_MARK_LVL_TRIGGER_RETAIN = static_cast<uint8_t>(~modm::Bit7),
        };
        MODM_FLAGS8(FifoConfig0);

        enum class FifoConfig1 : uint8_t
        {
            FIFO = 0x40,
            STREAM = 0x80,
        };
        MODM_FLAGS8(FifoConfig1);

        using Registers_t = modm::FlagsGroup<
            GyroIntStat1_t,
            FifoStatus_t,
            GyroRange_t,
            GyroBandwidth_t,
            GyroLpm1_t,
            GyroSoftreset_t,
            GyroIntCtrl_t,
            Int3Int4IoConf_t,
            Int3Int4IoMap_t,
            FifoWmEnable_t,
            FifoExtIntS_t,
            GyroSelfTest_t,
            FifoConfig0_t,
            FifoConfig1_t>;
    };

    struct Acc
    {
        /// List of register addresses for the bmi088's accelerometer
        enum Register : uint8_t
        {
            ACC_CHIP_ID = 0x00,
            ACC_ERR_REG = 0x02,
            ACC_STATUS = 0x03,
            ACC_X_LSB = 0x12,
            ACC_X_MSB = 0x13,
            ACC_Y_LSB = 0x14,
            ACC_Y_MSB = 0x15,
            ACC_Z_LSB = 0x16,
            ACC_Z_MSB = 0x17,
            SENSORTIME_0 = 0x18,
            SENSORTIME_1 = 0x19,
            SENSORTIME_2 = 0x1a,
            ACC_INT_STAT_1 = 0x1d,
            TEMP_MSB = 0x22,
            TEMP_LSB = 0x23,
            FIFO_LENGTH_0 = 0x24,
            FIFO_LENGTH_1 = 0x25,
            FIFO_DATA = 0x26,
            ACC_CONF = 0x40,
            ACC_RANGE = 0x41,
            INT1_IO_CTRL = 0x53,
            INT2_IO_CTRL = 0x54,
            INT_MAP_DATA = 0x58,
            ACC_SELF_TEST = 0x6d,
            ACC_PWR_CONF = 0x7c,
            ACC_PWR_CTRL = 0x7d,
            ACC_SOFTRESET = 0x7e,
        };

        /// The id of the accelerometer that will is stored in address `ACC_CHIP_ID`.
        static constexpr uint8_t ACC_CHIP_ID_VALUE = 0x1e;

        enum class AccErr : uint8_t
        {
            ERROR_CODE = modm::Bit2 | modm::Bit3 | modm::Bit4,
            FATAL_ERR = modm::Bit0,
        };
        MODM_FLAGS8(AccErr);

        enum class AccStatus : uint8_t
        {
            ACC_DRDY = modm::Bit7,
        };
        MODM_FLAGS8(AccStatus);

        enum class AccIntStat1 : uint8_t
        {
            ACC_DRDY = modm::Bit7,
        };
        MODM_FLAGS8(AccIntStat1);

        enum class AccConf : uint8_t
        {
            AccBandwidth_Mask = modm::Bit4 | modm::Bit5 | modm::Bit6 | modm::Bit7,
            AccOutputRate_Mask = modm::Bit0 | modm::Bit1 | modm::Bit2 | modm::Bit3,
        };
        MODM_FLAGS8(AccConf);

        /// @see section 4.4.1 of the bmi088 datasheet.
        enum class AccBandwidth : uint8_t
        {
            OSR4_OVERSAMPLING = 0x08,
            OSR2_OVERSAMPLING = 0x09,
            NORMAL = 0x0a,
        };
        typedef modm::Configuration<AccConf_t, AccBandwidth, 0b1111, 4> AccBandwidth_t;  // Bit 4..7

        enum class AccOutputRate : uint8_t
        {
            Hz12_5 = 0x05,
            Hz25 = 0x06,
            Hz50 = 0x07,
            Hz100 = 0x08,
            Hz200 = 0x09,
            Hz400 = 0x0a,
            Hz800 = 0x0b,
            Hz1600 = 0x0c,
        };
        MODM_FLAGS_CONFIG(AccConf, AccOutputRate);

        enum class AccRange : uint8_t
        {
            G3 = 0x0,
            G6 = 0x1,
            G12 = 0x2,
            G24 = 0x03,
        };
        MODM_FLAGS8(AccRange);

        enum class IntMapData : uint8_t
        {
            INT2_DRDY = modm::Bit6,
            Int2_FWRM = modm::Bit5,
            INT2_FFULL = modm::Bit4,
            INT1_DRDY = modm::Bit2,
            INT1_FWM = modm::Bit1,
            INT1_FFULL = modm::Bit0,
        };
        MODM_FLAGS8(IntMapData);

        enum class Int1IoConf : uint8_t
        {
            INT1_IN = modm::Bit4,
            INT1_OUT = modm::Bit3,
            Int1Od_Mask = modm::Bit2,
            Int1Lvl_Mask = modm::Bit1,
        };
        MODM_FLAGS8(Int1IoConf);

        enum class Int1Od : uint8_t
        {
            PUSH_PULL = 0,
            OPEN_DRAIN = 1,
        };
        typedef modm::Configuration<Int1IoConf_t, Int1Od, 0b1, 2> Int1Od_t;

        enum class Int1Lvl : uint8_t
        {
            ACTIVE_LOW = 0,
            ACTIVE_HIGH = 1,
        };
        typedef modm::Configuration<Int1IoConf_t, Int1Lvl, 0b1, 1> Int1Lvl_t;

        enum class Int2IoConf : uint8_t
        {
            INT2_IO = modm::Bit4,
            INT2_OUT = modm::Bit3,
            Int2Od_Mask = modm::Bit2,
            Int2Lvl_Mask = modm::Bit1,
        };
        MODM_FLAGS8(Int2IoConf);

        enum class Int2Od : uint8_t
        {
            PUSH_PULL = 0,
            OPEN_DRAIN = 1,
        };
        typedef modm::Configuration<Int2IoConf_t, Int2Od, 0b1, 2> Int2Od_t;

        enum class Int2Lvl : uint8_t
        {
            ACTIVE_LOW = 0x00,
            ACTIVE_HIGH = 0x01,
        };
        typedef modm::Configuration<Int2IoConf_t, Int2Lvl, 0b1, 1> Int2Lv1_t;

        enum class AccSelfTest : uint8_t
        {
            SELF_TEST_OFF = 0x00,
            POSITIVE_SELF_TEST_SIGNAL = 0x0d,
            NEGATIVE_SELF_TEST_SIGNAL = 0x09,
        };
        MODM_FLAGS8(AccSelfTest);

        enum class AccPwrConf : uint8_t
        {
            ACTIVE_MODE = 0x00,
            SUSPEND_MODE = 0x03
        };
        MODM_FLAGS8(AccPwrConf);

        enum class AccPwrCtrl : uint8_t
        {
            ACCELEROMETER_OFF = 0X00,
            ACCELEROMETER_ON = 0X04
        };
        MODM_FLAGS8(AccPwrCtrl);

        /// Writing this to the AccSoftreset register will perform a soft reset of the IMU
        enum class AccSoftreset : uint8_t
        {
            ACC_SOFTRESET_VAL = 0xb6,
        };
        MODM_FLAGS8(AccSoftreset);

        using Registers_t = modm::FlagsGroup<
            AccErr_t,
            AccStatus_t,
            AccIntStat1_t,
            AccConf_t,
            Int1IoConf_t,
            Int2IoConf_t,
            AccSelfTest_t,
            AccRange_t,
            AccPwrConf_t,
            AccPwrCtrl_t,
            AccSoftreset_t,
            IntMapData_t>;
    };
};

}  // namespace tap::communication::sensors::imu::bmi088

#endif  // TAPROOT_BMI088_DATA_HPP_
