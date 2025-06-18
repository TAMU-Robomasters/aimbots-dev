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

#include "bmi088.hpp"

#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/board/board.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/math/geometry/angle.hpp"
#include "modm/math/units.hpp"

#include "bmi088_data.hpp"
#include "bmi088_hal.hpp"

using namespace modm::literals;
using namespace tap::arch;
using namespace Board;

namespace tap::communication::sensors::imu::bmi088
{
#if defined(PLATFORM_HOSTED)
#define DELAY_MS(ms)
#define DELAY_US(us)
#else
#define DELAY_MS(ms) modm::delay_ms(ms);
#define DELAY_US(us) modm::delay_us(us);
#endif

Bmi088::Bmi088(tap::Drivers *drivers) : drivers(drivers), imuHeater(drivers) {}

Bmi088::ImuState Bmi088::getImuState() const { return imuState; }

void Bmi088::requestRecalibration()
{
    if (imuState == ImuState::IMU_NOT_CALIBRATED || imuState == ImuState::IMU_CALIBRATED)
    {
        data.gyroOffsetRaw[ImuData::X] = 0;
        data.gyroOffsetRaw[ImuData::Y] = 0;
        data.gyroOffsetRaw[ImuData::Z] = 0;
        data.accOffsetRaw[ImuData::X] = 0;
        data.accOffsetRaw[ImuData::Y] = 0;
        data.accOffsetRaw[ImuData::Z] = 0;
        data.gyroDegPerSec[ImuData::X] = 0;
        data.gyroDegPerSec[ImuData::Y] = 0;
        data.gyroDegPerSec[ImuData::Z] = 0;
        data.accG[ImuData::X] = 0;
        data.accG[ImuData::Y] = 0;
        data.accG[ImuData::Z] = 0;
        calibrationSample = 0;
        imuState = ImuState::IMU_CALIBRATING;
    }
}

void Bmi088::initialize(float sampleFrequency, float mahonyKp, float mahonyKi)
{
#if !defined(PLATFORM_HOSTED)
    ImuCS1Accel::GpioOutput();
    ImuCS1Gyro::GpioOutput();

    DELAY_MS(100);

    ImuSpiMaster::connect<ImuMiso::Miso, ImuMosi::Mosi, ImuSck::Sck>();
    ImuSpiMaster::initialize<SystemClock, 10_MHz>();

    DELAY_MS(1);
#endif

    imuState = ImuState::IMU_NOT_CALIBRATED;

    initializeAcc();
    initializeGyro();

    imuHeater.initialize();

    mahonyAlgorithm.begin(sampleFrequency, mahonyKp, mahonyKi);
}

void Bmi088::initializeAcc()
{
    // Write to the accelerometer a few times to get it to wake up (without this the bmi088 will not
    // turn on properly from cold boot).
    Bmi088Hal::bmi088AccReadSingleReg(Acc::ACC_CHIP_ID);
    DELAY_MS(1);
    Bmi088Hal::bmi088AccReadSingleReg(Acc::ACC_CHIP_ID);
    DELAY_MS(1);

    // Page 13 of the bmi088 datasheet states:
    // After the POR (power-on reset) the gyroscope is in normal mode, while the accelerometer is in
    // suspend mode. To switch the accelerometer into normal mode, the user must perform the
    // following steps:
    // a. Power up the sensor
    // b. Wait 1 ms
    // c. Enter normal mode by writing '4' to ACC_PWR_CTRL
    // d. Wait for 450 microseconds

    Bmi088Hal::bmi088AccWriteSingleReg(Acc::ACC_PWR_CTRL, Acc::AccPwrCtrl::ACCELEROMETER_ON);

    DELAY_US(450);

    // read ACC_CHIP_ID to start SPI communication
    // Page 45 of the bmi088 datasheet states:
    // "To change the sensor to SPI mode in the initialization phase, the user
    // could perform a dummy SPI read operation"
    Bmi088Hal::bmi088AccReadSingleReg(Acc::ACC_CHIP_ID);

    // check communication is normal after reset
    uint8_t readChipID = Bmi088Hal::bmi088AccReadSingleReg(Acc::ACC_CHIP_ID);
    DELAY_MS(1);

    if (readChipID != Acc::ACC_CHIP_ID_VALUE)
    {
        RAISE_ERROR(drivers, "bmi088 accel init failed");
        imuState = ImuState::IMU_NOT_CONNECTED;
        return;
    }

    setAndCheckAccRegister(
        Acc::ACC_CONF,
        Acc::AccBandwidth_t(Acc::AccBandwidth::NORMAL) |
            Acc::AccOutputRate_t(Acc::AccOutputRate::Hz800));

    setAndCheckAccRegister(Acc::ACC_RANGE, ACC_RANGE);
}

void Bmi088::initializeGyro()
{
    // reset gyro
    Bmi088Hal::bmi088GyroWriteSingleReg(Gyro::GYRO_SOFTRESET, Gyro::GyroSoftreset::RESET_SENSOR);
    DELAY_MS(80);

    // check communication normal after reset
    Bmi088Hal::bmi088GyroReadSingleReg(Gyro::GYRO_CHIP_ID);
    DELAY_MS(1);
    uint8_t res = Bmi088Hal::bmi088GyroReadSingleReg(Gyro::GYRO_CHIP_ID);
    DELAY_MS(1);

    if (res != Gyro::GYRO_CHIP_ID_VALUE)
    {
        RAISE_ERROR(drivers, "bmi088 gyro init failed");
        imuState = ImuState::IMU_NOT_CONNECTED;
    }

    setAndCheckGyroRegister(Gyro::GYRO_RANGE, GYRO_RANGE);

    // extra 0x80 is because the bandwidth register will always have 0x80 masked
    // so when checking, we want to mask as well to avoid an error
    setAndCheckGyroRegister(
        Gyro::GYRO_BANDWIDTH,
        Gyro::GyroBandwidth::ODR1000_BANDWIDTH116 | Gyro::GyroBandwidth_t(0x80));

    setAndCheckGyroRegister(Gyro::GYRO_LPM1, Gyro::GyroLpm1::PWRMODE_NORMAL);
}

void Bmi088::periodicIMUUpdate()
{
    if (imuState == ImuState::IMU_NOT_CONNECTED)
    {
        RAISE_ERROR(drivers, "periodicIMUUpdate called w/ imu not connected");
        return;
    }

    uint8_t rxBuff[6] = {};

    Bmi088Hal::bmi088AccReadMultiReg(Acc::ACC_X_LSB, rxBuff, 6);

    prevIMUDataReceivedTime = tap::arch::clock::getTimeMicroseconds();

    data.accRaw[ImuData::X] = bigEndianInt16ToFloat(rxBuff);
    data.accRaw[ImuData::Y] = bigEndianInt16ToFloat(rxBuff + 2);
    data.accRaw[ImuData::Z] = bigEndianInt16ToFloat(rxBuff + 4);

    Bmi088Hal::bmi088GyroReadMultiReg(Gyro::RATE_X_LSB, rxBuff, 6);
    data.gyroRaw[ImuData::X] = bigEndianInt16ToFloat(rxBuff);
    data.gyroRaw[ImuData::Y] = bigEndianInt16ToFloat(rxBuff + 2);
    data.gyroRaw[ImuData::Z] = bigEndianInt16ToFloat(rxBuff + 4);

    Bmi088Hal::bmi088AccReadMultiReg(Acc::TEMP_MSB, rxBuff, 2);
    data.temperature = parseTemp(rxBuff[0], rxBuff[1]);

    if (imuState == ImuState::IMU_CALIBRATING)
    {
        computeOffsets();
    }
    else
    {
        data.gyroDegPerSec[ImuData::X] =
            GYRO_DS_PER_GYRO_COUNT * (data.gyroRaw[ImuData::X] - data.gyroOffsetRaw[ImuData::X]);
        data.gyroDegPerSec[ImuData::Y] =
            GYRO_DS_PER_GYRO_COUNT * (data.gyroRaw[ImuData::Y] - data.gyroOffsetRaw[ImuData::Y]);
        data.gyroDegPerSec[ImuData::Z] =
            GYRO_DS_PER_GYRO_COUNT * (data.gyroRaw[ImuData::Z] - data.gyroOffsetRaw[ImuData::Z]);

        data.accG[ImuData::X] =
            ACC_G_PER_ACC_COUNT * (data.accRaw[ImuData::X] - data.accOffsetRaw[ImuData::X]);
        data.accG[ImuData::Y] =
            ACC_G_PER_ACC_COUNT * (data.accRaw[ImuData::Y] - data.accOffsetRaw[ImuData::Y]);
        data.accG[ImuData::Z] =
            ACC_G_PER_ACC_COUNT * (data.accRaw[ImuData::Z] - data.accOffsetRaw[ImuData::Z]);

        mahonyAlgorithm.updateIMU(
            data.gyroDegPerSec[ImuData::X],
            data.gyroDegPerSec[ImuData::Y],
            data.gyroDegPerSec[ImuData::Z],
            data.accG[ImuData::X],
            data.accG[ImuData::Y],
            data.accG[ImuData::Z]);
    }

    imuHeater.runTemperatureController(data.temperature);
}

void Bmi088::computeOffsets()
{
    calibrationSample++;

    data.gyroOffsetRaw[ImuData::X] += data.gyroRaw[ImuData::X];
    data.gyroOffsetRaw[ImuData::Y] += data.gyroRaw[ImuData::Y];
    data.gyroOffsetRaw[ImuData::Z] += data.gyroRaw[ImuData::Z];
    data.accOffsetRaw[ImuData::X] += data.accRaw[ImuData::X];
    data.accOffsetRaw[ImuData::Y] += data.accRaw[ImuData::Y];
    data.accOffsetRaw[ImuData::Z] +=
        data.accRaw[ImuData::Z] - (tap::algorithms::ACCELERATION_GRAVITY / ACC_G_PER_ACC_COUNT);

    if (calibrationSample >= BMI088_OFFSET_SAMPLES)
    {
        calibrationSample = 0;
        data.gyroOffsetRaw[ImuData::X] /= BMI088_OFFSET_SAMPLES;
        data.gyroOffsetRaw[ImuData::Y] /= BMI088_OFFSET_SAMPLES;
        data.gyroOffsetRaw[ImuData::Z] /= BMI088_OFFSET_SAMPLES;
        data.accOffsetRaw[ImuData::X] /= BMI088_OFFSET_SAMPLES;
        data.accOffsetRaw[ImuData::Y] /= BMI088_OFFSET_SAMPLES;
        data.accOffsetRaw[ImuData::Z] /= BMI088_OFFSET_SAMPLES;
        imuState = ImuState::IMU_CALIBRATED;
        mahonyAlgorithm.reset();
    }
}

void Bmi088::setAndCheckAccRegister(Acc::Register reg, Acc::Registers_t value)
{
    Bmi088Hal::bmi088AccWriteSingleReg(reg, value);
    DELAY_US(150);

    uint8_t val = Bmi088Hal::bmi088AccReadSingleReg(reg);
    DELAY_US(150);

    if (val != value.value)
    {
        RAISE_ERROR(drivers, "bmi088 acc config failed");
        imuState = ImuState::IMU_NOT_CONNECTED;
    }
}

void Bmi088::setAndCheckGyroRegister(Gyro::Register reg, Gyro::Registers_t value)
{
    Bmi088Hal::bmi088GyroWriteSingleReg(reg, value);
    DELAY_US(150);

    uint8_t val = Bmi088Hal::bmi088GyroReadSingleReg(reg);
    DELAY_US(150);

    if (val != value.value)
    {
        RAISE_ERROR(drivers, "bmi088 gyro config failed");
        imuState = ImuState::IMU_NOT_CONNECTED;
    }
}

}  // namespace tap::communication::sensors::imu::bmi088
