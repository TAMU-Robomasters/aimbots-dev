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

#include "mpu6500.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"
#include "tap/rm-dev-board-a/board.hpp"

#include "mpu6500_reg.hpp"

namespace tap
{
namespace sensors
{
using namespace modm::literals;

void Mpu6500::init()
{
#ifndef PLATFORM_HOSTED
    Board::ImuNss::GpioOutput();

    // connect GPIO pins to the alternate SPI function
    Board::ImuSpiMaster::connect<Board::ImuMiso::Miso, Board::ImuMosi::Mosi, Board::ImuSck::Sck>();

    // initialize SPI with clock speed
    Board::ImuSpiMaster::initialize<Board::SystemClock, 703125_Hz>();

    // set power mode
    spiWriteRegister(MPU6500_PWR_MGMT_1, 0x80);

    modm::delay_ms(100);

    // reset gyro, accel, and temperature
    spiWriteRegister(MPU6500_SIGNAL_PATH_RESET, 0x07);
    modm::delay_ms(100);

    // verify mpu register ID
    if (MPU6500_ID != spiReadRegister(MPU6500_WHO_AM_I))
    {
        RAISE_ERROR(
            drivers,
            "failed to initialize the imu properly",
            tap::errors::Location::MPU6500,
            tap::errors::Mpu6500ErrorType::IMU_NOT_RECEIVING_PROPERLY);
        return;
    }

    imuInitialized = true;

    // 0: 250hz; 1: 184hz; 2: 92hz; 3: 41hz; 4: 20hz; 5: 10hz; 6: 5hz; 7: 3600hz
    uint8_t Mpu6500InitData[7][2] = {
        {MPU6500_PWR_MGMT_1, 0x03},      // Auto selects Clock Source
        {MPU6500_PWR_MGMT_2, 0x00},      // all enable
        {MPU6500_CONFIG, 0x02},          // gyro bandwidth 0x00:250Hz 0x04:20Hz
        {MPU6500_GYRO_CONFIG, 0x18},     // gyro range 0x10:+-1000dps 0x18:+-2000dps
        {MPU6500_ACCEL_CONFIG, 0x10},    // acc range 0x10:+-8G
        {MPU6500_ACCEL_CONFIG_2, 0x00},  // acc bandwidth 0x00:250Hz 0x04:20Hz
        {MPU6500_USER_CTRL, 0x20},       // Enable the I2C Master I/F module
                                         // pins ES_DA and ES_SCL are isolated from
                                         // pins SDA/SDI and SCL/SCLK.
    };

    // write init setting to registers
    for (int i = 0; i < 7; i++)
    {
        spiWriteRegister(Mpu6500InitData[i][0], Mpu6500InitData[i][1]);
        modm::delay_ms(1);
    }

    calculateAccOffset();
    calculateGyroOffset();

    readRegistersTimeout.restart(DELAY_BTWN_CALC_AND_READ_REG);
#endif
}

void Mpu6500::calcIMUAngles()
{
    if (imuInitialized)
    {
        mahonyAlgorithm.updateIMU(getGx(), getGy(), getGz(), getAx(), getAy(), getAz());
        tiltAngleCalculated = false;
        // Start reading registers in DELAY_BTWN_CALC_AND_READ_REG us
        readRegistersTimeout.restart(DELAY_BTWN_CALC_AND_READ_REG);
    }
    else
    {
        RAISE_ERROR(
            drivers,
            "failed to initialize the imu properly",
            tap::errors::Location::MPU6500,
            tap::errors::Mpu6500ErrorType::IMU_DATA_NOT_INITIALIZED);
    }
}

bool Mpu6500::read()
{
#ifndef PLATFORM_HOSTED
    PT_BEGIN();
    while (true)
    {
        PT_WAIT_UNTIL(readRegistersTimeout.execute());

        mpuNssLow();
        tx = MPU6500_ACCEL_XOUT_H | 0x80;
        rx = 0;
        txBuff[0] = tx;
        PT_CALL(Board::ImuSpiMaster::transfer(&tx, &rx, 1));
        PT_CALL(Board::ImuSpiMaster::transfer(txBuff, rxBuff, ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE));
        mpuNssHigh();

        raw.accel.x = (rxBuff[0] << 8 | rxBuff[1]) - raw.accelOffset.x;
        raw.accel.y = (rxBuff[2] << 8 | rxBuff[3]) - raw.accelOffset.y;
        raw.accel.z = (rxBuff[4] << 8 | rxBuff[5]) - raw.accelOffset.z;

        raw.temperature = rxBuff[6] << 8 | rxBuff[7];

        raw.gyro.x = ((rxBuff[8] << 8 | rxBuff[9]) - raw.gyroOffset.x);
        raw.gyro.y = ((rxBuff[10] << 8 | rxBuff[11]) - raw.gyroOffset.y);
        raw.gyro.z = ((rxBuff[12] << 8 | rxBuff[13]) - raw.gyroOffset.z);
    }
    PT_END();
#else
    return false;
#endif
}

// Getter functions.

bool Mpu6500::initialized() const { return imuInitialized; }

float Mpu6500::getAx() const
{
    return validateReading(
        static_cast<float>(raw.accel.x) * ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
}

float Mpu6500::getAy() const
{
    return validateReading(
        static_cast<float>(raw.accel.y) * ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
}

float Mpu6500::getAz() const
{
    return validateReading(
        static_cast<float>(raw.accel.z) * ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
}

float Mpu6500::getGx() const
{
    return validateReading(static_cast<float>(raw.gyro.x) / LSB_D_PER_S_TO_D_PER_S);
}

float Mpu6500::getGy() const
{
    return validateReading(static_cast<float>(raw.gyro.y) / LSB_D_PER_S_TO_D_PER_S);
}

float Mpu6500::getGz() const
{
    return validateReading(static_cast<float>(raw.gyro.z) / LSB_D_PER_S_TO_D_PER_S);
}

float Mpu6500::getTemp() const
{
    return validateReading(21.0f + static_cast<float>(raw.temperature) / 333.87f);
}

float Mpu6500::getYaw() { return validateReading(mahonyAlgorithm.getYaw()); }

float Mpu6500::getPitch() { return validateReading(mahonyAlgorithm.getPitch()); }

float Mpu6500::getRoll() { return validateReading(mahonyAlgorithm.getRoll()); }

float Mpu6500::getTiltAngle()
{
    if (!tiltAngleCalculated)
    {
        tiltAngle = tap::algorithms::radiansToDegrees(acosf(
            cosf(mahonyAlgorithm.getPitchRadians()) * cosf(mahonyAlgorithm.getRollRadians())));
        tiltAngleCalculated = true;
    }
    return validateReading(tiltAngle);
}

float Mpu6500::validateReading(float reading) const
{
    if (imuInitialized)
    {
        return reading;
    }
    RAISE_ERROR(
        drivers,
        "failed to initialize the imu properly",
        tap::errors::Location::MPU6500,
        tap::errors::Mpu6500ErrorType::IMU_DATA_NOT_INITIALIZED);
    return 0.0f;
}

// Helper functions for calibration.

void Mpu6500::calculateGyroOffset()
{
#ifndef PLATFORM_HOSTED
    for (int i = 0; i < MPU6500_OFFSET_SAMPLES; i++)
    {
        spiReadRegisters(MPU6500_ACCEL_XOUT_H, rxBuff, 14);
        raw.gyroOffset.x += (rxBuff[8] << 8) | rxBuff[9];
        raw.gyroOffset.y += (rxBuff[10] << 8) | rxBuff[11];
        raw.gyroOffset.z += (rxBuff[12] << 8) | rxBuff[13];
        modm::delay_ms(2);
    }

    raw.gyroOffset.x /= MPU6500_OFFSET_SAMPLES;
    raw.gyroOffset.y /= MPU6500_OFFSET_SAMPLES;
    raw.gyroOffset.z /= MPU6500_OFFSET_SAMPLES;
#endif
}

void Mpu6500::calculateAccOffset()
{
#ifndef PLATFORM_HOSTED
    for (int i = 0; i < MPU6500_OFFSET_SAMPLES; i++)
    {
        spiReadRegisters(MPU6500_ACCEL_XOUT_H, rxBuff, 14);
        raw.accelOffset.x += (rxBuff[0] << 8) | rxBuff[1];
        raw.accelOffset.y += (rxBuff[2] << 8) | rxBuff[3];
        raw.accelOffset.z += ((rxBuff[4] << 8) | rxBuff[5]) - 4096;
        modm::delay_ms(2);
    }

    raw.accelOffset.x /= MPU6500_OFFSET_SAMPLES;
    raw.accelOffset.y /= MPU6500_OFFSET_SAMPLES;
    raw.accelOffset.z /= MPU6500_OFFSET_SAMPLES;
#endif
}

// Hardware interface functions (blocking functions, for initialization only)

uint8_t Mpu6500::spiWriteRegister(uint8_t reg, uint8_t data)
{
#ifndef PLATFORM_HOSTED
    mpuNssLow();
    uint8_t tx = reg & 0x7F;
    uint8_t rx = 0;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    tx = data;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    mpuNssHigh();
#endif
    return 0;
}

uint8_t Mpu6500::spiReadRegister(uint8_t reg)
{
#ifndef PLATFORM_HOSTED
    mpuNssLow();
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    mpuNssHigh();
    return rx;
#else
    return 0;
#endif
}

uint8_t Mpu6500::spiReadRegisters(uint8_t regAddr, uint8_t *pData, uint8_t len)
{
#ifndef PLATFORM_HOSTED
    mpuNssLow();
    uint8_t tx = regAddr | 0x80;
    uint8_t rx = 0;
    txBuff[0] = tx;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(txBuff, pData, len);
    mpuNssHigh();
#endif
    return 0;
}

void Mpu6500::mpuNssLow()
{
#ifndef PLATFORM_HOSTED
    Board::ImuNss::setOutput(modm::GpioOutput::Low);
#endif
}

void Mpu6500::mpuNssHigh()
{
#ifndef PLATFORM_HOSTED
    Board::ImuNss::setOutput(modm::GpioOutput::High);
#endif
}

}  // namespace sensors

}  // namespace tap
