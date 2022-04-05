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

#ifndef TAPROOT_MPU6500_HPP_
#define TAPROOT_MPU6500_HPP_

#include <cstdint>

#include "tap/algorithms/MahonyAHRS.h"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/imu_heater/imu_heater.hpp"
#include "tap/util_macros.hpp"

#include "modm/processing/protothread.hpp"

namespace tap
{
class Drivers;
namespace sensors
{
/**
 * A class specifically designed for interfacing with the RoboMaster type A board Mpu6500.
 *
 * To use this class, call Remote::init() to properly initialize and calibrate
 * the MPU6500. Next, call Remote::read() to read acceleration, gyro, and temperature
 * values from the imu. Use the getter methods to access imu information.
 *
 * @note if you are shaking the imu while it is initializing, the offsets will likely
 *      be calibrated poorly and unexpectedly bad results may occur.
 */
class Mpu6500 : public ::modm::pt::Protothread
{
public:
    /**
     * Possible IMU states for the Mpu6500.
     */
    enum class ImuState
    {
        /** Indicates the IMU's init function was not called or initialization failed, so data from
           this class will be undefined. */
        IMU_NOT_CONNECTED,
        /** Indicates the IMU is connected and reading data, but calibration offsets have not been
           computed. */
        IMU_NOT_CALIBRATED,
        /** Indicates the IMU is in the process of computing calibration offsets. Data read when the
           IMU is in this state is undefined. */
        IMU_CALIBRATING,
        /// Indicates the IMU is connected and calibration offsets have been computed.
        IMU_CALIBRATED,
    };

    Mpu6500(Drivers *drivers);
    DISALLOW_COPY_AND_ASSIGN(Mpu6500)
    mockable ~Mpu6500() = default;

    /**
     * Initialize the imu and the SPI line. Uses SPI1, which is internal to the
     * type A board.
     *
     * @note this function can block for approximately 12 seconds.
     */
    mockable void init();

    /**
     * Calculates the IMU's pitch, roll, and yaw angles usign the Mahony AHRS algorithm.
     * Also runs a controller to keep the temperature constant.
     * Call at 500 hz for best performance.
     */
    mockable void periodicIMUUpdate();

    /**
     * Read data from the imu. This is a protothread that reads the SPI bus using
     * nonblocking I/O.
     *
     * @return `true` if the function is not done, `false` otherwise
     */
    mockable bool read();

    /**
     * Returns the state of the IMU. Can be not connected, connected but not calibrated, calibrating
     * or calibrated. When not connected, IMU data is undefiend. When not calibrated, IMU data is
     * valid but the computed yaw angle data will drift. When calibrating, the IMU data is invalid.
     * When calibrated, the IMU data is valid and assuming proper calibration the IMU data should
     * not drift.
     *
     * To be safe, whenever you call functions that return IMU (acceleration, gyroscope,
     * temperature, and angle) data, call this function to ensure the data you are about to receive
     * is not undefined.
     */
    mockable inline ImuState getImuState() const { return imuState; }

    /**
     * Returns the acceleration reading in the x direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    mockable float getAx() const;

    /**
     * Returns the acceleration reading in the y direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    mockable float getAy() const;

    /**
     * Returns the acceleration reading in the z direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    mockable float getAz() const;

    /**
     * Returns the gyroscope reading in the x direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    mockable float getGx() const;

    /**
     * Returns the gyroscope reading in the y direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    mockable float getGy() const;

    /**
     * Returns the gyroscope reading in the z direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    mockable float getGz() const;

    /**
     * Returns the temperature of the imu in degrees C.
     */
    mockable float getTemp() const;

    /**
     * Returns yaw angle. in degrees.
     */
    mockable float getYaw();

    /**
     * Returns pitch angle in degrees.
     */
    mockable float getPitch();

    /**
     * Returns roll angle in degrees.
     */
    mockable float getRoll();

    /**
     * Returns the angle difference between the normal vector of the plane that the
     * type A board lies on and of the angle directly upward.
     */
    mockable float getTiltAngle();

    /**
     * Uninitializes the mpu6500 and enters calibration mode.
     */
    mockable void requestCalibration();

    /**
     * Use for converting from gyro values we receive to more conventional degrees / second.
     */
    static constexpr float LSB_D_PER_S_TO_D_PER_S = 16.384f;

private:
    static constexpr float ACCELERATION_GRAVITY = 9.80665f;

    /**
     * Use to convert the raw acceleration into more conventional degrees / second^2
     */
    static constexpr float ACCELERATION_SENSITIVITY = 4096.0f;

    /**
     * The number of samples we take while calibrating in order to determine the mpu offsets.
     */
    static constexpr float MPU6500_OFFSET_SAMPLES = 1000;

    /**
     * The number of bytes read to read acceleration, gyro, and temperature.
     */
    static constexpr uint8_t ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE = 14;

    /**
     * The delay between calculation of imu angles and the start of reading new raw IMU data,
     * in microseconds.
     */
    static constexpr int DELAY_BTWN_CALC_AND_READ_REG = 1550;

    /**
     * Time in ms to wait for the IMU heat to stabalize upon initialization.
     */
    static constexpr uint32_t MAX_WAIT_FOR_IMU_TEMPERATURE_STABALIZE = 10'000;

    /**
     * Time in ms to wait after IMU heat has reached stable point upon initialization.
     */
    static constexpr uint32_t WAIT_TIME_AFTER_CALIBRATION = 10'000;

    /**
     * Bit appended or removed from a register while reading/writing.
     */
    static constexpr uint8_t MPU6500_READ_BIT = 0x80;

    /**
     * Storage for the raw data we receive from the mpu6500, as well as offsets
     * that are used each time we receive data.
     */
    struct RawData
    {
        struct Vector
        {
            float x = 0;
            float y = 0;
            float z = 0;
        };

        /**
         * Raw acceleration data.
         */
        Vector accel;
        /**
         * Raw gyroscope data.
         */
        Vector gyro;

        /**
         * Raw temperature.
         */
        uint16_t temperature = 0;

        /**
         * Acceleration offset calculated in init.
         */
        Vector accelOffset;
        /**
         * Gyroscope offset calculated in init.
         */
        Vector gyroOffset;
    };

    Drivers *drivers;

    ImuState imuState = ImuState::IMU_NOT_CONNECTED;

    tap::arch::MicroTimeout readRegistersTimeout;
    uint8_t tx = 0;  ///< Byte used for reading data in the read protothread
    uint8_t rx = 0;  ///< Byte used for reading data in the read protothread

    RawData raw;

    Mahony mahonyAlgorithm;

    sensors::ImuHeater imuHeater;

    float tiltAngle = 0.0f;
    bool tiltAngleCalculated = false;

    uint8_t txBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = {0};

    uint8_t rxBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = {0};

    int calibrationSample = 0;

    // Functions for interacting with hardware directly.

    /**
     * Pull the NSS pin low to initiate contact with the imu.
     */
    void mpuNssLow();

    /**
     * Pull the NSS pin high to end contact with the imu.
     */
    void mpuNssHigh();

    /**
     * If the imu is not initializes, logs an error and returns 0,
     * otherwise returns the value passed in.
     */
    inline float validateReading(float reading) const;

    /**
     * Write to a given register.
     */
    void spiWriteRegister(uint8_t reg, uint8_t data);

    /**
     * Read from a given register.
     */
    uint8_t spiReadRegister(uint8_t reg);

    /**
     * Read from several registers.
     * regAddr is the first address read, and it reads len number of addresses
     * from that point.
     */
    void spiReadRegisters(uint8_t regAddr, uint8_t *pData, uint8_t len);
};

}  // namespace sensors

}  // namespace tap

#endif  // TAPROOT_MPU6500_HPP_
