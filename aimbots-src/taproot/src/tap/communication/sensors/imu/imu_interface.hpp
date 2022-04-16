/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_IMU_INTERFACE_HPP_
#define TAPROOT_IMU_INTERFACE_HPP_

namespace tap::communication::sensors::imu
{
/**
 * An interface for interacting with a 6 axis IMU.
 */
class ImuInterface
{
public:
    /**
     * Possible IMU states for an IMU.
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

    virtual inline const char *getName() const = 0;

    /**
     * Returns the linear acceleration in the x direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    virtual inline float getAx() = 0;

    /**
     * Returns the linear acceleration in the y direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    virtual inline float getAy() = 0;

    /**
     * Returns the linear acceleration in the z direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    virtual inline float getAz() = 0;

    /**
     * Returns the gyroscope reading (rotational speed) in the x direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    virtual inline float getGx() = 0;

    /**
     * Returns the gyroscope reading (rotational speed) in the y direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    virtual inline float getGy() = 0;

    /**
     * Returns the gyroscope reading (rotational speed) in the z direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    virtual inline float getGz() = 0;

    /**
     * Returns the temperature of the imu in degrees C.
     */
    virtual inline float getTemp() = 0;

    /**
     * Returns yaw angle. in degrees.
     */
    virtual inline float getYaw() = 0;

    /**
     * Returns pitch angle in degrees.
     */
    virtual inline float getPitch() = 0;

    /**
     * Returns roll angle in degrees.
     */
    virtual inline float getRoll() = 0;
};
}  // namespace tap::communication::sensors::imu

#endif  // IMU_INTERFACE_HPP_
