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

#ifndef TAPROOT_IMU_TERMINAL_SERIAL_HANDLER_HPP_
#define TAPROOT_IMU_TERMINAL_SERIAL_HANDLER_HPP_

#include "tap/communication/serial/terminal_serial.hpp"
#include "tap/util_macros.hpp"

#include "modm/architecture/interface/register.hpp"

#include "imu_interface.hpp"

namespace tap
{
class Drivers;
}

namespace tap::communication::sensors::imu
{
/**
 * Interface for reading IMU data. Connects to the terminal serial driver and allows
 * the user to query gyro, accel, angle, and temperature data. Single query and streaming
 * modes supported.
 */
class ImuTerminalSerialHandler : public communication::serial::TerminalSerialCallbackInterface
{
public:
    ImuTerminalSerialHandler(Drivers* drivers, ImuInterface* imu)
        : drivers(drivers),
          imu(imu),
          subjectsBeingInspected(0)
    {
    }

    mockable void init();

    bool terminalSerialCallback(
        char* inputLine,
        modm::IOStream& outputStream,
        bool streamingEnabled) override;

    void terminalSerialStreamCallback(modm::IOStream& outputStream) override;

private:
    /** Usage without the name since this is dependent on the IMU. */
    static constexpr char USAGE[] =
        " [-h] [angle] [gyro] [accel] [temp]\n"
        "  Where:\n"
        "    - [-h] Prints usage\n"
        "    - [angle] Prints angle data\n"
        "    - [gyro] Prints gyro data\n"
        "    - [accel] Prints accel data\n"
        "    - [temp] Prints temp data\n";

    Drivers* drivers;

    ImuInterface* imu;

    /**
     * Each element in the enum corresponds to a sensor that a user may query.
     * Each enum value is a bit mask that when set in a bit map indicates
     * the sensor data will be printed to the screen.
     */
    enum InspectSubject : uint8_t
    {
        ANGLES = 1,
        GYRO = 1 << 1,
        ACCEL = 1 << 2,
        TEMP = 1 << 3
    };

    MODM_FLAGS8(InspectSubject);

    /**
     * Contains which elements are being inspected (one, none, or a combination
     * of `InspectSubject`) using bit masking techniques.
     *
     * For example, if `ANGLES` and `GYRO` are being inspected, `subjectsBeingInspected`
     * is equal to `ANGLES | GYRO`.
     */
    InspectSubject_t subjectsBeingInspected;

    void printHeader(modm::IOStream& outputStream);
};  // class ImuTerminalSerialHandler
}  // namespace tap::communication::sensors::imu

#endif  // TAPROOT_MPU6500_TERMINAL_SERIAL_HANDLER_HPP_
