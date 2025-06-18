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

#ifndef TAPROOT_DJI_MOTOR_TERMINAL_SERIAL_HANDLER_HPP_
#define TAPROOT_DJI_MOTOR_TERMINAL_SERIAL_HANDLER_HPP_

#include "tap/communication/serial/terminal_serial.hpp"
#include "tap/util_macros.hpp"

#include "dji_motor.hpp"

namespace tap
{
class Drivers;
namespace motor
{
class DjiMotorTxHandler;
class DjiMotorTerminalSerialHandler : public communication::serial::TerminalSerialCallbackInterface
{
public:
    static constexpr char HEADER[] = "motorinfo";

    DjiMotorTerminalSerialHandler(Drivers* drivers) : drivers(drivers) {}

    mockable void init();

    bool terminalSerialCallback(
        char* inputLine,
        modm::IOStream& outputStream,
        bool streamingEnabled) override;

    void terminalSerialStreamCallback(modm::IOStream& outputStream) override;

private:
    typedef DjiMotor const* (DjiMotorTxHandler::*getMotorByIdFunc)(MotorId);

    static constexpr char USAGE[] =
        "Usage: motorinfo <[-H] | [all] | [motor [mid]] [can [cid]]>\n"
        "  Where:\n"
        "    - [-H]  prints usage\n"
        "    - [all] prints all motor info\n"
        "    Or specifiy a motor id and/or can id, where\n"
        "    - [mid] is the id of a motor, in [1, 8]\n"
        "    - [cid] is some the can id, in [1, 2]\n";

    Drivers* drivers;

    bool motorIdValid = false;
    uint32_t motorId = 0;
    bool canBusValid = false;
    int canBus = 0;
    bool printAll = false;

    bool printInfo(modm::IOStream& outputStream);

    void getMotorInfoToString(const DjiMotor* motor, modm::IOStream& outputStream);

    void printAllMotorInfo(getMotorByIdFunc func, modm::IOStream& outputStream);
};  // class DjiMotorTerminalSerialHandler
}  // namespace motor
}  // namespace tap

#endif  // TAPROOT_DJI_MOTOR_TERMINAL_SERIAL_HANDLER_HPP_
