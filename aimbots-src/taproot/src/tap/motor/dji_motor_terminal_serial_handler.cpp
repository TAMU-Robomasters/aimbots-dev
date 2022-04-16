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

#include "dji_motor_terminal_serial_handler.hpp"

#include "tap/algorithms/strtok.hpp"
#include "tap/drivers.hpp"

#include "dji_motor_tx_handler.hpp"

namespace tap
{
namespace motor
{
constexpr char DjiMotorTerminalSerialHandler::HEADER[];
constexpr char DjiMotorTerminalSerialHandler::USAGE[];

void DjiMotorTerminalSerialHandler::terminalSerialStreamCallback(modm::IOStream& outputStream)
{
    printInfo(outputStream);
}

void DjiMotorTerminalSerialHandler::init() { drivers->terminalSerial.addHeader(HEADER, this); }

bool DjiMotorTerminalSerialHandler::terminalSerialCallback(
    char* inputLine,
    modm::IOStream& outputStream,
    bool streamingEnabled)
{
    char* arg;
    motorId = 0;
    canBusValid = false;
    motorIdValid = false;
    canBus = 0;
    printAll = false;
    while (
        (arg = strtokR(inputLine, communication::serial::TerminalSerial::DELIMITERS, &inputLine)))
    {
        if (strcmp(arg, "motor") == 0)
        {
            arg = strtokR(inputLine, communication::serial::TerminalSerial::DELIMITERS, &inputLine);
            if (arg == nullptr)
            {
                outputStream << "motorinfo: must specify motor id" << modm::endl;
                return false;
            }
            char* indexEnd;
            motorId = strtol(arg, &indexEnd, 10);
            if (indexEnd != arg + strlen(arg) ||
                motorId < (DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR1) + 1) ||
                motorId > (DJI_MOTOR_TO_NORMALIZED_ID(MotorId::MOTOR8) + 1))
            {
                outputStream << "motorinfo: Invalid motorID" << modm::endl << USAGE;
                return false;
            }
            motorId--;
            motorIdValid = true;
        }
        else if (strcmp(arg, "can") == 0)
        {
            arg = strtokR(inputLine, communication::serial::TerminalSerial::DELIMITERS, &inputLine);
            if (arg == nullptr)
            {
                outputStream << "motorinfo: must specify can bus" << modm::endl;
                return false;
            }
            char* indexEnd;
            canBus = strtol(arg, &indexEnd, 10);
            if (indexEnd != arg + strlen(arg) || (canBus != 1 && canBus != 2))
            {
                outputStream << "motorinfo: Invalid can bus ID" << modm::endl << USAGE;
                return false;
            }
            canBusValid = true;
        }
        else if (strcmp(arg, "all") == 0)
        {
            printAll = true;
            break;
        }
        else if (strcmp(arg, "-H") == 0)
        {
            outputStream << USAGE;
            // If streamingEnabled == true, we want to return false to indicate we shouldn't start
            // streaming. Also if any of the other inputs have been set, return false since the user
            // shouldn't specify -H and another argument.
            return !streamingEnabled && !canBusValid && !motorIdValid && !printAll &&
                   (*inputLine == '\0');
        }
        else
        {
            outputStream << USAGE;
            return false;
        }
    }

    if (((canBusValid || motorIdValid) && printAll) || (*inputLine != '\0'))
    {
        outputStream << USAGE;
        return false;
    }

    return printInfo(outputStream);
}

bool DjiMotorTerminalSerialHandler::printInfo(modm::IOStream& outputStream)
{
    if (printAll)
    {
        outputStream << "CAN 1:" << modm::endl;
        printAllMotorInfo(&DjiMotorTxHandler::getCan1Motor, outputStream);
        outputStream.flush();
        outputStream << "CAN 2:" << modm::endl;
        printAllMotorInfo(&DjiMotorTxHandler::getCan2Motor, outputStream);
    }
    else if (!canBusValid && !motorIdValid)
    {
        outputStream << USAGE;
        return false;
    }
    else if (canBusValid && !motorIdValid)
    {
        if (canBus == 1)
        {
            printAllMotorInfo(&DjiMotorTxHandler::getCan1Motor, outputStream);
        }
        else if (canBus == 2)
        {
            printAllMotorInfo(&DjiMotorTxHandler::getCan2Motor, outputStream);
        }
        else
        {
            outputStream << USAGE;
            return false;
        }
    }
    else if (!canBusValid && motorIdValid)
    {
        outputStream << "CAN 1:" << modm::endl;
        getMotorInfoToString(
            drivers->djiMotorTxHandler.getCan1Motor(
                static_cast<MotorId>(motorId + tap::motor::MOTOR1)),
            outputStream);
        outputStream << "CAN 2:" << modm::endl;
        getMotorInfoToString(
            drivers->djiMotorTxHandler.getCan2Motor(
                static_cast<MotorId>(motorId + tap::motor::MOTOR1)),
            outputStream);
    }
    else
    {
        if (canBus == 1)
        {
            getMotorInfoToString(
                drivers->djiMotorTxHandler.getCan1Motor(
                    static_cast<MotorId>(motorId + tap::motor::MOTOR1)),
                outputStream);
        }
        else
        {
            getMotorInfoToString(
                drivers->djiMotorTxHandler.getCan2Motor(
                    static_cast<MotorId>(motorId + tap::motor::MOTOR1)),
                outputStream);
        }
    }
    return true;
}

void DjiMotorTerminalSerialHandler::getMotorInfoToString(
    const DjiMotor* motor,
    modm::IOStream& outputStream)
{
    if (motor != nullptr)
    {
        outputStream << (DJI_MOTOR_TO_NORMALIZED_ID(motor->getMotorIdentifier()) + 1) << ". "
                     << motor->getName() << ": online: " << (motor->isMotorOnline() ? "yes" : "no")
                     << ", enc: " << motor->getEncoderWrapped() << ", rpm: " << motor->getShaftRPM()
                     << ", out des: " << motor->getOutputDesired() << modm::endl;
    }
}

void DjiMotorTerminalSerialHandler::printAllMotorInfo(
    getMotorByIdFunc func,
    modm::IOStream& outputStream)
{
    for (int i = static_cast<int>(MOTOR1); i <= static_cast<int>(MOTOR8); i++)
    {
        const DjiMotor* motor = (drivers->djiMotorTxHandler.*(func))(static_cast<MotorId>(i));
        getMotorInfoToString(motor, outputStream);
    }
}
}  // namespace motor
}  // namespace tap
