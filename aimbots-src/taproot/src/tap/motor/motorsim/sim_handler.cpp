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

#ifdef PLATFORM_HOSTED

#include "sim_handler.hpp"

#include "modm/architecture/interface/can_message.hpp"

#include "can_serializer.hpp"

namespace tap
{
namespace motorsim
{
/* Singleton Class Variables */
std::array<
    std::array<MotorSim*, tap::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN>,
    SimHandler::CAN_BUSSES>
    SimHandler::sims;

std::array<uint8_t, SimHandler::CAN_BUSSES> SimHandler::nextCanSendIndex;

SimHandler::SimHandler() { resetMotorSims(); }

SimHandler::~SimHandler()
{
    // Loop through all registered sims and destroy them.
    for (size_t i = 0; i < sims.size(); i++)
    {
        for (size_t j = 0; j < sims[i].size(); j++)
        {
            if (sims[i][j] != nullptr)
            {
                delete sims[i][j];
                sims[i][j] = nullptr;
            }
        }
    }
}

void SimHandler::resetMotorSims()
{
    // for-loop used in case additonal Can busses are utilized
    for (uint32_t i = 0; i < nextCanSendIndex.size(); i++)
    {
        nextCanSendIndex[i] = 0;
    }
    for (uint32_t i = 0; i < sims.size(); i++)
    {
        for (uint32_t j = 0; j < sims[i].size(); j++)
        {
            if (sims[i][j] != nullptr)
            {
                sims[i][j]->reset();
            }
        }
    }
}
void SimHandler::registerSim(
    MotorSim::MotorType type,
    tap::can::CanBus bus,
    tap::motor::MotorId id,
    float loading)
{
    int8_t port = CanSerializer::idToPort(id);
    int8_t busIndex = static_cast<uint8_t>(bus);

    if (sims[busIndex][port] != nullptr)
    {
        printf("Warning: overwriting sim in SimHandler.cpp:registerSim()");
        delete sims[busIndex][port];
    }

    sims[busIndex][port] = new motorsim::MotorSim(type, loading);
}

bool SimHandler::readyToSend(tap::can::CanBus bus)
{
    uint8_t busIndex = static_cast<uint8_t>(bus);
    if (sims[busIndex][nextCanSendIndex[busIndex]] == nullptr)
    {
        nextCanSendIndex[busIndex] =
            (nextCanSendIndex[busIndex] + 1) % tap::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN;
        return false;
    }
    return true;
}

bool SimHandler::getMessage(tap::can::CanBus bus, const modm::can::Message& message)
{
    std::array<int16_t, 4> newInputs = CanSerializer::parseMessage(&message);
    uint8_t busIndex = static_cast<uint8_t>(bus);

    if (message.getIdentifier() == 0x1FF)
    {
        for (int i = 0; i < (tap::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN / 2); i++)
        {
            if (sims[busIndex][i] != nullptr)
            {
                sims[busIndex][i]->setInput(newInputs[i]);
            }
        }
    }
    else if (message.getIdentifier() == 0x200)
    {
        for (int i = (tap::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN / 2);
             i < tap::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN;
             i++)
        {
            if (sims[busIndex][i] != nullptr)
            {
                sims[busIndex][i]->setInput(newInputs[i]);
            }
        }
    }
    else
    {
        return false;
    }
    return true;
}
bool SimHandler::sendMessage(tap::can::CanBus bus, modm::can::Message* message)
{
    uint8_t busIndex = static_cast<uint8_t>(bus);

    // Check to make sure sim actually exists.
    nextCanSendIndex[busIndex] = (nextCanSendIndex[busIndex] + 1) % INDEX_LAST_PORT;

    if (sims[busIndex][nextCanSendIndex[busIndex]] == nullptr)
    {
        return false;
    }

    *message = CanSerializer::serializeFeedback(
        sims[busIndex][nextCanSendIndex[busIndex]]->getEnc(),
        sims[busIndex][nextCanSendIndex[busIndex]]->getRPM(),
        sims[busIndex][nextCanSendIndex[busIndex]]->getInput(),
        nextCanSendIndex[busIndex]);

    return true;
}

void SimHandler::updateSims()
{
    for (uint32_t i = 0; i < sims.size(); i++)
    {
        for (uint32_t j = 0; j < sims[0].size(); j++)
        {
            if (sims[i][j] != nullptr)
            {
                sims[i][j]->update();
            }
        }
    }
}

}  // namespace motorsim

}  // namespace tap

#endif  // PLATFORM_HOSTED
