#include "graphic_helper.hpp"

#include <cstdint>

namespace src::GUI
{
uint32_t GraphicHelper::currGraphicName = 0;

GraphicHelper::GraphicHelper(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter)
    : refSerialTransmitter(refSerialTransmitter)
{
}
//d:\RoboMasters\Codebases\aimbots-dev\aimbots-src\taproot\src\tap\communication\serial\ref_serial_transmitter.cpp
//Saving for reference later
void GraphicHelper::resetGraphicNameGenerator() { currGraphicName = 0; }

void GraphicHelper::getUnusedGraphicName(uint8_t graphicName[3])
{
    if (currGraphicName > 0xffffff)
    {
        return;
    }
    else
    {
        graphicName[0] = static_cast<uint8_t>((currGraphicName >> 16) & 0xff);
        graphicName[1] = static_cast<uint8_t>((currGraphicName >> 8) & 0xff);
        graphicName[2] = static_cast<uint8_t>(currGraphicName & 0xff);
        currGraphicName++;
    }
}

}