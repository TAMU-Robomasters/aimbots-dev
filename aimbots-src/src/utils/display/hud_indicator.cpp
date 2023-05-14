#include "hud_indicator.hpp"

#include <cstdint>

namespace src::utils::display {

uint32_t HudIndicator::currGraphicName = 0;

HudIndicator::HudIndicator(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter) : refSerialTransmitter(refSerialTransmitter) {}

void HudIndicator::resetGraphicNameGenerator() { currGraphicName = 0; }

void HudIndicator::getUnusedGraphicName(uint8_t graphicName[3]) {
    if (currGraphicName > 0xffffff) {
        return;
    } else {
        graphicName[0] = static_cast<uint8_t>((currGraphicName >> 16) & 0xff);
        graphicName[1] = static_cast<uint8_t>((currGraphicName >> 8) & 0xff);
        graphicName[2] = static_cast<uint8_t>(currGraphicName & 0xff);
        currGraphicName++;
    }
}

}  // namespace utils::display