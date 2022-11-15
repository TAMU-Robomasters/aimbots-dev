#pragma once

#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "modm/processing/resumable.hpp"

namespace tap::communication::serial
{
class RefSerialTransmitter;
}

namespace src::GUI
{

//Helper subclass for several basic GUI graphics functions
class GraphicHelper : protected tap::communication::serial::RefSerialData
{
public:

    static constexpr uint16_t SCREEN_WIDTH = 1920; //Right is positive X
    static constexpr uint16_t SCREEN_HEIGHT = 1080; //Top is positive Y

    static constexpr uint8_t DEFAULT_GRAPHIC_LAYER = 0;

    GraphicHelper(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    virtual modm::ResumableResult<bool> sendInitialGraphics() = 0;

    virtual modm::ResumableResult<bool> update() = 0;

    virtual void initialize() = 0;

    static void resetGraphicNameGenerator();

protected:
    //Graphics must have a unique 3 byte identifier.  Utility function gets an unused name to give to the graphic
    static void getUnusedGraphicName(uint8_t graphicName[3]);

    static uint32_t currGraphicName;

    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter;

    //Delay between GUI update messages 
    tap::arch::MilliTimeout delayTimer;
};
}
