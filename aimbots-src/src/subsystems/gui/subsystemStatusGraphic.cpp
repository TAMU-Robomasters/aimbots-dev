#include "subsystemStatusGraphic.hpp"

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "drivers.hpp"
// #ifdef __INTELLISENSE__
//     #pragma diag_suppress 1227
// #endif

using namespace tap::communication::serial;
using namespace tap::communication::referee;

namespace src::GUI {

SubsystemStatusGraphic::SubsystemStatusGraphic(src::Drivers* drivers, tap::communication::serial::RefSerialTransmitter &refSerialTransmitter) :
GraphicHelper(refSerialTransmitter),
      drivers(drivers) {

      }

modm::ResumableResult<bool> SubsystemStatusGraphic::sendInitialGraphics() {
    // not actually an error
    RF_BEGIN(0);

    RF_CALL(refSerialTransmitter.sendGraphic(&statusStaticGraphics[0]));
    //RF_CALL(refSerialTransmitter.sendGraphic(&statusStaticGraphics[1]));

    RF_CALL(refSerialTransmitter.sendGraphic(&statusStaticLabelGraphics[0]));
    //RF_CALL(refSerialTransmitter.sendGraphic(&statusStaticLabelGraphics[1]));

    RF_END();
}

modm::ResumableResult<bool> SubsystemStatusGraphic::update() {
    // not actually an error
    RF_BEGIN(1);

    //RF_CALL(statusStaticGraphics[0].draw());

    RF_END();
}

void SubsystemStatusGraphic::initialize() {
    uint8_t graphicName[3] = {};
    
    for (int i = 0; i < 2; i++) {
        //Gets a name and assigns it to graphicName
        getUnusedGraphicName(graphicName);

        //Sets up the basic information for a graphic, needs to be called on every single one
        RefSerialTransmitter::configGraphicGenerics(&statusStaticGraphics[i].graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);

        //Specific graphic Configuration type
        RefSerialTransmitter::configCircle(/*Width*/40,
        /*X*/280,
        /*Y*/760,
        /*Radius*/125,
        &statusStaticGraphics[i].graphicData);    
        
        //Gets a name and assigns it to graphicName
        getUnusedGraphicName(graphicName);

        //Sets up the basic information for a graphic, needs to be called on every single one
        RefSerialTransmitter::configGraphicGenerics(&statusStaticLabelGraphics[i].graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);

        //Specific graphic Configuration type
        //Char Size, Line Width, X, Y, Message (30 chars)
        RefSerialTransmitter::configCharacterMsg(40,
            50,
            900,
            760,
            sampleMsg,
            &statusStaticLabelGraphics[i]);


        
    }

}

}