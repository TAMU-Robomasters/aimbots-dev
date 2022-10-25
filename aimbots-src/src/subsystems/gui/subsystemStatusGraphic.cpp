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

SubsystemStatusGraphic::SubsystemStatusGraphic(src::Drivers &drivers, tap::communication::serial::RefSerialTransmitter &refSerialTransmitter) :
GraphicHelper(refSerialTransmitter),
      drivers(&drivers) {

      }

modm::ResumableResult<bool> SubsystemStatusGraphic::sendInitialGraphics() {
    // not actually an error
    #ifdef __INTELLISENSE__
        #pragma diag_suppress 1227
    #endif
    RF_BEGIN(0);
    #ifdef __INTELLISENSE__
        #pragma diag_default 1227
    #endif

    //RF_RETURN_CALL(refSerialTransmitter.sendGraphic(&statusStaticGraphics[0]));
    // RF_END_RETURN_CALL();
    RF_CALL(refSerialTransmitter.sendGraphic(&statusStaticGraphics[0]));

    RF_END();
}

modm::ResumableResult<bool> SubsystemStatusGraphic::update() {
    // not actually an error
    RF_BEGIN(1);

    RF_CALL(refSerialTransmitter.sendGraphic(&statusStaticGraphics[0]));

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
        Tx::GraphicColor::PURPLISH_RED);

        //Specific graphic Configuration type
        RefSerialTransmitter::configCircle(/*Width*/20,
        /*X*/280,
        /*Y*/760,
        /*Radius*/15,
        &statusStaticGraphics[i].graphicData);
        
        //Couldn't get text on screen to work right, will revisit soon.
        
        /*
        //Gets a name and assigns it to graphicName
        getUnusedGraphicName(graphicName);

        //Sets up the basic information for a graphic, needs to be called on every single one
        RefSerialTransmitter::configGraphicGenerics(&statusStaticLabelGraphics[i].graphicData,
        graphicName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        Tx::GraphicColor::GREEN);

        //Specific graphic Configuration type
        RefSerialTransmitter::configCharacterMsg(Char Size20,
            Line Width11,
            X500,
            Y500,
            The message in graphic (limited to 30 chars)sampleMsg,
            &statusStaticLabelGraphics[i].graphicData);


        */
    }

}

}