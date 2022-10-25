#include "gui_display_command.hpp"
#include "gui_display.hpp"
#include "drivers.hpp"

namespace src::GUI {

GUI_DisplayCommand::GUI_DisplayCommand(src::Drivers &drivers,GUI_DisplaySubsystem &GUI_Display) :
Command(), drivers(drivers), refSerialTransmitter(&drivers), subsystemStatGraphics(drivers, refSerialTransmitter) {
    
    

    addSubsystemRequirement(&GUI_Display);

    
}

//GUI command just runs a protothread (lightweight multithread) to allow for all graphic writing code to be run.
void GUI_DisplayCommand::initialize() {
    
    restart();
    
    subsystemStatGraphics.initialize();
    //Initialize other drawing commands here
}

void GUI_DisplayCommand::execute() {
    run();
}

bool GUI_DisplayCommand::run() {
    //Intentional error, ignore
    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());

    PT_CALL(subsystemStatGraphics.sendInitialGraphics());

    while (true)
    {
        PT_CALL(subsystemStatGraphics.update());
        PT_YIELD();
    }
    PT_END();

}


}