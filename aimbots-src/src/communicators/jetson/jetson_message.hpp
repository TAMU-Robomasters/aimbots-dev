#pragma once

#include <cstdint>
#include "drivers.hpp"
#include "tap/communication/serial/uart.hpp"
#include "subsystems/jetson/jetson_constants.hpp"
#include "data_channel.hpp"

// TODO: decide if this can be further abstracted for any uart message
// TODO: add option for dynamic length messages

/* 
NOTE: do not like the idea of messages having a pointer to the uart driver
because the can potentially write or read at the same time
*/
// TODO: see if there's a better design solution to address the concern above

//TODO: think of better Namespace
#ifdef JETSON_COMPATIBLE

#define READ_BYTES(data, length) drivers->uart.read(src::Jetson::JETSON_UART_PORT, data, length)
#define WRITE_BYTES(data, length) drivers->uart.write(src::Jetson::JETSON_UART_PORT, data, length)
namespace src::Communication {

class MessageFromJetson {
protected:
    // all messages from the Jetson have the same header
    static constexpr uint8_t header = src::Jetson::jetsonMessageHeader; 
    src::Drivers* drivers;
    uint8_t messageID;
public:
    MessageFromJetson(src::Drivers* drivers, uint8_t messageID) 
        : drivers(drivers), messageID(messageID)
    {}

    uint8_t getID() const { return messageID; }
    virtual void processMessage() = 0;
};

template <typename DataStruct>
class DataFromJetson : public MessageFromJetson {
    DataChannel<DataStruct> channel;
    alignas(DataStruct) uint8_t dataBytes[sizeof(DataStruct)];
public:
    DataFromJetson(src::Drivers* drivers, uint8_t messageID) 
        : MessageFromJetson(drivers, messageID) 
    {}

    void processMessage() {
        READ_BYTES(dataBytes, sizeof(DataStruct));
        channel.setData(dataBytes);
    }
};

template <typename DataStruct>
class QueryFromJetson : public MessageFromJetson {
    static constexpr uint8_t devboardHeader = src::Jetson::devboardMessageHeader; // for sending
    DataStruct data;
public:
    QueryFromJetson(src::Drivers* drivers, uint8_t messageID) 
        : MessageFromJetson(drivers, messageID) 
    {}

    virtual DataStruct fetchData() = 0;
    void processMessage() {
        data = fetchData();
        WRITE_BYTES((uint8_t*) &devboardHeader, sizeof(devboardHeader));
        WRITE_BYTES((uint8_t*) &messageID, sizeof(messageID)); 
        WRITE_BYTES((uint8_t*) &data, sizeof(DataStruct));
    }
};


} // namespace src::Informants::Vision

#endif // #ifdef JETSON_COMPATIBLE