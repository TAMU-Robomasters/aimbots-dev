#pragma once

#include <cstdint>
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

#define READ_BYTES(data, length) uartDriver.read(src::Jetson::JETSON_UART_PORT, data, length)
#define WRITE_BYTES(data, length) uartDriver.write(src::Jetson::JETSON_UART_PORT, data, length)
namespace src::Communication {

class MessageFromJetson {
    // all messages from the Jetson have the same header
    static constexpr uint8_t header = src::Jetson::jetsonMessageHeader; 
    uint8_t messageID;
    tap::communication::serial::Uart uartDriver;
public:
    MessageFromJetson(uint8_t messageID) : messageID(messageID) {}

    uint8_t getID() const { return messageID; }
    virtual void processMessage() = 0;
};

template <typename DataStruct>
class DataFromJetson : public MessageFromJetson {
    DataChannel<DataStruct> channel;
    alignas(DataStruct) uint8_t dataBytes[sizeof(DataStruct)];
public:
    DataFromJetson(uint8_t messageID) : MessageFromJetson(messageID) {}

    void processMessage() {
        READ_BYTES(dataBytes, sizeof(DataStruct));
        channel.setData(dataBytes);
    }
};

template <typename DataStruct>
class QueryFromJetson : public MessageFromJetson {
    DataStruct data;
public:
    QueryFromJetson(uint8_t messageID) : MessageFromJetson(messageID) {}

    virtual DataStruct fetchResponseData() = 0;
    void processMessage() {
        data = fetchResponseData();
        WRITE_BYTES((uint8_t*) &data, sizeof(DataStruct));
    }
};


} // namespace src::Informants::Vision

#endif // #ifdef JETSON_COMPATIBLE