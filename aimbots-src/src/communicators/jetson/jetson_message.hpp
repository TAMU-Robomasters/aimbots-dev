#pragma once

#include <cstdint>
#include <subsystems/jetson/jetson_constants.hpp>
#include "data_channel.hpp"

#ifdef JETSON_COMPATIBLE
// TODO: decide if this can be further abstracted for any uart message
// TODO: add option for dynamic length messages

//TODO: think of better Namespace
namespace src::Communication {

class MessageFromJetson {
    // all messages from the Jetson have the same header
    static constexpr uint8_t header = jetsonMessageHeader; 
    uint8_t messageID;

public:
    MessageFromJetson(uint8_t messageID) : messageID(messageID) {}

    uint8_t getID() {return messageID;}
    virtual void processMessage() = 0;
};

template <typename DataStruct>
class DataFromJetson : public MessageFromJetson {
    DataChannel<DataStruct> channel;
    alignas(DataStruct) uint8_t dataBytes[sizeof(DataStruct)];
public:
    DataFromJetson(uint8_t messageID) : MessageFromJetson(messageID) {}

    void processMessage {
        READ(dataBytes, sizeof(DataStruct));
        channel.setData(dataBytes);
    }
};

template <typename DataStruct>
class QueryFromJetson : public MessageFromJetson {
public:
    QueryFromJetson(uint8_t messageID) : messageID(messageID) {}

    virtual DataStruct fetchResponseData() = 0;
    DataStruct data;
    void processMessage {
        data = fetchResponseData();
        WRITE((uint8_t*) &data, sizeof(DataStruct));
    }
};


} // namespace src::Informants::Vision

#endif //#ifdef JETSON_COMPATIBLE