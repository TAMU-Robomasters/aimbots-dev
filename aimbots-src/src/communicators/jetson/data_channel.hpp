#pragma once
#include <tap/architecture/clock.hpp>
#include <cstring>

#ifdef JETSON_COMPATIBLE 

namespace src::Communication {

template <typename DataStruct>
class DataChannel {
    float updateTimeStamp = 0.0f;
    DataStruct data;
public:
    bool isNotStale(uint32_t staleTimeoutMs) {
        uint32_t timeSinceLastUpdate = tap::arch::clock::getTimeMilliseconds() - updateTimeStamp;
        if (timeSinceLastUpdate < staleTimeoutMs) {
            return true;
        }
        return false;
    }
    void setData(uint8_t bytes[]) {
        std::memcpy(&data, bytes, sizeof(DataStruct));
        updateTimeStamp = tap::arch::clock::getTimeMilliseconds();
    }
    const DataStruct& getData() const {return data;}
};

} // namespace src::Informants::Vision

#endif // #ifdef JETSON_COMPATIBLE 