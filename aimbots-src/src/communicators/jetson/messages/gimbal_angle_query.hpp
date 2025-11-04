#pragma once

#include "drivers.hpp"
#include "communicators/jetson/jetson_message.hpp"

#ifdef JETSON_COMPATIBLE

namespace src::Communication {

struct GimbalMessage {
    float yawDegrees;
    float pitchDegress;
} __attribute__((packed)); // tell compiler to not adding padding

class GimbalQuery : public QueryFromJetson<GimbalMessage> {
public:
    GimbalQuery(src::Drivers* drivers, uint8_t messageID);
    virtual GimbalMessage fetchData() override;
};

} // namespace src::Communication

#endif // #ifdef JETSON_COMPATIBLE