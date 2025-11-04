#include "gimbal_angle_query.hpp"
#include "communicators/jetson/jetson_message.hpp"

#ifdef GIMBAL_COMPATIBLE

using namespace src::Communication;

GimbalQuery::GimbalQuery(src::Drivers* drivers, uint8_t messageID)
    : QueryFromJetson<GimbalMessage>(drivers, messageID)
{}

GimbalMessage GimbalQuery::fetchData() {
    return GimbalMessage(20.0f, 20.0f);
}

#endif // #ifdef GIMBAL_COMPATIBLE