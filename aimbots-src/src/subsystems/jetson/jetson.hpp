#pragma once
#include <array>
#include "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "communicators/jetson/jetson_message_handler.hpp"
#include "jetson_constants.hpp"

#ifdef JETSON_COMPATIBLE

namespace src::Jetson {

template <size_t messageCount>
class JetsonSubsystem : public tap::control::Subsystem {
public:
    JetsonSubsystem(
        src::Drivers* drivers, 
        std::array<src::Communication::MessageFromJetson* const, messageCount> messages) 
        : tap::control::Subsystem(drivers),
          messageHandler(drivers, messages) //TODO: rethink this dependency injection
    {}
    ~JetsonSubsystem() = default;

    void initialize() {
        INIT_UART();
    }
    void refresh() {
        messageHandler.checkForMessage();
    }

    const char* getName() const override { return "Jetson Subsystem"; }
private:
    src::Communication::JetsonMessageHandler<messageCount> messageHandler;
};
    
} // namespace src::Jetson

#endif // #ifdef JETSON_COMPATIBLE

