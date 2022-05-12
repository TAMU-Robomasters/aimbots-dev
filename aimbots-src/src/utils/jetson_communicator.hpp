#pragma once

#include <tap/architecture/timeout.hpp>
#include <tap/util_macros.hpp>

#include <utils/common_types.hpp>
#include <utils/jetson_protocol.hpp>

namespace src { class Drivers; }

namespace utils {

enum class JetsonCommunicatorSerialState : uint8_t {
    SearchingForMagic = 0,
    AssemblingMessage,
};

class JetsonCommunicator {
   public:
    JetsonCommunicator(src::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(JetsonCommunicator);
    ~JetsonCommunicator() = default;

    void initialize();
    void updateSerial();

    inline bool isJetsonOnline() const { return !jetsonOfflineTimeout.isExpired(); }

    inline JetsonMessage const& lastValidMessage() const { return lastMessage; }

   private:
    src::Drivers* drivers;

    uint8_t       rawSerialBuffer[sizeof(JetsonMessage)];
    JetsonMessage lastMessage;

    JetsonCommunicatorSerialState currentSerialState;
    size_t                        nextByteIndex;

    tap::arch::MilliTimeout jetsonOfflineTimeout;

    static constexpr uint32_t JETSON_BAUD_RATE = 115200;
    static constexpr uint16_t JETSON_OFFLINE_TIMEOUT_MILLISECONDS = 5000;
    static constexpr UartPort JETSON_UART_PORT = UartPort::Uart1;
};

}