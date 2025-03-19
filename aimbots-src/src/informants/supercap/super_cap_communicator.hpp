#pragma once

#include <tap/architecture/timeout.hpp>
#include <tap/util_macros.hpp>
#include <utils/common_types.hpp>

// #include "informants/enemy_data_conversion.hpp"

#include "super_cap_protocol.hpp"

namespace src {
class Drivers;
}

namespace src::Informants::SuperCap {

enum class SuperCapCommunicatorSerialState : uint8_t {
    SearchingForMagic = 0,
    AssemblingMessage,
};

class SuperCapCommunicator {
public:
    SuperCapCommunicator(src::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(SuperCapCommunicator);
    ~SuperCapCommunicator() = default;

    void initialize();
    void updateSerial();

    inline bool isSuperCapOnline() const { return !superCapOfflineTimeout.isExpired(); }

    inline SuperCapMessageRecieved const& getLastValidMessage() const { return lastMessage; }

    inline void setCommand(SuperCapCommand command) { this->command = command; }

    inline std::string makeChargeMessage(int x) const { return ("CHARGE " + std::to_string(x) + "\n"); }
    inline void setChargeValue(uint8_t x) { chargeValue = x; }

private:
    src::Drivers* drivers;
    // src::Informants::Vision::VisionDataConversion visionDataConverter;

    alignas(SuperCapMessageRecieved) uint8_t rawSerialBuffer[sizeof(SuperCapMessageRecieved)];
    uint8_t* rawSerialBufferSent;

    SuperCapCommunicatorSerialState currentSerialState;
    size_t nextByteIndex;

    // applicable to supercap?
    tap::arch::MilliTimeout superCapOfflineTimeout;

    static constexpr uint32_t SUPER_CAP_BAUD_RATE = 115200;
    static constexpr uint16_t SUPER_CAP_OFFLINE_TIMEOUT_MILLISECONDS = 2000;
    static constexpr UartPort SUPER_CAP_UART_PORT = UartPort::Uart1;

    SuperCapMessageRecieved lastMessage;
    SuperCapMessageSent lastSentMessage;
    uint32_t lastUpdate;
    SuperCapCommand command;
    uint8_t chargeValue;

    static constexpr char SUPER_CAP_DISCHARGE_MSG[] = "DISCHARGE\n";
};
}  // namespace src::Informants::SuperCap