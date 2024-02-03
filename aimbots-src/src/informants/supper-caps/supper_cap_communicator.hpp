#pragma once

#include <tap/architecture/timeout.hpp>
#include <tap/util_macros.hpp>
#include <utils/common_types.hpp>

// #include "informants/enemy_data_conversion.hpp"

#include "supper_cap_protocol.hpp"

namespace src {
class Drivers;
}

namespace src::Informants::SupperCap {

enum class SupperCapCommunicatorSerialState : uint8_t {
    SearchingForMagic = 0,
    AssemblingMessage,
};

class SupperCapCommunicator {
public:
    SupperCapCommunicator(src::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(SupperCapCommunicator);
    ~SupperCapCommunicator() = default;

    void initialize();
    void updateSerial();

    inline bool isSupperCapOnline() const { return !supperCapOfflineTimeout.isExpired(); }

    inline SupperCapMessageRecieved const& getLastValidMessage() const { return lastMessage; }

    inline void setCommand(SupperCapCommand command) { this->command = command; }

    inline std::string makeChargeMessage(int x) const { return ("CHARGE " + std::to_string(x) + "\n"); }
    inline void setChargeValue(uint8_t x) { chargeValue = x; }
private:
    src::Drivers* drivers;
    // src::Informants::Vision::VisionDataConversion visionDataConverter;

    alignas(SupperCapMessageRecieved) uint8_t rawSerialBuffer[sizeof(SupperCapMessageRecieved)];
    alignas(SupperCapMessageSent) uint8_t rawSerialBufferSent[sizeof(SupperCapMessageSent)];

    SupperCapCommunicatorSerialState currentSerialState;
    size_t nextByteIndex;

    // applicable to supercap?
    tap::arch::MilliTimeout supperCapOfflineTimeout;

    static constexpr uint32_t SUPPER_CAP_BAUD_RATE = 115200;
    static constexpr uint16_t SUPPER_CAP_OFFLINE_TIMEOUT_MILLISECONDS = 2000;
    static constexpr UartPort SUPPER_CAP_UART_PORT = UartPort::Uart6;

    SupperCapMessageRecieved lastMessage;
    SupperCapMessageSent lastSentMessage;
    uint32_t lastUpdate;
    SupperCapCommand command;
    uint8_t chargeValue;

    static constexpr char SUPPER_CAP_DISCHARGE_MSG[] = "DISCHARGE\n";
};
}  // namespace src::Informants::SupperCap