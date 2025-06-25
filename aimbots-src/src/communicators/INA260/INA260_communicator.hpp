#pragma once

#ifndef ALL_SENTRIES

#include <tap/architecture/timeout.hpp>
#include <tap/util_macros.hpp>
#include <utils/tools/common_types.hpp>


#include "INA260_protocol.hpp"

namespace src {
class Drivers;
}

namespace src::Informants::INA260 {

enum class INA260CommunicatorSerialState : uint8_t {
    SearchingForMagic = 0,
    AssemblingMessage,
};

class INA260Communicator{
public:
    INA260Communicator(src::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(INA260Communicator);
    ~INA260Communicator() = default;

    void initialize();
    void updateSerial();

    inline bool isINA260Online() const { return !INA260OfflineTimeout.isExpired(); }

    INA260Message const& getLastValidMessage() const;

private:
    src::Drivers* drivers;

    alignas(INA260Message) uint8_t rawSerialBuffer[sizeof(INA260Message)];

    INA260CommunicatorSerialState currentSerialState;
    size_t nextByteIndex;

    tap::arch::MilliTimeout INA260OfflineTimeout;

    static constexpr uint32_t INA260_BAUD_RATE = 115200;
    static constexpr uint16_t INA260_OFFLINE_TIMEOUT_MILLISECONDS = 2000;
    static constexpr UartPort INA260_UART_PORT = UartPort::Uart1;

    INA260Message lastMessage;
};
}  // namespace src::Informants::INA260
#endif // ALL_SENTRIES