#pragma once

#include <tap/architecture/timeout.hpp>
#include <tap/drivers.hpp>
#include <tap/util_macros.hpp>

#include <utils/common_types.hpp>
#include <utils/jetson_protocol.hpp>

class JetsonCommunicator {
public:
    JetsonCommunicator(tap::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(JetsonCommunicator);
    ~JetsonCommunicator() = default;

    void initialize();
    void updateSerial();

    inline bool isJetsonOnline() const { return !m_jetsonOfflineTimeout.isExpired(); }

    inline bool isLastDataValid() const { return m_isLastDataValid; }
    inline ChassisPacket const& lastChassisMessage() const { return m_lastChassisMessage; }
    inline GimbalPacket const& lastGimbalMessage() const { return m_lastGimbalMessage; }

private:
    tap::Drivers* m_drivers;

    uint8_t m_rawSerialBuffer[sizeof(JetsonMessage)];

    bool          m_isLastDataValid;
    ChassisPacket m_lastChassisMessage;
    GimbalPacket  m_lastGimbalMessage;

    tap::arch::MilliTimeout m_jetsonOfflineTimeout;

    static constexpr uint16_t JETSON_OFFLINE_TIMEOUT_MILLISECONDS = 5000;
    static constexpr TapUartPort JETSON_UART_PORT = TapUartPort::Uart1;
};