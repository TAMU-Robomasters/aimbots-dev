#include "jetson_communicator.hpp"

#define READ(data, length) m_drivers->uart.read(JETSON_UART_PORT, data, length)

JetsonCommunicator::JetsonCommunicator(tap::Drivers* drivers)
    : m_drivers(drivers),
      m_isLastDataValid(false),
      m_lastChassisMessage(),
      m_lastGimbalMessage(),
      m_jetsonOfflineTimeout() { }

void JetsonCommunicator::initialize() {
    m_jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);
    m_drivers->uart.init<JETSON_UART_PORT, 115200>();
}

void JetsonCommunicator::updateSerial() {
    size_t bytesRead = READ(&m_rawSerialBuffer[0], sizeof(JetsonMessage));

    if(bytesRead != sizeof(JetsonMessage)) {
        // TODO: Possibly propagate this error? It's probably not necessary, so
        //       for now, let's just silently fail.
        return;
    }

    m_jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);

    // TODO: Address possible endianness issues with CV team.
    JetsonMessage* message = (JetsonMessage*)&m_rawSerialBuffer;

    m_isLastDataValid = false;

    m_lastChassisMessage.cvState         = message->cvState;
    m_lastChassisMessage.forwardSpeed    = message->forwardSpeed;
    m_lastChassisMessage.horizontalSpeed = message->horizontalSpeed;

    m_lastGimbalMessage.cvState           = message->cvState;
    m_lastGimbalMessage.yaw               = message->yaw;
    m_lastGimbalMessage.pitch             = message->pitch;
    m_lastGimbalMessage.targetYawOffset   = message->targetYawOffset;
    m_lastGimbalMessage.targetPitchOffset = message->targetPitchOffset;

    m_isLastDataValid = true;
}