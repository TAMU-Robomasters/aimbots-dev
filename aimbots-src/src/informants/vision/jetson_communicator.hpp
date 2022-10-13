#pragma once

#include <tap/algorithms/contiguous_float.hpp>
#include <tap/architecture/timeout.hpp>
#include <tap/util_macros.hpp>
#include <utils/common_types.hpp>

#include "subsystems/gimbal/gimbal.hpp"

#include "jetson_protocol.hpp"

namespace src {
class Drivers;
}

namespace src::Informants::vision {

enum class JetsonCommunicatorSerialState : uint8_t {
    SearchingForMagic = 0,
    AssemblingMessage,
};

enum GimbalAxis {
    yaw = 0,
    pitch = 1,
};

class JetsonCommunicator {
public:
    JetsonCommunicator(src::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(JetsonCommunicator);
    ~JetsonCommunicator() = default;

    void initialize();
    void updateSerial();

    inline bool isJetsonOnline() const { return !jetsonOfflineTimeout.isExpired(); }

    inline JetsonMessage const& getLastValidMessage() const { return lastMessage; }

    void setGimbalSubsystem(src::Gimbal::GimbalSubsystem* gimbal) { this->gimbal = gimbal; }

    Matrix<float, 1, 2> const& getVisionTargetAngles() { return visionTargetAngles; }

private:
    src::Drivers* drivers;

    alignas(JetsonMessage) uint8_t rawSerialBuffer[sizeof(JetsonMessage)];

    JetsonMessage lastMessage;

    JetsonCommunicatorSerialState currentSerialState;
    size_t nextByteIndex;

    tap::arch::MilliTimeout jetsonOfflineTimeout;

    src::Gimbal::GimbalSubsystem* gimbal;

    float fieldRelativeYawAngleAtVisionUpdate;
    float chassisRelativePitchAngleAtVisionUpdate;

    Matrix<float, 1, 2> visionTargetAngles;

    static constexpr uint32_t JETSON_BAUD_RATE = 115200;
    static constexpr uint16_t JETSON_OFFLINE_TIMEOUT_MILLISECONDS = 2000;
    static constexpr UartPort JETSON_UART_PORT = UartPort::Uart1;
};
}  // namespace src::Informants::vision