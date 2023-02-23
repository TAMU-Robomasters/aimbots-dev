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

struct plateKinematicState {
    Vector3f position;
    Vector3f velocity;
    Vector3f acceleration;
    float timestamp_uS;  // time that 'best guess' was made
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

    plateKinematicState getPlateKinematicState() const { return lastPlateKinematicState; }

    // What is this???
    Matrix<float, 1, 2> const& getVisionTargetAngles() { return visionTargetAngles; }

    // Doesn't yet actually do anything...
    Vector3f const& getVisionTargetPosition() { return visionTargetPosition; }

    uint32_t getLastFoundTargetTime() const { return lastFoundTargetTime; }

private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbal;

    alignas(JetsonMessage) uint8_t rawSerialBuffer[sizeof(JetsonMessage)];

    JetsonCommunicatorSerialState currentSerialState;
    size_t nextByteIndex;

    tap::arch::MilliTimeout jetsonOfflineTimeout;

    static constexpr uint32_t JETSON_BAUD_RATE = 115200;
    static constexpr uint16_t JETSON_OFFLINE_TIMEOUT_MILLISECONDS = 2000;
    static constexpr UartPort JETSON_UART_PORT = UartPort::Uart1;

    JetsonMessage lastMessage;
    uint32_t lastFoundTargetTime;

    float fieldRelativeYawAngleAtVisionUpdate;
    float chassisRelativePitchAngleAtVisionUpdate;

    plateKinematicState lastPlateKinematicState;

    Matrix<float, 1, 2> visionTargetAngles;
    Vector3f visionTargetPosition;
};
}  // namespace src::Informants::vision