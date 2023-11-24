#pragma once

#include <tap/architecture/timeout.hpp>
#include <tap/util_macros.hpp>
#include <utils/common_types.hpp>

#include "informants/enemy_data_conversion.hpp"

#include "jetson_protocol.hpp"

namespace src {
class Drivers;
}

namespace src::Informants::Vision {

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

    inline JetsonMessageReceive const& getLastValidMessage() const { return lastMessage; }

    PlateKinematicState getPlatePrediction(uint32_t dt) const;

    uint32_t getLastFoundTargetTime() const { return lastFoundTargetTime; }

    uint32_t getLastFrameCaptureDelay() const { return visionDataConverter.getLastFrameCaptureDelay(); }

    bool isLastFrameStale() const;

    inline void setCVDectionType(CVDetectionType detectionType) { this->detectionType = detectionType; }
    inline CVDetectionType getCVDectionType() const { return detectionType; }

private:
    src::Drivers* drivers;
    src::Informants::Vision::VisionDataConversion visionDataConverter;

    alignas(JetsonMessageReceive) uint8_t rawSerialBufferRecieved[sizeof(JetsonMessageReceive)];

    JetsonCommunicatorSerialState currentSerialState;
    size_t nextByteIndex;

    tap::arch::MilliTimeout jetsonOfflineTimeout;

    static constexpr uint32_t JETSON_BAUD_RATE = 115200;
    static constexpr uint16_t JETSON_OFFLINE_TIMEOUT_MILLISECONDS = 2000;
    static constexpr UartPort JETSON_UART_PORT = UartPort::Uart1;

    JetsonMessageReceive lastMessage;
    uint32_t lastFoundTargetTime;

    float fieldRelativeYawAngleAtVisionUpdate;
    float chassisRelativePitchAngleAtVisionUpdate;

    PlateKinematicState lastPlateKinematicState;

    Matrix<float, 1, 2> visionTargetAngles;
    Vector3f visionTargetPosition;

    CVDetectionType detectionType;
    JetsonMessageSend lastSend;
    alignas(JetsonMessageSend) uint8_t rawSerialBufferSend[sizeof(JetsonMessageSend)];
};
}  // namespace src::Informants::Vision