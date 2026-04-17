#pragma once

#include <tap/architecture/timeout.hpp>
#include <tap/util_macros.hpp>
#include <utils/tools/common_types.hpp>
#include <modm/math/geometry/location_2d.hpp>


#include "informants/kinematics/enemy_data_conversion.hpp"

#include "jetson_protocol.hpp"

namespace src {
class Drivers;
}

namespace src::Informants::Vision {

enum class JetsonCommunicatorSerialState : uint8_t {
    SearchingForMagic = 0,
    HandleMessageType,
    AssemblingAimMessage,
    AssemblingLocalizationMessage,
};

struct AutoAimAngles {
    float yaw;
    float pitch;
};

class JetsonCommunicator {
public:
    JetsonCommunicator(src::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(JetsonCommunicator);
    ~JetsonCommunicator() = default;

    void initialize();
    void updateSerial();

    inline bool isJetsonOnline() const { return !jetsonOfflineTimeout.isExpired(); }

    inline JetsonAimMessage const& getLastValidAimMessage() const { return lastAimMessage; }
    inline JetsonLocalizationMessage const& getLastValidLocalizationMessage() const { return lastLocalizationMessage; }

    PlateKinematicState getPlatePrediction(uint32_t dt) const;

    AutoAimAngles getAutoAimAngles() const;
    modm::Location2D<float> getLocationEstimate() const;

    uint32_t getLastFoundTargetTime() const { return lastFoundTargetTime; }

    uint32_t getLastFrameCaptureDelay() const { return visionDataConverter.getLastFrameCaptureDelay(); }

    bool isLastFrameStale() const;

    bool shouldFire();
private:
    src::Drivers* drivers;
    src::Informants::Vision::VisionDataConversion visionDataConverter;

    alignas(JetsonAimMessage) uint8_t rawSerialBuffer[JETSON_MAX_MESSAGE_SIZE];

    JetsonCommunicatorSerialState currentSerialState;
    size_t nextByteIndex;

    tap::arch::MilliTimeout jetsonOfflineTimeout;
    tap::arch::MilliTimeout fireTimeout;

    static constexpr uint32_t JETSON_BAUD_RATE = 115200;
    static constexpr uint16_t JETSON_OFFLINE_TIMEOUT_MILLISECONDS = 2000;
    static constexpr UartPort JETSON_UART_PORT = UartPort::Uart1;

    JetsonAimMessage lastAimMessage {
        .magic = 0,
        .messageType = 0,
        .pitch = 0.0f,
        .yaw = 0.0f,
        .timeUntilNextFire = 0,
        .cvState = CVState::NOT_FOUND
    };
    JetsonLocalizationMessage lastLocalizationMessage {
        .magic = 0,
        .messageType = 0,
        .x = 0.0f,
        .y = 0.0f,
        .theta = 0.0f
    };

    EmbeddedTransformationMessage transformationMessageToJetson;
    EmbeddedOdometryMessage odometryMessageToJetson;
    uint32_t lastFoundTargetTime;

    PlateKinematicState lastPlateKinematicState;

    Matrix<float, 1, 2> visionTargetAngles;
    Vector3f visionTargetPosition;
};
}  // namespace src::Informants::Vision