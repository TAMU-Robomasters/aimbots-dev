#pragma once

#include <tap/architecture/timeout.hpp>
#include <tap/architecture/periodic_timer.hpp>
#include <tap/util_macros.hpp>
#include <utils/common_types.hpp>

#include "informants/enemy_data_conversion.hpp"

namespace src {
class Drivers;
}

namespace src::Informants::Vision {

enum class JetsonCommunicatorSerialState : uint8_t {
    SearchingForMagic = 0,
    AssemblingMessage,
};

enum CVState : uint8_t {
    NOT_FOUND = 0,
    FOUND = 1,
    FIRE = 2,
};

static constexpr uint8_t JETSON_MESSAGE_MAGIC = 'a';

struct JetsonMessage {
    uint8_t magic;
    float targetX;
    float targetY;
    float targetZ;
    uint8_t delay;  // ms
    CVState cvState;
} __attribute__((packed));

static_assert(sizeof(JetsonMessage) == 15, "JetsonMessage is not the correct size");

static constexpr size_t JETSON_MESSAGE_SIZE = sizeof(JetsonMessage);

class JetsonCommunicator {
    public:
        JetsonCommunicator(src::Drivers* drivers);
        DISALLOW_COPY_AND_ASSIGN(JetsonCommunicator);
        ~JetsonCommunicator() = default;

        void initialize();
        void updateSerial();

        inline bool isJetsonOnline() const { return !jetsonOfflineTimeout.isExpired(); }

        inline JetsonMessage const& getLastValidMessage() const { return lastMessage; }

        PlateKinematicState getPlatePrediction(uint32_t dt) const;

        inline uint32_t getLastFoundTargetTime() const { return lastFoundTargetTime; }

        inline uint32_t getLastFrameCaptureDelay() const { return visionDataConverter.getLastFrameCaptureDelay(); }

        bool isLastFrameStale() const;

        void sendRobotID();
        void sendMatchTime();
        void sendMatchState();

    private:
        src::Drivers* drivers;
        src::Informants::Vision::VisionDataConversion visionDataConverter;

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

        PlateKinematicState lastPlateKinematicState;

        Matrix<float, 1, 2> visionTargetAngles;
        Vector3f visionTargetPosition;

        enum TxMessageID
        { 
            TX_MESSAGE_REFEREE_MATCH_TIME = 0,
            TX_MESSAGE_REFEERE_MATCH_RESULT = 1,
            TX_MESSAGE_ROBOT_ID = 2,
        };

        static constexpr uint32_t TIME_ROBOTID_FREQ = 3'000;

        tap::arch::PeriodicMilliTimer sendRobotIdTimeout{TIME_ROBOTID_FREQ};
    };
}  // namespace src::Informants::Vision