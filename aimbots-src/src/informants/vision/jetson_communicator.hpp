#pragma once

#include <cassert>
#include <deque>

#include <tap/architecture/timeout.hpp>
#include <tap/architecture/periodic_timer.hpp>
#include <tap/util_macros.hpp>
#include <tap/communication/serial/dji_serial.hpp>
#include <tap/communication/serial/ref_serial_data.hpp>
#include <tap/drivers.hpp>

#include "utils/common_types.hpp"
#include "informants/enemy_data_conversion.hpp"

namespace src {
class Drivers;
}

namespace src::Informants::Vision {

enum CVState : uint8_t {
    NOT_FOUND = 0,
    FOUND = 1,
    FIRE = 2,
};

class JetsonCommunicator : public tap::communication::serial::DJISerial {
    public:

        static constexpr uint32_t JETSON_BAUD_RATE = 115200;

        static constexpr UartPort JETSON_UART_PORT = UartPort::Uart1;
        struct JetsonMessage {
            float targetX;
            float targetY;
            float targetZ;
            uint8_t delay;  // ms
            CVState cvState;
        } modm_packed;

        JetsonCommunicator(src::Drivers* drivers);
        DISALLOW_COPY_AND_ASSIGN(JetsonCommunicator);
        ~JetsonCommunicator() = default;

        void initialize();

        void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

        void parseRxMessage();

        void updateSerial();

        bool isLastFrameStale() const;

        void sendMessage();

        void sendRobotID();

        void sendMatchInfo();

        void sendMatchState();

        inline bool isJetsonOnline() const { return !jetsonOfflineTimeout.isExpired(); }

        inline JetsonMessage const& getLastValidMessage() const { return lastMessage; }

        PlateKinematicState getPlatePrediction(uint32_t dt) const;

        inline uint32_t getLastFoundTargetTime() const { return lastFoundTargetTime; }

        inline uint32_t getLastFrameCaptureDelay() const { return visionDataConverter.getLastFrameCaptureDelay(); }

    private:
        enum TXMessageTypes {
            TX_MESSAGE_ROBOT_ID = 1,
            TX_MESSAGE_MATCH_INFO = 2,
            TX_MESSAGE_MATCH_STATE = 4
        };

        // Add more message types as CV does more cool shit
        enum RXMessageTypes{
            RX_TARGET_DATA = 1
        };

        src::Drivers* drivers;
        src::Informants::Vision::VisionDataConversion visionDataConverter;

        alignas(JetsonMessage) uint8_t rawSerialBuffer[sizeof(JetsonMessage)];

        // JetsonCommunicatorSerialState currentSerialState;
        // size_t nextByteIndex;

        static constexpr uint16_t JETSON_OFFLINE_TIMEOUT_MS = 2'000;

        static constexpr uint32_t PERIOD_BTWN_TX_ROBOTID_MS = 3'000;

        static constexpr uint32_t PERIOD_BTWN_TX_MATCH_INFO_MS = 1'000;

        static constexpr uint32_t PERIOD_BTWN_TX_MATCH_RESULT_MS = 5'000;

        tap::arch::MilliTimeout jetsonOfflineTimeout;

        tap::arch::PeriodicMilliTimer robotIdTimeoutTX{PERIOD_BTWN_TX_ROBOTID_MS};

        tap::arch::PeriodicMilliTimer matchInfoTimeoutTX{PERIOD_BTWN_TX_MATCH_INFO_MS};

        tap::arch::PeriodicMilliTimer matchResultTimeoutTX{PERIOD_BTWN_TX_MATCH_RESULT_MS};

        JetsonMessage lastMessage;
        uint32_t lastFoundTargetTime;

        float fieldRelativeYawAngleAtVisionUpdate;
        float chassisRelativePitchAngleAtVisionUpdate;

        PlateKinematicState lastPlateKinematicState;

        Matrix<float, 1, 2> visionTargetAngles;
        Vector3f visionTargetPosition;

    };
}  // namespace src::Informants::Vision