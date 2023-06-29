#pragma once

#include "utils/common_types.hpp"


namespace src {
    class Drivers;
} // namespace src


namespace src::Informants::TurretComms {

static constexpr uint32_t COMMS_DISCONNECTED_TIMEOUT = 1000;

class TurretCommunicator
{
    static constexpr float ANGLE_PRECISION_FACTOR = 10000.0f;
    static constexpr float CMPS2_TO_MPS2 = 0.01f;

    enum class CanID
    {
        TurretStatus    = 0x1f8,
        YawData         = 0x1f9,
        PitchData       = 0x1fa,
        RollData        = 0x1fb,
        ChassisToTurret = 0x1fc,
    };

    struct AngleMessageData
    {
        int16_t target;
        int16_t angularVelocity;
        int16_t linearAcceleration;
        uint8_t seq;
    } modm_packed;

    struct IMUData
    {
        float yaw             = 0.0f;
        int16_t yawVelocity   = 0.0f;
        float yawAcceleration = 0.0f;

        float pitch             = 0.0f;
        int16_t pitchVelocity   = 0.0f;
        float pitchAcceleration = 0.0f;

        float roll             = 0.0f;
        int16_t rollVelocity   = 0.0f;
        float rollAcceleration = 0.0f;

        uint8_t seq = 0xff;
    };

public:
    TurretCommunicator(src::Drivers* drivers, CANBus bus);

    void init();

#ifdef TARGET_TURRET
    void sendIMUData();
#else
    void handleYawDataRX(modm::can::Message const& msg);
    void handlePitchDataRX(modm::can::Message const& msg);
    void handleRollDataRX(modm::can::Message const& msg);
#endif

    using CANListenerProc = void (TurretCommunicator::*)(const modm::can::Message& message);
    class RXHandler : public tap::can::CanRxListener
    {
    public:
        RXHandler(src::Drivers* drivers, uint32_t id, CANBus bus, TurretCommunicator* ctx, CANListenerProc proc);
        void processMessage(modm::can::Message const& msg) override;

    private:
        TurretCommunicator* ctx;
        CANListenerProc proc;
    };

private:
    src::Drivers* drivers;

    MilliTimeout disconnectedTimeout;

    IMUData currentIMUData;
    IMUData lastIMUData;

#ifndef TARGET_TURRET
    RXHandler yawDataRXHandler;
    RXHandler pitchDataRXHandler;
    RXHandler rollDataRXHandler;
#endif
};

} // namespace src::Informants::TurretComms