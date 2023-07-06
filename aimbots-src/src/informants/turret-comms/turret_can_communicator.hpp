#pragma once

#include "informants/kinematic_informant.hpp"
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

    static constexpr uint32_t SEND_TO_TURRET_PERIOD  = 300;

    enum class CanID
    {
        TurretStatus    = 0x1f8,
        YawData         = 0x1f9,
        PitchData       = 0x1fa,
        RollData        = 0x1fb,
        ChassisToTurret = 0x1fc,
    };

    enum : uint8_t
    {
        CHASSIS_TO_TURRET_MSG_REQUEST_IMU_CALIBRATION = 0x01,
    };

    struct AngleMessageData
    {
        int16_t target;
        int16_t angularVelocity;
		int16_t angularAcceleration;
        uint8_t seq;
    } modm_packed;

public:
    struct IMUData
    {
        float yaw                    = 0.0f;
        float yawAngularVelocity     = 0.0f;
		float yawAngularAcceleration = 0.0f;

        float pitch                    = 0.0f;
        float pitchAngularVelocity     = 0.0f;
		float pitchAngularAcceleration = 0.0f;

        float roll                    = 0.0f;
        float rollAngularVelocity     = 0.0f;
		float rollAngularAcceleration = 0.0f;

        uint8_t seq = 0xff;
    };

    TurretCommunicator(src::Drivers* drivers, CANBus bus);

    void init();

	float getLastReportedAngle(AngularAxis axis, AngleUnit unit);
	float getLastReportedAngularVelocity(AngularAxis axis, AngleUnit unit);
	float getLastReportedAngularAcceleration(AngularAxis axis, AngleUnit unit);

#ifdef TARGET_TURRET
    void sendIMUData();
    void handleChassisRequestRX(modm::can::Message const& msg);
#else
    inline void requestTurretIMUCalibrate() { chassisRequestData |= CHASSIS_TO_TURRET_MSG_REQUEST_IMU_CALIBRATION; }

    void sendTurretRequest();

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
    CANBus bus;

    MilliTimeout disconnectedTimeout;

    IMUData currentIMUData;
    IMUData lastIMUData;

    uint8_t chassisRequestData;

#ifndef TARGET_TURRET
    PeriodicMilliTimer sendToTurretTimer;

    RXHandler yawDataRXHandler;
    RXHandler pitchDataRXHandler;
    RXHandler rollDataRXHandler;
#else
    uint8_t sendSequence = 0;

    RXHandler chassisRequestRXHandler;
#endif
};

} // namespace src::Informants::TurretComms
