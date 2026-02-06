#pragma once

#include "informants/kinematics/kinematic_informant.hpp"
#include "utils/tools/common_types.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace src::Informants::PowerComms {

static constexpr uint32_t COMMS_DISCONNECTED_TIMEOUT = 1000;

class PowerCommunicator {

   // const float currentMul = 0.00125;

    // static constexpr float ANGLE_PRECISION_FACTOR = 10000.0f;  // Max input before clipping = 32'767 / 10000 = +-3.276 rads
    // static constexpr float LINEAR_PRECISION_FACTOR = 100.0f;   // Max input before clipping = 32'767 / 100 = +-327.6 m/s^2
    // static constexpr float CMPS2_TO_MPS2 = 0.01f;

    // static constexpr uint32_t SEND_TO_TURRET_PERIOD = 300;

    enum class CanID {
        Power = 0x446,
    };


    // struct AngleMessageData {
    //     int16_t target;
    //     int16_t angularVelocity;
    //     int16_t linearAcceleration;
    //     uint8_t seq;
    // } modm_packed;

public:
    struct PowerData { // change when kebard isn spid
        uint16_t c = 0;
        uint16_t v = 0;
        uint16_t p = 0;
    };

    PowerCommunicator(src::Drivers* drivers, CANBus bus);

    void init();

    float getLastReportedAngle(AngularAxis axis, AngleUnit unit);
    float getLastReportedAngularVelocity(AngularAxis axis, AngleUnit unit);
    float getLastReportedLinearAcceleration(LinearAxis axis);

// #ifdef TARGET_TURRET
    void sendIMUData();
    void handleChassisRequestRX(modm::can::Message const& msg);
// #else

    void sendTurretRequest();

    void handleYawDataRX(modm::can::Message const& msg);
    void handlePitchDataRX(modm::can::Message const& msg);
    void handleRollDataRX(modm::can::Message const& msg);
// #endif

    using CANListenerProc = void (PowerCommunicator::*)(const modm::can::Message& message);
    class RXHandler : public tap::can::CanRxListener {
    public:
        RXHandler(src::Drivers* drivers, uint32_t id, CANBus bus, PowerCommunicator* ctx, CANListenerProc proc);
        void processMessage(modm::can::Message const& msg) override;

    private:
        PowerCommunicator* ctx;
        CANListenerProc proc;
    };

private:
    src::Drivers* drivers;
    CANBus bus;

    MilliTimeout disconnectedTimeout;


    uint8_t chassisRequestData;

// #ifndef TARGET_TURRET
    PeriodicMilliTimer sendToTurretTimer;

    // RXHandler yawDataRXHandler;
    // RXHandler pitchDataRXHandler;
    // RXHandler rollDataRXHandler;
// #else
    uint8_t sendSequence = 0;

    // RXHandler chassisRequestRXHandler;
// #endif
};

}  // namespace src::Informants::TurretComms
