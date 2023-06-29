#include "turret_can_communicator.hpp"

namespace src::Informants::TurretComms {

TurretCommunicator::TurretCommunicator(src::Drivers* drivers, CANBus bus)
    : drivers(drivers)
    , disconnectedTimeout(COMMS_DISCONNECTED_TIMEOUT)
#ifndef TARGET_TURRET
    , yawDataRXHandler(drivers, static_cast<uint32_t>(CanID::YawData), bus, this, handleYawDataRX)
    , pitchDataRXHandler(drivers, static_cast<uint32_t>(CanID::PitchData), bus, this, handlePitchDataRX)
    , rollDataRXHandler(drivers, static_cast<uint32_t>(CanID::RollData), bus, this, handleRollDataRX)
#endif
{ }


void TurretCommunicator::init()
{
#ifndef TARGET_TURRET
    yawDataRXHandler.attachSelfToRxHandler();
    pitchDataRXHandler.attachSelfToRxHandler();
    rollDataRXHandler.attachSelfToRxHandler();
#endif
}

#ifdef TARGET_TURRET
void TurretCommunicator::sendIMUData()
{
    // TODO: Send imu data to chassis
}
#else
void TurretCommunicator::handleYawDataRX(modm::can::Message const& msg)
{
    AngleMessageData const* data = reinterpret_cast<AngleMessageData const*>(msg.data);

    currentIMUData.yaw             = static_cast<float>(data->target) / ANGLE_PRECISION_FACTOR;
    currentIMUData.yawVelocity     = data->angularVelocity;
    currentIMUData.yawAcceleration = static_cast<float>(data->linearAcceleration) * CMPS2_TO_MPS2;

    currentIMUData.seq = data->seq;
}


void TurretCommunicator::handlePitchDataRX(modm::can::Message const& msg)
{
    AngleMessageData const* data = reinterpret_cast<AngleMessageData const*>(msg.data);
    if ( data->seq != currentIMUData.seq )
    {
        // Got an out-of-sequence message, so we can't reliably use this data
        // TODO: Report this error in a better way?
        return;
    }

    currentIMUData.pitch             = static_cast<float>(data->target) / ANGLE_PRECISION_FACTOR;
    currentIMUData.pitchVelocity     = data->angularVelocity;
    currentIMUData.pitchAcceleration = static_cast<float>(data->linearAcceleration) * CMPS2_TO_MPS2;
}


void TurretCommunicator::handleRollDataRX(modm::can::Message const& msg)
{
    AngleMessageData const* data = reinterpret_cast<AngleMessageData const*>(msg.data);
    if ( data->seq != currentIMUData.seq )
    {
        // Got an out-of-sequence message, so we can't reliably use this data
        // TODO: Report this error in a better way?
        return;
    }

    currentIMUData.roll             = static_cast<float>(data->target) / ANGLE_PRECISION_FACTOR;
    currentIMUData.rollVelocity     = data->angularVelocity;
    currentIMUData.rollAcceleration = static_cast<float>(data->linearAcceleration) * CMPS2_TO_MPS2;

    lastIMUData    = currentIMUData;
    currentIMUData = IMUData { };
}
#endif

} // namespace src::Informants::TurretComms