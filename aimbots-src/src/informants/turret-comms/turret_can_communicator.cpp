#include "turret_can_communicator.hpp"

#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/communication/sensors/imu/bmi088/bmi088.hpp"

#include "drivers.hpp"

namespace src::Informants::TurretComms {

TurretCommunicator::IMUData DBG_recievedIMUData;

TurretCommunicator::TurretCommunicator(src::Drivers* drivers, CANBus bus)
    : drivers(drivers),
      bus(bus),
      disconnectedTimeout(COMMS_DISCONNECTED_TIMEOUT),
      chassisRequestData(0)
#ifndef TARGET_TURRET
      ,
      sendToTurretTimer(SEND_TO_TURRET_PERIOD),
      yawDataRXHandler(drivers, static_cast<uint32_t>(CanID::YawData), bus, this, &TurretCommunicator::handleYawDataRX),
      pitchDataRXHandler(
          drivers,
          static_cast<uint32_t>(CanID::PitchData),
          bus,
          this,
          &TurretCommunicator::handlePitchDataRX),  // TODO: Probably needs to be Pitch/Roll
      rollDataRXHandler(drivers, static_cast<uint32_t>(CanID::RollData), bus, this, &TurretCommunicator::handleRollDataRX)
#else
      ,
      chassisRequestRXHandler(
          drivers,
          static_cast<uint32_t>(CanID::ChassisToTurret),
          bus,
          this,
          &TurretCommunicator::handleChassisRequestRX)
#endif
{
}

void TurretCommunicator::init() {
#ifndef TARGET_TURRET
    yawDataRXHandler.attachSelfToRxHandler();
    pitchDataRXHandler.attachSelfToRxHandler();
    rollDataRXHandler.attachSelfToRxHandler();
    sendToTurretTimer.restart(SEND_TO_TURRET_PERIOD);
#else
    chassisRequestRXHandler.attachSelfToRxHandler();
#endif
}

float TurretCommunicator::getLastReportedAngle(AngularAxis axis, AngleUnit unit) {
    float val = 0.0f;

    switch (axis) {
        case YAW_AXIS:
            val = lastIMUData.yaw;
            break;
        case PITCH_AXIS:
            val = lastIMUData.pitch;
            break;
        case ROLL_AXIS:
            val = lastIMUData.roll;
            break;
    }

    return (unit == AngleUnit::Radians) ? val : modm::toDegree(val);
}

float TurretCommunicator::getLastReportedAngularVelocity(AngularAxis axis, AngleUnit unit) {
    float val = 0.0f;

    switch (axis) {
        case YAW_AXIS:
            val = lastIMUData.yawAngularVelocity;
            break;
        case PITCH_AXIS:
            val = lastIMUData.pitchAngularVelocity;
            break;
        case ROLL_AXIS:
            val = lastIMUData.rollAngularVelocity;
            break;
    }

    return (unit == AngleUnit::Radians) ? val : modm::toDegree(val);
}

float TurretCommunicator::getLastReportedLinearAcceleration(LinearAxis axis) {
    float val = 0.0f;

    switch (axis) {
        case X_AXIS:
            val = lastIMUData.xLinearAcceleration;
            break;
        case Y_AXIS:
            val = lastIMUData.yLinearAcceleration;
            break;
        case Z_AXIS:
            val = lastIMUData.zLinearAcceleration;
            break;
    }
    return val;
}

#ifdef TARGET_TURRET

uint8_t sequenceDisplay = 0;

void TurretCommunicator::sendIMUData() {
    using namespace tap::communication::sensors::imu::bmi088;
    Bmi088::ImuState imuState = drivers->bmi088.getImuState();

    if ((imuState == Bmi088::ImuState::IMU_CALIBRATED || imuState == Bmi088::ImuState::IMU_NOT_CALIBRATED) &&
        drivers->can.isReadyToSend(bus)) {
        switch (sendState) {
            case (AngularAxis::YAW_AXIS): {
                modm::can::Message yawMsg(static_cast<uint32_t>(CanID::YawData), 7);
                AngleMessageData* yawData = reinterpret_cast<AngleMessageData*>(yawMsg.data);
                yawData->target =
                    static_cast<int16_t>(drivers->kinematicInformant.getLocalIMUAngle(YAW_AXIS) * ANGLE_PRECISION_FACTOR);
                // TODO: Check if this is right??
                yawData->angularVelocity = static_cast<int16_t>(
                    drivers->kinematicInformant.getIMUAngularVelocity(YAW_AXIS) * ANGLE_PRECISION_FACTOR);
                yawData->linearAcceleration = static_cast<int16_t>(
                    drivers->kinematicInformant.getIMULinearAcceleration(Z_AXIS) * LINEAR_PRECISION_FACTOR);
                yawData->seq = sendSequence;

                drivers->can.sendMessage(bus, yawMsg);
                sendState = AngularAxis::PITCH_AXIS;
                break;
            }
            case (AngularAxis::PITCH_AXIS): {
                modm::can::Message pitchMsg(static_cast<uint32_t>(CanID::PitchData), 7);
                AngleMessageData* pitchData = reinterpret_cast<AngleMessageData*>(pitchMsg.data);
                pitchData->target =
                    static_cast<int16_t>(drivers->kinematicInformant.getLocalIMUAngle(PITCH_AXIS) * ANGLE_PRECISION_FACTOR);
                // TODO: Check if this is right??
                pitchData->angularVelocity = static_cast<int16_t>(
                    drivers->kinematicInformant.getIMUAngularVelocity(PITCH_AXIS) * ANGLE_PRECISION_FACTOR);
                pitchData->linearAcceleration = static_cast<int16_t>(
                    drivers->kinematicInformant.getIMULinearAcceleration(X_AXIS) * LINEAR_PRECISION_FACTOR);
                pitchData->seq = sendSequence;

                drivers->can.sendMessage(bus, pitchMsg);
                sendState = AngularAxis::ROLL_AXIS;
                break;
            }
            case (AngularAxis::ROLL_AXIS): {
                modm::can::Message rollMsg(static_cast<uint32_t>(CanID::RollData), 7);
                AngleMessageData* rollData = reinterpret_cast<AngleMessageData*>(rollMsg.data);
                rollData->target =
                    static_cast<int16_t>(drivers->kinematicInformant.getLocalIMUAngle(ROLL_AXIS) * ANGLE_PRECISION_FACTOR);
                // TODO: Check if this is right??
                rollData->angularVelocity = static_cast<int16_t>(
                    drivers->kinematicInformant.getIMUAngularVelocity(ROLL_AXIS) * ANGLE_PRECISION_FACTOR);
                rollData->linearAcceleration = static_cast<int16_t>(
                    drivers->kinematicInformant.getIMULinearAcceleration(Y_AXIS) * LINEAR_PRECISION_FACTOR);
                rollData->seq = sendSequence;

                drivers->can.sendMessage(bus, rollMsg);

                sendSequence++;

                sequenceDisplay = sendSequence;
                sendState = AngularAxis::YAW_AXIS;
                break;
            }
        }
    }
}

void TurretCommunicator::handleChassisRequestRX(modm::can::Message const& msg) {
    uint8_t data = msg.data[0];

    if (data & CHASSIS_TO_TURRET_MSG_REQUEST_IMU_CALIBRATION) {
        drivers->leds.set(tap::gpio::Leds::Green, true);

        tap::buzzer::playNote(&drivers->pwm, 700);

        drivers->kinematicInformant.recalibrateIMU();
    }
}
#else
void TurretCommunicator::sendTurretRequest() {
    if (/*sendToTurretTimer.execute() &&*/ drivers->can.isReadyToSend(bus)) {
        modm::can::Message msg(static_cast<uint32_t>(CanID::ChassisToTurret), 1);
        msg.setExtended(false);
        msg.data[0] = chassisRequestData;
        drivers->can.sendMessage(bus, msg);

        chassisRequestData = 0;

        sendToTurretTimer.restart();
    }
}

float yawDataDisplay = 0;
float pitchDataDisplay = 0;
float rollDataDisplay = 0;

float debugYawDisplay = 0;
float debugPitchDisplay = 0;
float debugRollDisplay = 0;

bool outOfSequence = false;
uint8_t incomingSeqDisplay = 0;
uint8_t currentSeqDisplay = 0;

void TurretCommunicator::handleYawDataRX(modm::can::Message const& msg) {
    AngleMessageData const* data = reinterpret_cast<AngleMessageData const*>(msg.data);
    yawDataDisplay = static_cast<float>(data->target) / ANGLE_PRECISION_FACTOR;
    currentIMUData.yaw = static_cast<float>(data->target) / ANGLE_PRECISION_FACTOR;
    currentIMUData.yawAngularVelocity = static_cast<float>(data->angularVelocity) / ANGLE_PRECISION_FACTOR;
    currentIMUData.zLinearAcceleration = static_cast<float>(data->linearAcceleration) / LINEAR_PRECISION_FACTOR;

    currentIMUData.seq = data->seq;

    outOfSequence = false;
}

void TurretCommunicator::handlePitchDataRX(modm::can::Message const& msg) {
    AngleMessageData const* data = reinterpret_cast<AngleMessageData const*>(msg.data);
    incomingSeqDisplay = data->seq;
    currentSeqDisplay = currentIMUData.seq;
    if (data->seq != currentIMUData.seq) {
        // Got an out-of-sequence message, so we can't reliably use this data
        // TODO: Report this error in a better way?
        outOfSequence = true;
        return;
    }

    pitchDataDisplay = static_cast<float>(data->target) / ANGLE_PRECISION_FACTOR;
    currentIMUData.pitch = static_cast<float>(data->target) / ANGLE_PRECISION_FACTOR;
    currentIMUData.pitchAngularVelocity = static_cast<float>(data->angularVelocity) / ANGLE_PRECISION_FACTOR;
    currentIMUData.xLinearAcceleration = static_cast<float>(data->linearAcceleration) / LINEAR_PRECISION_FACTOR;
}

void TurretCommunicator::handleRollDataRX(modm::can::Message const& msg) {
    AngleMessageData const* data = reinterpret_cast<AngleMessageData const*>(msg.data);
    if (data->seq != currentIMUData.seq) {
        // Got an out-of-sequence message, so we can't reliably use this data
        // TODO: Report this error in a better way?
        return;
    }

    rollDataDisplay = static_cast<float>(data->target) / ANGLE_PRECISION_FACTOR;
    currentIMUData.roll = static_cast<float>(data->target) / ANGLE_PRECISION_FACTOR;
    currentIMUData.rollAngularVelocity = static_cast<float>(data->angularVelocity) / ANGLE_PRECISION_FACTOR;
    currentIMUData.yLinearAcceleration = static_cast<float>(data->linearAcceleration) / LINEAR_PRECISION_FACTOR;

    lastIMUData = currentIMUData;
    DBG_recievedIMUData = lastIMUData;
    currentIMUData = IMUData{};
}
#endif

TurretCommunicator::RXHandler::RXHandler(
    src::Drivers* drivers,
    uint32_t id,
    CANBus bus,
    TurretCommunicator* ctx,
    CANListenerProc proc)
    : CanRxListener(drivers, id, bus),
      ctx(ctx),
      proc(proc) {}

void TurretCommunicator::RXHandler::processMessage(modm::can::Message const& msg) { (ctx->*proc)(msg); }

}  // namespace src::Informants::TurretComms
