#include "jetson_communicator.hpp"

#include <chrono>
#include <drivers.hpp>
#include <modm/platform/uart/uart_1.hpp>

#include "tap/communication/sensors/buzzer/buzzer.hpp"

#define READ(data, length) drivers->uart.read(JETSON_UART_PORT, data, length)
#define WRITE(data, length) drivers->uart.write(JETSON_UART_PORT, data, length)

namespace src::Informants::vision {

JetsonCommunicator::JetsonCommunicator(src::Drivers* drivers)
    : drivers(drivers),
      //   rawSerialByte(0),
      //   messageBuffer(visionBuffer<512>(JETSON_END_BYTE)),
      lastMessage(),
      currentSerialState(JetsonCommunicatorSerialState::SearchingForMagic),
      nextByteIndex(0),
      jetsonOfflineTimeout(),
#ifdef TARGET_SENTRY
      fieldRelativeYawAngleAtVisionUpdate(modm::toRadian(YAW_START_ANGLE)),
#else
      fieldRelativeYawAngleAtVisionUpdate(0.0f),
#endif
      chassisRelativePitchAngleAtVisionUpdate(modm::toRadian(PITCH_START_ANGLE))  //
{
}

void JetsonCommunicator::initialize() {
    jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);
    drivers->uart.init<JETSON_UART_PORT, JETSON_BAUD_RATE>();
}

uint8_t displayBuffer[JETSON_MESSAGE_SIZE];

float yawOffsetDisplay = 0;
float pitchOffsetDisplay = 0;
CVState cvStateDisplay = CVState::NOT_FOUND;
int readUnequal = 0;

int lastMsgTime = 0;
int msBetweenLastMessage = 0;

int displayBufIndex = 0;

/**
 * @brief Need to use modm's uart functions to read from the Jetson.
 * The Jetson sends information in the form of a JetsonMessage.
 *
 * modm currently loads received bytes into an internal buffer, accessible using the READ() call.
 * When we receive the message-agnostic end byte we unload from the buffer, check the message length, and reinterpret a JetsonMessage from our received bytes.
 */
void JetsonCommunicator::updateSerial() {
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    switch (currentSerialState) {
        case JetsonCommunicatorSerialState::SearchingForMagic: {
            size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);
            if (bytesRead != 1) return;

            displayBuffer[displayBufIndex] = rawSerialBuffer[0];
            displayBufIndex = (displayBufIndex + 1) % JETSON_MESSAGE_SIZE;

            // We've gotten data from the Jetson, so we can restart this.
            jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);

            // little endian moment
            // The the first byte in the message will be the LSB of the magic number,
            // so we only have to and it with 0xff, rather than shifting it right first.
            if (rawSerialBuffer[nextByteIndex] == ((JETSON_MESSAGE_MAGIC >> (8 * nextByteIndex)) & 0xff)) {
                nextByteIndex++;
            } else {
                nextByteIndex = 0;
            }

            if (nextByteIndex == sizeof(decltype(JETSON_MESSAGE_MAGIC))) {
                // We know that the magic is right, so we can just change the state. If the
                // magic number wasn't an exact match, we wouldn't have gotten this far.
                currentSerialState = JetsonCommunicatorSerialState::AssemblingMessage;
            }
            break;
        }
        case JetsonCommunicatorSerialState::AssemblingMessage: {
            size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);
            if (bytesRead != 1) {
                return;
            } else {
                nextByteIndex++;
            }

            displayBuffer[displayBufIndex] = rawSerialBuffer[0];
            displayBufIndex = (displayBufIndex + 1) % JETSON_MESSAGE_SIZE;

            // We've gotten data from the Jetson, so we can restart this.
            jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);

            if (nextByteIndex == JETSON_MESSAGE_SIZE) {
                lastMessage = *reinterpret_cast<JetsonMessage*>(&rawSerialBuffer);

                if (lastMsgTime == 0) {
                    lastMsgTime = tap::arch::clock::getTimeMilliseconds();
                } else {
                    msBetweenLastMessage = currTime - lastMsgTime;
                    lastMsgTime = currTime;
                }

                yawOffsetDisplay = lastMessage.targetYawOffset;
                pitchOffsetDisplay = lastMessage.targetPitchOffset;
                cvStateDisplay = lastMessage.cvState;

                // uint8_t* buffer = reinterpret_cast<uint8_t*>(&lastMessage);
                // WRITE(buffer, sizeof(decltype(JETSON_MESSAGE_MAGIC)));
                currentSerialState = JetsonCommunicatorSerialState::SearchingForMagic;
                nextByteIndex = 0;

                yawOffsetPredictor.update(lastMessage.targetYawOffset, currTime);
                pitchOffsetPredictor.update(lastMessage.targetPitchOffset, currTime);

                if (lastMessage.cvState == CVState::FOUND || lastMessage.cvState == CVState::FIRE) {
                    fieldRelativeYawAngleAtVisionUpdate = gimbal->getCurrentFieldRelativeYawAngle(AngleUnit::Radians);
                    chassisRelativePitchAngleAtVisionUpdate = gimbal->getCurrentChassisRelativePitchAngle(AngleUnit::Radians);

                    visionTargetAngles[0][yaw] = fieldRelativeYawAngleAtVisionUpdate + lastMessage.targetYawOffset;
                    visionTargetAngles[0][pitch] = chassisRelativePitchAngleAtVisionUpdate + lastMessage.targetPitchOffset;
                }
                if (lastMessage.cvState == CVState::FOUND) {
                    tap::buzzer::playNote(&drivers->pwm, 466);
                } else if (lastMessage.cvState == CVState::FIRE) {
                    tap::buzzer::playNote(&drivers->pwm, 932);
                } else {
                    tap::buzzer::playNote(&drivers->pwm, 0);
                }
            }
            break;
        }
    }
    if (!isJetsonOnline()) {
        tap::buzzer::playNote(&drivers->pwm, 0);
    }
}

Matrix<float, 1, 2> const& JetsonCommunicator::getVisionTargetAngles() {
    // uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    // float yawOffsetPredicted = /*lastMessage.cvState == FIRE ? */ yawOffsetPredictor.getInterpolatedValue(currTime);
    // float yawOffsetPredicted = lastMessage.targetYawOffset;
    // float pitchOffsetPredicted = /*lastMessage.cvState == FIRE ? */ pitchOffsetPredictor.getInterpolatedValue(currTime);
    // float pitchOffsetPredicted = lastMessage.targetPitchOffset;
    // visionTargetAngles[0][yaw] = fieldRelativeYawAngleAtVisionUpdate - yawOffsetPredicted;
    // visionTargetAngles[0][pitch] = chassisRelativePitchAngleAtVisionUpdate - pitchOffsetPredicted;
    // return visionTargetAngles;

    // visionTargetAngles[0][yaw] = modm::toRadian(38.0f);
    // visionTargetAngles[0][pitch] = modm::toRadian(143.0f);
    return visionTargetAngles;
}
}  // namespace src::Informants::vision
