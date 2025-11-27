#include "jetson_communicator.hpp"

#include <drivers.hpp>

#include "tap/communication/sensors/buzzer/buzzer.hpp"

#define READ(data, length) drivers->uart.read(JETSON_UART_PORT, data, length)
#define WRITE(data, length) drivers->uart.write(JETSON_UART_PORT, data, length)

namespace src::Informants::Vision {

JetsonCommunicator::JetsonCommunicator(src::Drivers* drivers)
    : drivers(drivers),
      visionDataConverter(drivers),
      currentSerialState(JetsonCommunicatorSerialState::SearchingForMagic),
      nextByteIndex(0),
      jetsonOfflineTimeout(),
      fireTimeout(),
      lastMessage(),
      transformationMessageToJetson({EMBEDDED_MESSAGE_MAGIC, {0}})
{}

void JetsonCommunicator::initialize() {
    jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);
    drivers->uart.init<JETSON_UART_PORT, JETSON_BAUD_RATE>();
}

uint8_t displayBuffer[JETSON_MESSAGE_SIZE];
int displayBufIndex = 0;
size_t nextByteIndexDisplay = 0;
uint32_t timeDisplay = 0;

float targetYawDisplay = 0;
float targetPitchDisplay = 0;
uint8_t timeUntilNextFireDisplay = 0;
CVState cvStateDisplay = CVState::NOT_FOUND;

float fieldRelativeYawAngleDisplay = 0;
float chassisRelativePitchAngleDisplay = 0;

int lastMsgTimeDisplay = 0;
int msBetweenLastMessageDisplay = 0;

float frameDelayOffsetDisplay_ms = 0.0f;

uint32_t fireTimeoutTimeRemainingDisplay = 0.0f;

float messageTypeDisplay = 0.0f;

/**
 * @brief Need to use modm's uart functions to read from the Jetson.
 * The Jetson sends information in the form of a JetsonMessage.
 *
 * We send a magic number from the Jetson to the Development Board as the header of every message, and we can use that magic
 * number to determine whether a message is (probably) going to be valid. If a long magic number comes through as valid, we
 * can assume that the rest of the message is valid as well.
 *
 * modm currently loads received bytes into an internal buffer, accessible using the READ() call.
 * When we receive the message-agnostic end byte we unload from the buffer, check the message length, and reinterpret a
 * JetsonMessage from our received bytes.
 */
alignas(JetsonMessage) uint8_t rawSerialDisplay[sizeof(JetsonMessage)];

void JetsonCommunicator::updateSerial() {
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    timeDisplay = currTime;

    fireTimeoutTimeRemainingDisplay = fireTimeout.timeRemaining();

    size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);  // attempts to pull one byte from the buffer
    if (bytesRead != 1) return;

    nextByteIndexDisplay = 1;
    // We've successfully read a new byte from the Jetson, so we can restart this.
    jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);

    displayBuffer[displayBufIndex] = rawSerialBuffer[0];            // copy byte to display buffer
    displayBufIndex = (displayBufIndex + 1) % JETSON_MESSAGE_SIZE;  // increment display index and wrap around if necessary

    switch (currentSerialState) {
        // ...looking for message start...
        case JetsonCommunicatorSerialState::SearchingForMagic: {
            // Check if the byte we just read is the byte we expected in the magic number.
            if (rawSerialBuffer[nextByteIndex] == ((JETSON_MESSAGE_MAGIC >> (8 * nextByteIndex)) & 0xff)) {
                nextByteIndex++;
            } else {
                nextByteIndex = 0;  // if not, reset the index and start over.
            }

            // Wait until we've reached the end of the magic number. If any of the bytes in the magic number weren't a match,
            // we wouldn't have gotten this far.
            if (nextByteIndex == sizeof(decltype(JETSON_MESSAGE_MAGIC))) {
                currentSerialState = JetsonCommunicatorSerialState::HandleMessageType;
            }
            break;
        }
        
        case JetsonCommunicatorSerialState::HandleMessageType: {
            uint8_t messageType = rawSerialBuffer[nextByteIndex];
            messageTypeDisplay = messageType;
            if (messageType == JETSON_AIM_MESSAGE) {
                nextByteIndex++;
                currentSerialState = JetsonCommunicatorSerialState::AssemblingMessage;
            } 
            else if (messageType == JETSON_TRANSFORMATION_QUERY) { // respond to query
                uint8_t frameDelay_ms = 0.0f; 
                //!!! potential issue where not enough time has passed and we don't read anything
                READ(&frameDelay_ms, 1);
                
                drivers->kinematicInformant.mirrorPastRobotFrame(frameDelay_ms + frameDelayOffsetDisplay_ms);

                src::Informants::Transformers::CoordinateFrame turretFieldFrame =
                    drivers->kinematicInformant.getTurretFrames().getFrame(Transformers::TurretFrameType::TURRET_FIELD_FRAME);

                src::Informants::Transformers::CoordinateFrame cameraFrame =
                    drivers->kinematicInformant.getTurretFrames().getFrame(Transformers::TurretFrameType::TURRET_CAMERA_FRAME);
                
                Matrix4f cameraToTurret = cameraFrame.getTransformToFrame(turretFieldFrame);
                
                std::memcpy(transformationMessageToJetson.matrix, cameraToTurret.element, sizeof(float) * 16);

                // Send data to Jetson
                WRITE((uint8_t*)&transformationMessageToJetson, sizeof(transformationMessageToJetson));

                // We responded to query from jetson, reset the byte index and go back to searching for the magic number.
                nextByteIndex = 0;
                currentSerialState = JetsonCommunicatorSerialState::SearchingForMagic;
            }
 
            // restart everything if nothing matches
            else {
                nextByteIndex = 0;
                currentSerialState = JetsonCommunicatorSerialState::SearchingForMagic;
            } 
             
            break;
        }
        // ...found message start, assemble message...
        case JetsonCommunicatorSerialState::AssemblingMessage: {
            nextByteIndex++;

            // Increment the byte index until we reach the expected end of a message, then parse the message.
            if (nextByteIndex == JETSON_MESSAGE_SIZE) {
                // Reinterpret the received bytes into a JetsonMessage
                lastMessage = *reinterpret_cast<JetsonMessage*>(&rawSerialBuffer);

                // Update last message time and time between last message and now.
                if (lastMsgTimeDisplay == 0) {
                    lastMsgTimeDisplay = tap::arch::clock::getTimeMilliseconds();
                } else {
                    msBetweenLastMessageDisplay =
                        currTime - lastMsgTimeDisplay;  // Should be pretty close to the message send rate.
                    lastMsgTimeDisplay = currTime;
                }

                targetYawDisplay = modm::toDegree(lastMessage.yaw);
                targetPitchDisplay = modm::toDegree(lastMessage.pitch);
                timeUntilNextFireDisplay = lastMessage.timeUntilNextFire;
                cvStateDisplay = lastMessage.cvState;

                if (lastMessage.cvState >= CVState::FOUND) {  // If the CV state is FOUND or better
                    // TODO: Explore using predictors to smoothen effect of large time gap between vision updates.

                    fireTimeout.restart(lastMessage.timeUntilNextFire);

                    // position is relative to camera
                    lastFoundTargetTime = tap::arch::clock::getTimeMicroseconds();
                }

                // Auditory indicator that helps debug our vision pipeline.
                if (lastMessage.cvState == CVState::FOUND) {
                    tap::buzzer::playNote(&drivers->pwm, 466);
                } else if (lastMessage.cvState == CVState::FIRE) {
                    tap::buzzer::playNote(&drivers->pwm, 932);
                } else {
                    tap::buzzer::playNote(&drivers->pwm, 0);
                }

                // As we've received a full message, reset the byte index and go back to searching for the magic number.
                nextByteIndex = 0;
                currentSerialState = JetsonCommunicatorSerialState::SearchingForMagic;
            } else {
                rawSerialDisplay[nextByteIndex] = rawSerialBuffer[nextByteIndex];
            }

            break;
        }
    }

    // if (!isJetsonOnline()) {
    //     lastMessage.targetX = 0.0f;
    //     lastMessage.targetY = 0.0f;
    // }
}

PlateKinematicState JetsonCommunicator::getPlatePrediction(uint32_t dt) const {
    return visionDataConverter.getPlatePrediction(dt);
}

AutoAimAngles JetsonCommunicator::getAutoAimAngles() const {
    AutoAimAngles angles;
    angles.yaw = lastMessage.yaw;
    angles.pitch = lastMessage.pitch;
    return angles;
}

bool JetsonCommunicator::isLastFrameStale() const { return visionDataConverter.isLastFrameStale(); }

bool JetsonCommunicator::shouldFire() {
    if (!fireTimeout.isExpired()) { // still need to wait before we fire
        return false;
    }
    // We should fire
    fireTimeout.stop(); // stop timer so we don't keep sending true
    return true;
}

}  // namespace src::Informants::Vision
