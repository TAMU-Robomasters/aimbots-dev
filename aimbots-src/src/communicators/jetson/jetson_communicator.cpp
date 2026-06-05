#include "jetson_communicator.hpp"
#include <cstdint>

#include <drivers.hpp>

#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "communicators/jetson/jetson_protocol.hpp"

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
      lastAimMessage(),
      lastLocalizationMessage(),
      transformationMessageToJetson({EMBEDDED_MESSAGE_MAGIC, {0}}),
      odometryMessageToJetson({EMBEDDED_MESSAGE_MAGIC, 0.0f, 0.0f, 0.0f})
{}

void JetsonCommunicator::initialize() {
    jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);
    drivers->uart.init<JETSON_UART_PORT, JETSON_BAUD_RATE>();
}

uint8_t displayBuffer[JETSON_MAX_MESSAGE_SIZE];
int displayBufIndex = 0;
size_t nextByteIndexDisplay = 0;
uint32_t timeDisplay = 0;

float targetYawDisplay = 0;
float targetPitchDisplay = 0;
uint16_t timeUntilNextFireDisplay = 0;
CVState cvStateDisplay = CVState::NO_TARGET;

float fieldRelativeYawAngleDisplay = 0;
float chassisRelativePitchAngleDisplay = 0;

int lastMsgTimeDisplay = 0;
int msBetweenLastMessageDisplay = 0;

float frameDelayOffsetDisplay_ms = 0.0f;
uint32_t fireTimeoutTimeRemainingDisplay = 0.0f;

uint8_t messageTypeDisplay = 0;

float odoXDisplay = 0;
float odoYDisplay = 0;
float odoThetaDisplay = 0;

float lidarXDisplay = 0.0f;
float lidarYDisplay = 0.0f;
float lidarThetaDisplay = 0.0f;

float velCmdXDisplay = 0.0f;
float velCmdYDisplay = 0.0f;

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
uint8_t rawSerialDisplay[JETSON_MAX_MESSAGE_SIZE];

void JetsonCommunicator::updateSerial() {
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    timeDisplay = currTime;

    fireTimeoutTimeRemainingDisplay = fireTimeout.timeRemaining();
    timeDisplay = currTime;

    fireTimeoutTimeRemainingDisplay = fireTimeout.timeRemaining();

    size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);  // attempts to pull one byte from the buffer
    if (bytesRead != 1) return;

    nextByteIndexDisplay = 1;
    nextByteIndexDisplay = 1;
    // We've successfully read a new byte from the Jetson, so we can restart this.
    jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);

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
        // ...received packet from Jetson, decide what to do
        case JetsonCommunicatorSerialState::HandleMessageType: {
            uint8_t messageType = rawSerialBuffer[nextByteIndex];
            messageTypeDisplay = messageType;
            if (messageType == JETSON_AIM_MESSAGE) {
                nextByteIndex++;
                currentSerialState = JetsonCommunicatorSerialState::AssemblingAimMessage;
            } 

            else if (messageType == JETSON_LOCALIZATION_MESSAGE) {
                nextByteIndex++;
                currentSerialState = JetsonCommunicatorSerialState::AssemblingLocalizationMessage;
            }

            else if (messageType == JETSON_VELOCITY_MESSAGE) {
                nextByteIndex++;
                currentSerialState = JetsonCommunicatorSerialState::AssemblingVelocityMessage;
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

                transformationMessageToJetson.yaw = drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue();
                transformationMessageToJetson.pitch = drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().getWrappedValue();
                
                std::memcpy(transformationMessageToJetson.matrix, cameraToTurret.element, sizeof(float) * 16);

                // Send data to Jetson
                WRITE((uint8_t*)&transformationMessageToJetson, sizeof(transformationMessageToJetson));

                // We responded to query from jetson, reset the byte index and go back to searching for the magic number.
                nextByteIndex = 0;
                currentSerialState = JetsonCommunicatorSerialState::SearchingForMagic;
            }
            else if (messageType == JETSON_ODOMETRY_QUERY) { // Respond to Query
                odometryMessageToJetson.x = drivers->kinematicInformant.getRobotLocation2D().getX();
                odometryMessageToJetson.y = drivers->kinematicInformant.getRobotLocation2D().getY();
                odometryMessageToJetson.theta = drivers->kinematicInformant.getGimbalIMUAngle(src::Informants::YAW_AXIS, AngleUnit::Radians);

                odoXDisplay = odometryMessageToJetson.x;
                odoYDisplay = odometryMessageToJetson.y;
                odoThetaDisplay = modm::toDegree(odometryMessageToJetson.theta);

                // Send data to Jetson
                WRITE((uint8_t*)&odometryMessageToJetson, sizeof(odometryMessageToJetson));

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
        case JetsonCommunicatorSerialState::AssemblingAimMessage: {
            nextByteIndex++;

            // Increment the byte index until we reach the expected end of a message, then parse the message.
            if (nextByteIndex == JETSON_AIM_MESSAGE_SIZE) {
                // Reinterpret the received bytes into a JetsonAimMessage
                std::memcpy(&lastAimMessage, rawSerialBuffer, sizeof(JetsonAimMessage));

                // Update last message time and time between last message and now.
                if (lastMsgTimeDisplay == 0) {
                    lastMsgTimeDisplay = tap::arch::clock::getTimeMilliseconds();
                } else {
                    msBetweenLastMessageDisplay =
                        currTime - lastMsgTimeDisplay;  // Should be pretty close to the message send rate.
                    lastMsgTimeDisplay = currTime;
                }

                targetYawDisplay = modm::toDegree(lastAimMessage.yaw);
                targetPitchDisplay = modm::toDegree(lastAimMessage.pitch);
                timeUntilNextFireDisplay = lastAimMessage.timeUntilNextFire;
                cvStateDisplay = lastAimMessage.cvState;
                if (lastAimMessage.cvState > CVState::NO_TARGET) {  // If the CV state is FOUND or better
                    // TODO: Explore using predictors to smoothen effect of large time gap between vision updates.

                    fireTimeout.restart(lastAimMessage.timeUntilNextFire);

                    lastFoundTargetTime = tap::arch::clock::getTimeMicroseconds();
                }

                // Auditory indicator that helps debug our vision pipeline.
                if (lastAimMessage.cvState == CVState::CONTINUOUS_FIRE) {
                    tap::buzzer::playNote(&drivers->pwm, 400);
                } else if (lastAimMessage.cvState == CVState::SHOT_TIMING) {
                    tap::buzzer::playNote(&drivers->pwm, 932);
                } else {
                    tap::buzzer::playNote(&drivers->pwm, 0);
                }

                // As we've received a full message, reset the byte index and go back to searching for the magic number.
                nextByteIndex = 0;
                currentSerialState = JetsonCommunicatorSerialState::SearchingForMagic;  
            }

            break;
        }

        case JetsonCommunicatorSerialState::AssemblingLocalizationMessage: {
            nextByteIndex++;

            // Increment the byte index until we reach the expected end of a message, then parse the message.
            if (nextByteIndex == JETSON_LOCALIZATION_MESSAGE_SIZE) {
                // Reinterpret the received bytes into a JetsonLocalizationMessage
                std::memcpy(&lastLocalizationMessage, rawSerialBuffer, sizeof(JetsonLocalizationMessage));
                        
                // Localization data from Jetson
                lidarXDisplay = lastLocalizationMessage.x;
                lidarYDisplay = lastLocalizationMessage.y;
                lidarThetaDisplay = modm::toDegree(lastLocalizationMessage.theta);

                // As we've received a full message, reset the byte index and go back to searching for the magic number.
                nextByteIndex = 0;
                currentSerialState = JetsonCommunicatorSerialState::SearchingForMagic;
            }

            break;
        }

        case JetsonCommunicatorSerialState::AssemblingVelocityMessage: {
            nextByteIndex++;

            // Increment the byte index until we reach the expected end of a message, then parse the message.
            if (nextByteIndex == JETSON_VELOCITY_MESSAGE_SIZE) {
                // Reinterpret the received bytes into a JetsonVelocityMessage
                std::memcpy(&lastVelocityMessage, rawSerialBuffer, sizeof(JetsonVelocityMessage));

                // Field-relative chassis velocity command from nav2 on the Jetson
                velCmdXDisplay = lastVelocityMessage.vx;
                velCmdYDisplay = lastVelocityMessage.vy;

                // As we've received a full message, reset the byte index and go back to searching for the magic number.
                nextByteIndex = 0;
                currentSerialState = JetsonCommunicatorSerialState::SearchingForMagic;
            }

            break;
        }
    }
}

PlateKinematicState JetsonCommunicator::getPlatePrediction(uint32_t dt) const {
    return visionDataConverter.getPlatePrediction(dt);
}

modm::Location2D<float> JetsonCommunicator::getLocationEstimate() const {
    return modm::Location2D(lastLocalizationMessage.x, lastLocalizationMessage.y, lastLocalizationMessage.theta);
}

AutoAimAngles JetsonCommunicator::getAutoAimAngles() const {
    AutoAimAngles angles;
    angles.yaw = lastAimMessage.yaw;
    angles.pitch = lastAimMessage.pitch;
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
