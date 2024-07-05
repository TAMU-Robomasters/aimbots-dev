#include "jetson_communicator.hpp"

#include <drivers.hpp>

#include "tap/communication/sensors/buzzer/buzzer.hpp"

#define READ(data, length) drivers->uart.read(JETSON_UART_PORT, data, length)
#define WRITE(data, length) drivers->uart.write(JETSON_UART_PORT, data, length)

namespace src::Informants::Vision {

JetsonCommunicator::JetsonCommunicator(src::Drivers* drivers)
    : DJISerial(drivers,JETSON_UART_PORT),
      drivers(drivers),
      visionDataConverter(drivers),
      lastMessage(),
      fieldRelativeYawAngleAtVisionUpdate(0.0f),
      chassisRelativePitchAngleAtVisionUpdate(modm::toRadian(PITCH_AXIS_START_ANGLE))  //
{}

uint8_t displayBuffer[JETSON_MESSAGE_SIZE];
int displayBufIndex = 0;

float targetXDisplay = 0;
float targetYDisplay = 0;
CVState cvStateDisplay = CVState::NOT_FOUND;

float fieldRelativeYawAngleDisplay = 0;
float chassisRelativePitchAngleDisplay = 0;

int lastMsgTimeDisplay = 0;
int msBetweenLastMessageDisplay = 0;

void JetsonCommunicator::initialize(){
    jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MS);
    drivers->uart.init<JETSON_UART_PORT, JETSON_BAUD_RATE>();
}

/** @todo Implement messageReceiveCallback 
*
*/
void JetsonCommunicator::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
{
    jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MS);
    switch (completeMessage.messageType)
    {
    case RX_TARGET_DATA:
    //todo
    default:
        break;
    }
}

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

// void JetsonCommunicator::updateSerial() {
//     uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

//     size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);  // attempts to pull one byte from the buffer
//     if (bytesRead != 1) return;

//     // We've successfully read a new byte from the Jetson, so we can restart this.
//     jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);

//     displayBuffer[displayBufIndex] = rawSerialBuffer[0];            // copy byte to display buffer
//     displayBufIndex = (displayBufIndex + 1) % JETSON_MESSAGE_SIZE;  // increment display index and wrap around if necessary

//     switch (currentSerialState) {
//         // ...looking for message start...
//         case JetsonCommunicatorSerialState::SearchingForMagic: {
//             // Check if the byte we just read is the byte we expected in the magic number.
//             if (rawSerialBuffer[nextByteIndex] == ((JETSON_MESSAGE_MAGIC >> (8 * nextByteIndex)) & 0xff)) {
//                 nextByteIndex++;
//             } else {
//                 nextByteIndex = 0;  // if not, reset the index and start over.
//             }

//             // Wait until we've reached the end of the magic number. If any of the bytes in the magic number weren't a match,
//             // we wouldn't have gotten this far.
//             if (nextByteIndex == sizeof(decltype(JETSON_MESSAGE_MAGIC))) {
//                 currentSerialState = JetsonCommunicatorSerialState::AssemblingMessage;
//             }
//             break;
//         }
//         // ...found message start, assemble message...
//         case JetsonCommunicatorSerialState::AssemblingMessage: {
//             nextByteIndex++;

//             // Increment the byte index until we reach the expected end of a message, then parse the message.
//             if (nextByteIndex == JETSON_MESSAGE_SIZE) {
//                 // Reinterpret the received bytes into a JetsonMessage
//                 lastMessage = *reinterpret_cast<JetsonMessage*>(&rawSerialBuffer);

//                 // Update last message time and time between last message and now.
//                 if (lastMsgTimeDisplay == 0) {
//                     lastMsgTimeDisplay = tap::arch::clock::getTimeMilliseconds();
//                 } else {
//                     msBetweenLastMessageDisplay =
//                         currTime - lastMsgTimeDisplay;  // Should be pretty close to the message send rate.
//                     lastMsgTimeDisplay = currTime;
//                 }

//                 targetXDisplay = lastMessage.targetX;
//                 targetYDisplay = lastMessage.targetY;
//                 cvStateDisplay = lastMessage.cvState;

//                 if (lastMessage.cvState >= CVState::FOUND) {  // If the CV state is FOUND or better
//                     // TODO: Explore using predictors to smoothen effect of large time gap between vision updates.

//                     // position is relative to camera
//                     visionTargetPosition.setX(lastMessage.targetX);
//                     visionTargetPosition.setY(lastMessage.targetY);
//                     visionTargetPosition.setZ(lastMessage.targetZ);

//                     visionDataConverter.updateTargetInfo(visionTargetPosition, lastMessage.delay);
//                     lastFoundTargetTime = tap::arch::clock::getTimeMicroseconds();
//                 }

//                 // Auditory indicator that helps debug our vision pipeline.
//                 if (lastMessage.cvState == CVState::FOUND) {
//                     tap::buzzer::playNote(&drivers->pwm, 466);
//                 } else if (lastMessage.cvState == CVState::FIRE) {
//                     tap::buzzer::playNote(&drivers->pwm, 932);
//                 } else {
//                     tap::buzzer::playNote(&drivers->pwm, 0);
//                 }

//                 // As we've received a full message, reset the byte index and go back to searching for the magic number.
//                 nextByteIndex = 0;
//                 currentSerialState = JetsonCommunicatorSerialState::SearchingForMagic;
//             } else {
//                 rawSerialDisplay[nextByteIndex] = rawSerialBuffer[nextByteIndex];
//             }

//             break;
//         }
//     }

//     // if (!isJetsonOnline()) {
//     //     lastMessage.targetX = 0.0f;
//     //     lastMessage.targetY = 0.0f;
//     // }
// }

PlateKinematicState JetsonCommunicator::getPlatePrediction(uint32_t dt) const {
    return visionDataConverter.getPlatePrediction(dt);
}

bool JetsonCommunicator::isLastFrameStale() const { return visionDataConverter.isLastFrameStale(); }

/**
 * @brief If more data is needed to be sent over UART to the Jetson, this function will 
 * need to be updated to accomdate new message types
 */
void JetsonCommunicator::sendMessage()
{
    sendRobotID();
    sendMatchState();
    sendMatchInfo();
}


void JetsonCommunicator::sendRobotID()
{
    if (robotIdTimeoutTX.execute()) {
        DJISerial::SerialMessage<1> robotIdMessage;
        robotIdMessage.messageType = TX_MESSAGE_ROBOT_ID;
        robotIdMessage.data[0] = static_cast<uint8_t>(drivers->refSerial.getRobotData().robotId);
        robotIdMessage.setCRC16();
        WRITE(reinterpret_cast<uint8_t*>(&robotIdMessage),sizeof(robotIdMessage));
    }
}

void JetsonCommunicator::sendMatchInfo()
{
    if (matchInfoTimeoutTX.execute())
    {
        struct RefGameTimeData 
        {
            uint8_t gameType;
            uint8_t gameProgress; // what stage of the match (mid-round, initialization, etc.)
            uint16_t gameRemainTime;
        } modm_packed;

        DJISerial::SerialMessage<sizeof(RefGameTimeData)> gameMessage;

        gameMessage.messageType = TX_MESSAGE_MATCH_INFO;

        RefGameTimeData* data = reinterpret_cast<RefGameTimeData*>(gameMessage.data);

        const auto& gameData = drivers->refSerial.getGameData();
        
        data->gameType = static_cast<uint8_t>(gameData.gameType);
        data->gameProgress = static_cast<uint8_t>(gameData.gameStage);
        data->gameRemainTime = static_cast<uint8_t>(gameData.stageTimeRemaining);

        gameMessage.setCRC16();
        WRITE(reinterpret_cast<uint8_t*>(&gameMessage),sizeof(gameMessage));
    }
}

}  // namespace src::Informants::Vision
