#include "supper_cap_communicator.hpp"

#include <drivers.hpp>

#include "tap/communication/sensors/buzzer/buzzer.hpp"

#define READ(data, length) drivers->uart.read(SUPPER_CAP_UART_PORT, data, length)
#define WRITE(data, length) drivers->uart.write(SUPPER_CAP_UART_PORT, data, length)

namespace src::Informants::SupperCap {

SupperCapCommunicator::SupperCapCommunicator(src::Drivers* drivers)
    : drivers(drivers),
      currentSerialState(SupperCapCommunicatorSerialState::SearchingForMagic),
      nextByteIndex(0),
      supperCapOfflineTimeout(),
      lastMessage(),
      command(STOP),
      chargeValue(0)  //
{}

void SupperCapCommunicator::initialize() {
    supperCapOfflineTimeout.restart(SUPPER_CAP_OFFLINE_TIMEOUT_MILLISECONDS);
    drivers->uart.init<SUPPER_CAP_UART_PORT, SUPPER_CAP_BAUD_RATE>();
}

uint8_t displayBuffer[SUPPER_CAP_MESSAGE_SIZE];
int displayBufIndex = 0;
int displayNextByteIndex = 0;
int lastMsgTimeDisplay = 0;
int msBetweenLastMessageDisplay = 0;

float voltage, current, state = 0;
float lVoltage;
float lPrecent;
float lPower;

float chargeValueR = 10;

char currentCommand;
/**
 * @brief Suppercap
 * blz give me break points
 */
void SupperCapCommunicator::updateSerial() {
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);  // attempts to pull one byte from the buffer
    if (bytesRead != 1) return;
    // restarts the timeout
    supperCapOfflineTimeout.restart(SUPPER_CAP_OFFLINE_TIMEOUT_MILLISECONDS);

    //
    displayBuffer[displayBufIndex] = rawSerialBuffer[0];  // copy byte to display buffer
    displayBufIndex =
        (displayBufIndex + 1) % SUPPER_CAP_MESSAGE_SIZE;  // increment display index and wrap around if necessary

    switch (currentSerialState) {
        case SupperCapCommunicatorSerialState::SearchingForMagic: {
            // Check if the byte we just read is the byte we expected in the magic number.

            if (rawSerialBuffer[nextByteIndex] == ((SUPPER_CAP_MESSAGE_RECIEVED_MAGIC >> (8 * nextByteIndex)) & 0xFF)) {
                currentSerialState = SupperCapCommunicatorSerialState::AssemblingMessage;
                // goes to the next byte
                nextByteIndex++;
            } else {
                // starts over
                nextByteIndex = 0;
            }

            if (nextByteIndex == sizeof(decltype(SUPPER_CAP_MESSAGE_RECIEVED_MAGIC))) {
                // received a full message and we can now interpret it
                currentSerialState = SupperCapCommunicatorSerialState::AssemblingMessage;
            }
            displayNextByteIndex = nextByteIndex;
            break;
        }
        case SupperCapCommunicatorSerialState::AssemblingMessage: {
            nextByteIndex++;
            displayNextByteIndex = nextByteIndex;

            if (nextByteIndex == SUPPER_CAP_MESSAGE_SIZE) {
                // received a full message and we can now interpret it
                //  puts it into lastMessage struct
                lastMessage = *reinterpret_cast<SupperCapMessageRecieved*>(rawSerialBuffer);

                if (lastMsgTimeDisplay == 0) {
                    // gets time
                    lastMsgTimeDisplay = tap::arch::clock::getTimeMilliseconds();
                } else {
                    msBetweenLastMessageDisplay =
                        currTime - lastMsgTimeDisplay;  // Should be pretty close to the message send rate.
                    lastMsgTimeDisplay = currTime;
                }

                // test values
                lVoltage = lastMessage.voltage;
                lPower = lastMessage.power;
                lPrecent = lastMessage.percent;

                nextByteIndex = 0;
                currentSerialState = SupperCapCommunicatorSerialState::SearchingForMagic;
            }
            break;
        }
    }
    displayNextByteIndex = nextByteIndex;
    currentCommand = command;
    lastSentMessage.magic = 'b';
    lastSentMessage.command = 'c';
    chargeValueR = 10;
    lastSentMessage.charge = chargeValueR+lastSentMessage.charge;
    size_t len = static_cast<size_t>(sizeof(lastSentMessage));
    rawSerialBufferSent = reinterpret_cast<uint8_t*>(&lastSentMessage);
    // rawSerialBufferSent[0] = lastSentMessage.magic;
    // rawSerialBufferSent[1] = lastSentMessage.command;
    // rawSerialBufferSent[2] = lastSentMessage.charge;
    WRITE(rawSerialBufferSent, len);
}

}  // namespace src::Informants::SupperCap
