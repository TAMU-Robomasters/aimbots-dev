#include "super_cap_communicator.hpp"

#include <drivers.hpp>

#include "tap/communication/sensors/buzzer/buzzer.hpp"

#define READ(data, length) drivers->uart.read(SUPER_CAP_UART_PORT, data, length)
#define WRITE(data, length) drivers->uart.write(SUPER_CAP_UART_PORT, data, length)

namespace src::Informants::SuperCap {

SuperCapCommunicator::SuperCapCommunicator(src::Drivers* drivers)
    : drivers(drivers),
      currentSerialState(SuperCapCommunicatorSerialState::SearchingForMagic),
      nextByteIndex(0),
      superCapOfflineTimeout(),
      lastMessage(),
      command(STOP),
      chargeValue(0)  //
{}

void SuperCapCommunicator::initialize() {
    superCapOfflineTimeout.restart(SUPER_CAP_OFFLINE_TIMEOUT_MILLISECONDS);
    drivers->uart.init<SUPER_CAP_UART_PORT, SUPER_CAP_BAUD_RATE>();
}

uint8_t displayBuffer[SUPER_CAP_MESSAGE_SIZE];
int displayBufIndex = 0;
int displayNextByteIndex = 0;
int lastMsgTimeDisplay = 0;
int msBetweenLastMessageDisplay = 0;

float voltage, current, state = 0;
float lVoltage;
float lPrecent;
float lPower;

float chargeValueR = 10;
float totalCharge = 0;

char currentCommand;
/**
 * @brief Supercap
 * blz give me break points
 */
void SuperCapCommunicator::updateSerial() {
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);  // attempts to pull one byte from the buffer
    if (bytesRead != 1) return;
    // restarts the timeout
    superCapOfflineTimeout.restart(SUPER_CAP_OFFLINE_TIMEOUT_MILLISECONDS);

    //
    displayBuffer[displayBufIndex] = rawSerialBuffer[0];  // copy byte to display buffer
    displayBufIndex =
        (displayBufIndex + 1) % SUPER_CAP_MESSAGE_SIZE;  // increment display index and wrap around if necessary

    switch (currentSerialState) {
        case SuperCapCommunicatorSerialState::SearchingForMagic: {
            // Check if the byte we just read is the byte we expected in the magic number.

            if (rawSerialBuffer[nextByteIndex] == ((SUPER_CAP_MESSAGE_RECIEVED_MAGIC >> (8 * nextByteIndex)) & 0xFF)) {
                currentSerialState = SuperCapCommunicatorSerialState::AssemblingMessage;
                // goes to the next byte
                nextByteIndex++;
            } else {
                // starts over
                nextByteIndex = 0;
            }

            if (nextByteIndex == sizeof(decltype(SUPER_CAP_MESSAGE_RECIEVED_MAGIC))) {
                // received a full message and we can now interpret it
                currentSerialState = SuperCapCommunicatorSerialState::AssemblingMessage;
            }
            displayNextByteIndex = nextByteIndex;
            break;
        }
        case SuperCapCommunicatorSerialState::AssemblingMessage: {
            nextByteIndex++;
            displayNextByteIndex = nextByteIndex;

            if (nextByteIndex == SUPER_CAP_MESSAGE_SIZE) {
                // received a full message and we can now interpret it
                //  puts it into lastMessage struct
                lastMessage = *reinterpret_cast<SuperCapMessageRecieved*>(rawSerialBuffer);

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
                currentSerialState = SuperCapCommunicatorSerialState::SearchingForMagic;
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
    totalCharge = lastSentMessage.charge;
    size_t len = static_cast<size_t>(sizeof(lastSentMessage));
    rawSerialBufferSent = reinterpret_cast<uint8_t*>(&lastSentMessage);
    // rawSerialBufferSent[0] = lastSentMessage.magic;
    // rawSerialBufferSent[1] = lastSentMessage.command;
    // rawSerialBufferSent[2] = lastSentMessage.charge;
    WRITE(rawSerialBufferSent, len);
}

}  // namespace src::Informants::SuperCap
