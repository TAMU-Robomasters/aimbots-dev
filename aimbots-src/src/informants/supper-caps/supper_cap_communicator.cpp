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

int lastMsgTimeDisplay = 0;
int msBetweenLastMessageDisplay = 0;

float voltage, current, state = 0;
float lVoltage;
float lPrecent;
float lPower;

char currentCommand;
/**
 * @brief Suppercap
 *
 */
void SupperCapCommunicator::updateSerial() {
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);  // attempts to pull one byte from the buffer
    if (bytesRead != 1) return;

    supperCapOfflineTimeout.restart(SUPPER_CAP_OFFLINE_TIMEOUT_MILLISECONDS);

    displayBuffer[displayBufIndex] = rawSerialBuffer[0];
    displayBufIndex = (displayBufIndex + 1) % SUPPER_CAP_MESSAGE_SIZE;

    switch (currentSerialState) {
        case SupperCapCommunicatorSerialState::SearchingForMagic:
            if (rawSerialBuffer[nextByteIndex] == ((SUPPER_CAP_MESSAGE_RECIEVED_MAGIC >> (8 * nextByteIndex)) & 0xFF)) {
                currentSerialState = SupperCapCommunicatorSerialState::AssemblingMessage;
                nextByteIndex++;
            } else {
                nextByteIndex = 0;
            }

            if (nextByteIndex == sizeof(decltype(SUPPER_CAP_MESSAGE_RECIEVED_MAGIC))) {
                currentSerialState = SupperCapCommunicatorSerialState::AssemblingMessage;
            }

            break;
        case SupperCapCommunicatorSerialState::AssemblingMessage:
            nextByteIndex++;

            if (nextByteIndex == SUPPER_CAP_MESSAGE_SIZE) {
                lastMessage = *reinterpret_cast<SupperCapMessageRecieved*>(rawSerialBuffer);
            }
            if (lastMsgTimeDisplay == 0) {
                lastMsgTimeDisplay = tap::arch::clock::getTimeMilliseconds();
            } else {
                msBetweenLastMessageDisplay =
                    currTime - lastMsgTimeDisplay;  // Should be pretty close to the message send rate.
                lastMsgTimeDisplay = currTime;
            }

            lVoltage = lastMessage.voltage;
            lPower = lastMessage.power;
            lPrecent = lastMessage.percent;

            nextByteIndex = 0;
            currentSerialState = SupperCapCommunicatorSerialState::SearchingForMagic;

            break;
    }
    currentCommand = command;
    switch (command) {
        case STOP:
            /* code */
            lastSentMessage.command = 's';
            lastSentMessage.charge = 0;
            break;
        case CHARGE:
            /* code */
            lastSentMessage.command = 'c';
            lastSentMessage.charge = 100;  // supperCapSubSystem->getCharge();
            break;
        case DISCHARGE:
            lastSentMessage.command = 'd';
            lastSentMessage.charge = 0;
        default:
            // notify("Invalid command");
            lastSentMessage.command = 's';
            lastSentMessage.charge = 0;
            break;
    }

    // rawSerialBufferSent = *reinterpret_cast<uint8_t*>(&lastSentMessage);
    rawSerialBufferSent[0] = lastSentMessage.magic;
    rawSerialBufferSent[1] = lastSentMessage.command;
    rawSerialBufferSent[2] = lastSentMessage.charge;
    WRITE(rawSerialBufferSent, SUPPER_CAP_MESSAGE_SENT_SIZE);
}

}  // namespace src::Informants::SupperCap
