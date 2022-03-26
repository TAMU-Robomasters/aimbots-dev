/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "ref_serial.hpp"

#include "tap/algorithms/crc.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/architecture/endianness_wrappers.hpp"
#include "tap/communication/serial/ref_serial_constants.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

using namespace tap::arch;

namespace tap::communication::serial
{
RefSerial::RefSerial(Drivers* drivers)
    : DJISerial(drivers, bound_ports::REF_SERIAL_UART_PORT),
      robotData(),
      gameData(),
      receivedDpsTracker()
{
    refSerialOfflineTimeout.stop();
}

bool RefSerial::getRefSerialReceivingData() const
{
    return !(refSerialOfflineTimeout.isStopped() || refSerialOfflineTimeout.isExpired());
}

void RefSerial::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
{
    refSerialOfflineTimeout.restart(TIME_OFFLINE_REF_DATA_MS);

    updateReceivedDamage();
    switch (completeMessage.messageType)
    {
        case REF_MESSAGE_TYPE_GAME_STATUS:
        {
            decodeToGameStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_GAME_RESULT:
        {
            decodeToGameResult(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ALL_ROBOT_HP:
        {
            decodeToAllRobotHP(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_STATUS:
        {
            decodeToRobotStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_POWER_AND_HEAT:
        {
            decodeToPowerAndHeat(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_POSITION:
        {
            decodeToRobotPosition(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_BUFF_STATUS:
        {
            decodeToRobotBuffs(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_AERIAL_ENERGY_STATUS:
        {
            decodeToAerialEnergyStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_RECEIVE_DAMAGE:
        {
            decodeToDamageStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_PROJECTILE_LAUNCH:
        {
            decodeToProjectileLaunch(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_BULLETS_REMAIN:
        {
            decodeToBulletsRemain(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_RFID_STATUS:
        {
            decodeToRFIDStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_CUSTOM_DATA:
        {
            handleRobotToRobotCommunication(completeMessage);
            break;
        }
        default:
            break;
    }
}

const RefSerialData::Rx::RobotData& RefSerial::getRobotData() const { return robotData; }

const RefSerialData::Rx::GameData& RefSerial::getGameData() const { return gameData; }

bool RefSerial::decodeToGameStatus(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 11)
    {
        return false;
    }
    // Ignore competition type, bits [0-3] of the first byte
    gameData.gameStage = static_cast<Rx::GameStage>(0xf & (message.data[0] >> 4));
    convertFromLittleEndian(&gameData.stageTimeRemaining, message.data + 1);
    // Ignore Unix time sent
    return true;
}

bool RefSerial::decodeToGameResult(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 1)
    {
        return false;
    }
    gameData.gameWinner = static_cast<Rx::GameWinner>(message.data[0]);
    return true;
}

bool RefSerial::decodeToAllRobotHP(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 32)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.allRobotHp.red.hero1, message.data);
    convertFromLittleEndian(&robotData.allRobotHp.red.engineer2, message.data + 2);
    convertFromLittleEndian(&robotData.allRobotHp.red.standard3, message.data + 4);
    convertFromLittleEndian(&robotData.allRobotHp.red.standard4, message.data + 6);
    convertFromLittleEndian(&robotData.allRobotHp.red.standard5, message.data + 8);
    convertFromLittleEndian(&robotData.allRobotHp.red.sentry7, message.data + 10);
    convertFromLittleEndian(&robotData.allRobotHp.red.outpost, message.data + 12);
    convertFromLittleEndian(&robotData.allRobotHp.red.base, message.data + 14);

    convertFromLittleEndian(&robotData.allRobotHp.blue.hero1, message.data + 16);
    convertFromLittleEndian(&robotData.allRobotHp.blue.engineer2, message.data + 18);
    convertFromLittleEndian(&robotData.allRobotHp.blue.standard3, message.data + 20);
    convertFromLittleEndian(&robotData.allRobotHp.blue.standard4, message.data + 22);
    convertFromLittleEndian(&robotData.allRobotHp.blue.standard5, message.data + 24);
    convertFromLittleEndian(&robotData.allRobotHp.blue.sentry7, message.data + 26);
    convertFromLittleEndian(&robotData.allRobotHp.blue.outpost, message.data + 28);
    convertFromLittleEndian(&robotData.allRobotHp.blue.base, message.data + 30);

    return true;
}

bool RefSerial::decodeToSiteEventData(const ReceivedSerialMessage&) { return false; }

bool RefSerial::decodeToRobotStatus(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 27)
    {
        return false;
    }
    robotData.robotId = static_cast<RobotId>(message.data[0]);
    robotData.robotLevel = message.data[1];
    convertFromLittleEndian(&robotData.currentHp, message.data + 2);
    convertFromLittleEndian(&robotData.maxHp, message.data + 4);
    convertFromLittleEndian(&robotData.turret.heatCoolingRate17ID1, message.data + 6);
    convertFromLittleEndian(&robotData.turret.heatLimit17ID1, message.data + 8);
    convertFromLittleEndian(&robotData.turret.barrelSpeedLimit17ID1, message.data + 10);
    convertFromLittleEndian(&robotData.turret.heatCoolingRate17ID2, message.data + 12);
    convertFromLittleEndian(&robotData.turret.heatLimit17ID2, message.data + 14);
    convertFromLittleEndian(&robotData.turret.barrelSpeedLimit17ID2, message.data + 16);
    convertFromLittleEndian(&robotData.turret.heatCoolingRate42, message.data + 18);
    convertFromLittleEndian(&robotData.turret.heatLimit42, message.data + 20);
    convertFromLittleEndian(&robotData.turret.barrelSpeedLimit42, message.data + 22);
    convertFromLittleEndian(&robotData.chassis.powerConsumptionLimit, message.data + 24);
    robotData.robotPower.value = message.data[26] & 0b111;
    robotData.robotDataReceivedTimestamp = clock::getTimeMilliseconds();

    processReceivedDamage(
        robotData.robotDataReceivedTimestamp,
        static_cast<int>(robotData.previousHp) - static_cast<int>(robotData.currentHp));
    robotData.previousHp = robotData.currentHp;

    return true;
}

bool RefSerial::decodeToPowerAndHeat(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 16)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.chassis.volt, message.data);
    convertFromLittleEndian(&robotData.chassis.current, message.data + 2);
    convertFromLittleEndian(&robotData.chassis.power, message.data + 4);
    convertFromLittleEndian(&robotData.chassis.powerBuffer, message.data + 8);
    convertFromLittleEndian(&robotData.turret.heat17ID1, message.data + 10);
    convertFromLittleEndian(&robotData.turret.heat17ID2, message.data + 12);
    convertFromLittleEndian(&robotData.turret.heat42, message.data + 14);
    return true;
}

bool RefSerial::decodeToRobotPosition(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 16)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.chassis.x, message.data);
    convertFromLittleEndian(&robotData.chassis.y, message.data + 4);
    convertFromLittleEndian(&robotData.chassis.z, message.data + 8);
    convertFromLittleEndian(&robotData.turret.yaw, message.data + 12);
    return true;
}

bool RefSerial::decodeToRobotBuffs(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 1)
    {
        return false;
    }
    robotData.robotBuffStatus.value = message.data[0] & 0b1111;
    return true;
}

bool RefSerial::decodeToAerialEnergyStatus(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 2)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.aerialEnergyStatus, message.data);
    return true;
}

bool RefSerial::decodeToDamageStatus(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 1)
    {
        return false;
    }
    robotData.damagedArmorId = static_cast<Rx::ArmorId>((message.data[0]) & 0xf);
    robotData.damageType = static_cast<Rx::DamageType>((message.data[0] >> 4) & 0xf);
    return true;
}

bool RefSerial::decodeToProjectileLaunch(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 7)
    {
        return false;
    }
    robotData.turret.bulletType = static_cast<Rx::BulletType>(message.data[0]);
    robotData.turret.launchMechanismID = static_cast<Rx::MechanismID>(message.data[1]);
    robotData.turret.firingFreq = message.data[2];
    robotData.turret.lastReceivedLaunchingInfoTimestamp = clock::getTimeMilliseconds();
    convertFromLittleEndian(&robotData.turret.bulletSpeed, message.data + 3);
    return true;
}

bool RefSerial::decodeToBulletsRemain(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 6)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.turret.bulletsRemaining17, message.data);
    convertFromLittleEndian(&robotData.turret.bulletsRemaining42, message.data + 2);
    convertFromLittleEndian(&robotData.remainingCoins, message.data + 4);
    return true;
}

bool RefSerial::decodeToRFIDStatus(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength != 4)
    {
        return false;
    }
    robotData.rfidStatus.value = message.data[0];
    return true;
}

bool RefSerial::handleRobotToRobotCommunication(const ReceivedSerialMessage& message)
{
    if (message.header.dataLength < sizeof(Tx::RobotToRobotMessage::interactiveHeader))
    {
        return false;
    }

    if (msgIdToRobotToRobotHandlerMap.size() == 0)
    {
        return true;
    }

    const Tx::InteractiveHeader* interactiveHeader =
        reinterpret_cast<const Tx::InteractiveHeader*>(message.data);

    if (msgIdToRobotToRobotHandlerMap.count(interactiveHeader->dataCmdId))
    {
        (*msgIdToRobotToRobotHandlerMap[interactiveHeader->dataCmdId])(message);
    }

    return true;
}

void RefSerial::processReceivedDamage(uint32_t timestamp, int32_t damageTaken)
{
    if (damageTaken > 0)
    {
        // create a new DamageEvent with the damage_taken, and current time
        Rx::DamageEvent damageEvent = {static_cast<uint16_t>(damageTaken), timestamp};

        if (receivedDpsTracker.getSize() == DPS_TRACKER_DEQUE_SIZE)
        {
            receivedDpsTracker.removeBack();
        }
        robotData.receivedDps += damageTaken;

        receivedDpsTracker.append(damageEvent);
    }
}

void RefSerial::updateReceivedDamage()
{
    // if current damage at head of circular array occurred more than a second ago,
    // decrease receivedDps by that amount of damage and increment head index
    while (receivedDpsTracker.getSize() > 0 &&
           clock::getTimeMilliseconds() > receivedDpsTracker.getFront().timestampMs + 1000)
    {
        robotData.receivedDps -= receivedDpsTracker.getFront().damageAmount;
        receivedDpsTracker.removeFront();
    }
}

void RefSerial::configGraphicGenerics(
    Tx::GraphicData* graphicData,
    const uint8_t* name,
    Tx::AddGraphicOperation operation,
    uint8_t layer,
    Tx::GraphicColor color)
{
    memcpy(graphicData->name, name, 3);
    graphicData->operation = operation;
    graphicData->layer = layer;
    graphicData->color = static_cast<uint8_t>(color);
}

void RefSerial::configLine(
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    uint16_t endX,
    uint16_t endY,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::STRAIGHT_LINE);
    sharedData->lineWidth = width;
    sharedData->startX = startX;
    sharedData->startY = startY;
    sharedData->endX = endX;
    sharedData->endY = endY;
}

void RefSerial::configRectangle(
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    uint16_t endX,
    uint16_t endY,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::RECTANGLE);
    sharedData->lineWidth = width;
    sharedData->startX = startX;
    sharedData->startY = startY;
    sharedData->endX = endX;
    sharedData->endY = endY;
}

void RefSerial::configCircle(
    uint16_t width,
    uint16_t centerX,
    uint16_t centerY,
    uint16_t radius,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::CIRCLE);
    sharedData->lineWidth = width;
    sharedData->startX = centerX;
    sharedData->startY = centerY;
    sharedData->radius = radius;
}

void RefSerial::configEllipse(
    uint16_t width,
    uint16_t centerX,
    uint16_t centerY,
    uint16_t xLen,
    uint16_t yLen,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::ELLIPSE);
    sharedData->lineWidth = width;
    sharedData->startX = centerX;
    sharedData->startY = centerY;
    sharedData->endX = xLen;
    sharedData->endY = yLen;
}

void RefSerial::configArc(
    uint16_t startAngle,
    uint16_t endAngle,
    uint16_t width,
    uint16_t centerX,
    uint16_t centerY,
    uint16_t xLen,
    uint16_t yLen,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::ARC);
    sharedData->startAngle = startAngle;
    sharedData->endAngle = endAngle;
    sharedData->lineWidth = width;
    sharedData->startX = centerX;
    sharedData->startY = centerY;
    sharedData->endX = xLen;
    sharedData->endY = yLen;
}

void RefSerial::configFloatingNumber(
    uint16_t fontSize,
    uint16_t decimalPrecision,
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    float value,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::FLOATING_NUM);
    sharedData->startAngle = fontSize;
    sharedData->endAngle = decimalPrecision;
    sharedData->lineWidth = width;
    sharedData->startX = startX;
    sharedData->startY = startY;
    // Store floating point value in fixed point with 3 decimal points precision
    sharedData->value = 1000 * value;
}

void RefSerial::configInteger(
    uint16_t fontSize,
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    int32_t value,
    Tx::GraphicData* sharedData)
{
    sharedData->type = static_cast<uint8_t>(Tx::GraphicType::INTEGER);
    sharedData->startAngle = fontSize;
    sharedData->lineWidth = width;
    sharedData->startX = startX;
    sharedData->startY = startY;
    sharedData->value = value;
}

void RefSerial::configCharacterMsg(
    uint16_t fontSize,
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    const char* dataToPrint,
    Tx::GraphicCharacterMessage* sharedData)
{
    sharedData->graphicData.type = static_cast<uint8_t>(Tx::GraphicType::CHARACTER);
    sharedData->graphicData.startAngle = fontSize;
    sharedData->graphicData.endAngle = strlen(dataToPrint);
    sharedData->graphicData.lineWidth = width;
    sharedData->graphicData.startX = startX;
    sharedData->graphicData.startY = startY;
    strncpy(sharedData->msg, dataToPrint, MODM_ARRAY_SIZE(sharedData->msg) - 1);
}

/**
 * Given RobotId, returns the client_id that the referee system uses to display
 * the received messages to the given client_id robot.
 *
 * @param[in] robotId the id of the robot received from the referee system
 *      to get the client_id of.
 * @return the client_id of the robot requested.
 */
static uint16_t getRobotClientID(RefSerial::RobotId robotId)
{
    return 0x100 + static_cast<uint16_t>(robotId);
}

void RefSerial::deleteGraphicLayer(
    Tx::DeleteGraphicOperation graphicOperation,
    uint8_t graphicLayer)
{
    if (robotData.robotId == RefSerial::RobotId::INVALID)
    {
        return;
    }

    Tx::DeleteGraphicLayerMessage msg;
    msg.deleteOperation = graphicOperation;
    msg.layer = graphicLayer;

    configFrameHeader(
        &msg.frameHeader,
        sizeof(msg.interactiveHeader) + sizeof(msg.deleteOperation) + sizeof(msg.layer));

    msg.cmdId = REF_MESSAGE_TYPE_CUSTOM_DATA;

    configInteractiveHeader(
        &msg.interactiveHeader,
        0x100,
        robotData.robotId,
        getRobotClientID(robotData.robotId));

    msg.crc16 = algorithms::calculateCRC16(
        reinterpret_cast<uint8_t*>(&msg),
        sizeof(Tx::DeleteGraphicLayerMessage) - sizeof(msg.crc16));

    drivers->uart.write(
        bound_ports::REF_SERIAL_UART_PORT,
        reinterpret_cast<uint8_t*>(&msg),
        sizeof(Tx::DeleteGraphicLayerMessage));
}

/**
 * A helper function used by the RefSerial's sendGraphic functions to send some GraphicType
 * to the referee system. The user may specify whether or not to configure the message header
 * and whether or not the actually send the message. This helper function is needed because the
 * sendGraphic functions all send messages the same way with only minor differences.
 */
template <typename GraphicType>
static void sendGraphicHelper(
    GraphicType* graphicMsg,
    uint16_t cmdId,
    bool configMsgHeader,
    bool sendMsg,
    RefSerial::RobotId robotId,
    Drivers* drivers,
    int extraDataLength = 0)
{
    if (robotId == RefSerial::RobotId::INVALID)
    {
        return;
    }

    if (configMsgHeader)
    {
        RefSerial::configFrameHeader(
            &graphicMsg->frameHeader,
            sizeof(graphicMsg->graphicData) + sizeof(graphicMsg->interactiveHeader) +
                extraDataLength);

        graphicMsg->cmdId = RefSerial::REF_MESSAGE_TYPE_CUSTOM_DATA;

        RefSerial::configInteractiveHeader(
            &graphicMsg->interactiveHeader,
            cmdId,
            robotId,
            getRobotClientID(robotId));

        graphicMsg->crc16 = algorithms::calculateCRC16(
            reinterpret_cast<uint8_t*>(graphicMsg),
            sizeof(GraphicType) - sizeof(graphicMsg->crc16));
    }

    if (sendMsg)
    {
        drivers->uart.write(
            bound_ports::REF_SERIAL_UART_PORT,
            reinterpret_cast<uint8_t*>(graphicMsg),
            sizeof(GraphicType));
    }
}
void RefSerial::sendGraphic(Tx::Graphic1Message* graphicMsg, bool configMsgHeader, bool sendMsg)
{
    sendGraphicHelper(graphicMsg, 0x101, configMsgHeader, sendMsg, robotData.robotId, drivers);
}

void RefSerial::sendGraphic(Tx::Graphic2Message* graphicMsg, bool configMsgHeader, bool sendMsg)
{
    sendGraphicHelper(graphicMsg, 0x102, configMsgHeader, sendMsg, robotData.robotId, drivers);
}

void RefSerial::sendGraphic(Tx::Graphic5Message* graphicMsg, bool configMsgHeader, bool sendMsg)
{
    sendGraphicHelper(graphicMsg, 0x103, configMsgHeader, sendMsg, robotData.robotId, drivers);
}

void RefSerial::sendGraphic(Tx::Graphic7Message* graphicMsg, bool configMsgHeader, bool sendMsg)
{
    sendGraphicHelper(graphicMsg, 0x104, configMsgHeader, sendMsg, robotData.robotId, drivers);
}

void RefSerial::sendGraphic(
    Tx::GraphicCharacterMessage* graphicMsg,
    bool configMsgHeader,
    bool sendMsg)
{
    sendGraphicHelper(
        graphicMsg,
        0x110,
        configMsgHeader,
        sendMsg,
        robotData.robotId,
        drivers,
        MODM_ARRAY_SIZE(graphicMsg->msg));
}

void RefSerial::sendRobotToRobotMsg(
    Tx::RobotToRobotMessage* robotToRobotMsg,
    uint16_t msgId,
    RobotId receiverId,
    uint16_t msgLen)
{
    if (msgId < 0x0200 || msgId >= 0x02ff)
    {
        RAISE_ERROR(drivers, "invalid msgId not betweene [0x200, 0x2ff)");
        return;
    }

    if (msgLen > 113)
    {
        RAISE_ERROR(drivers, "message length > 113-char maximum");
    }

    if (robotData.robotId == RobotId::INVALID)
    {
        return;
    }

    configFrameHeader(
        &robotToRobotMsg->frameHeader,
        sizeof(robotToRobotMsg->interactiveHeader) + msgLen);

    robotToRobotMsg->cmdId = REF_MESSAGE_TYPE_CUSTOM_DATA;

    configInteractiveHeader(
        &robotToRobotMsg->interactiveHeader,
        msgId,
        robotData.robotId,
        static_cast<uint16_t>(receiverId));

    uint16_t msgSizeToCRC16 = sizeof(robotToRobotMsg->frameHeader) +
                              sizeof(robotToRobotMsg->cmdId) +
                              sizeof(robotToRobotMsg->interactiveHeader) + msgLen;
    uint16_t* crc16Location = reinterpret_cast<uint16_t*>(robotToRobotMsg->dataAndCRC16 + msgLen);

    *crc16Location =
        algorithms::calculateCRC16(reinterpret_cast<uint8_t*>(robotToRobotMsg), msgSizeToCRC16);

    drivers->uart.write(
        bound_ports::REF_SERIAL_UART_PORT,
        reinterpret_cast<uint8_t*>(robotToRobotMsg),
        msgSizeToCRC16 + sizeof(uint16_t));
}

void RefSerial::configFrameHeader(DJISerial::FrameHeader* header, uint16_t msgLen)
{
    header->headByte = 0xa5;
    header->dataLength = msgLen;
    header->seq = 0;
    header->CRC8 = algorithms::calculateCRC8(
        reinterpret_cast<const uint8_t*>(header),
        sizeof(DJISerial::FrameHeader) - sizeof(header->CRC8));
}

void RefSerial::configInteractiveHeader(
    Tx::InteractiveHeader* header,
    uint16_t cmdId,
    RobotId senderId,
    uint16_t receiverId)
{
    header->dataCmdId = cmdId;
    header->senderId = static_cast<uint16_t>(senderId);
    header->receiverId = receiverId;
}

RefSerial::RobotId RefSerial::getRobotIdBasedOnCurrentRobotTeam(RobotId id)
{
    if (id == RobotId::INVALID || robotData.robotId == RobotId::INVALID)
    {
        return id;
    }

    if (!isBlueTeam(robotData.robotId) && isBlueTeam(id))
    {
        id = id - RobotId::BLUE_HERO + RobotId::RED_HERO;
    }
    else if (isBlueTeam(robotData.robotId) && !isBlueTeam(id))
    {
        id = id - RobotId::RED_HERO + RobotId::BLUE_HERO;
    }
    return id;
}

void RefSerial::attachRobotToRobotMessageHandler(
    uint16_t msgId,
    RobotToRobotMessageHandler* handler)
{
    if (msgIdToRobotToRobotHandlerMap.count(msgId) != 0 || msgId < 0x0200 || msgId > 0x02FF)
    {
        RAISE_ERROR(drivers, "error adding msg handler");
        return;
    }

    msgIdToRobotToRobotHandlerMap[msgId] = handler;
}

}  // namespace tap::communication::serial
