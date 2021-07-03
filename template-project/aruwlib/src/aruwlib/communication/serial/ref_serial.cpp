/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "ref_serial.hpp"

#include "aruwlib/algorithms/crc.hpp"
#include "aruwlib/architecture/clock.hpp"
#include "aruwlib/architecture/endianness_wrappers.hpp"
#include "aruwlib/drivers.hpp"

using namespace aruwlib::arch;

namespace aruwlib
{
namespace serial
{
RefSerial::RefSerial(Drivers* drivers)
    : DJISerial(drivers, Uart::UartPort::Uart6),
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

// rx stuff
void RefSerial::messageReceiveCallback(const SerialMessage& completeMessage)
{
    refSerialOfflineTimeout.restart(TIME_OFFLINE_REF_DATA_MS);

    updateReceivedDamage();
    switch (completeMessage.type)
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
        case REF_MESSAGE_TYPE_RECEIVE_DAMAGE:
        {
            decodeToReceiveDamage(completeMessage);
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
        default:
            break;
    }
}

uint16_t RefSerial::getRobotClientID(uint16_t robotId) { return 0x100 + robotId; }

const RefSerial::RobotData& RefSerial::getRobotData() const { return robotData; }

const RefSerial::GameData& RefSerial::getGameData() const { return gameData; }

bool RefSerial::decodeToGameStatus(const SerialMessage& message)
{
    if (message.length != 11)
    {
        return false;
    }
    // Ignore competition type, bits [0-3] of the first byte
    gameData.gameStage = static_cast<GameStages>(message.data[0] >> 4);
    convertFromLittleEndian(&gameData.stageTimeRemaining, message.data + 1);
    // Ignore Unix time sent
    return true;
}

bool RefSerial::decodeToGameResult(const SerialMessage& message)
{
    if (message.length != 1)
    {
        return false;
    }
    gameData.gameWinner = static_cast<GameWinner>(message.data[0]);
    return true;
}

bool RefSerial::decodeToAllRobotHP(const SerialMessage& message)
{
    if (message.length != 28)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.allRobotHp.redHero, message.data);
    convertFromLittleEndian(&robotData.allRobotHp.redEngineer, message.data + 2);
    convertFromLittleEndian(&robotData.allRobotHp.redSoldier1, message.data + 4);
    convertFromLittleEndian(&robotData.allRobotHp.redSoldier2, message.data + 6);
    convertFromLittleEndian(&robotData.allRobotHp.redSoldier3, message.data + 8);
    convertFromLittleEndian(&robotData.allRobotHp.redSentinel, message.data + 10);
    convertFromLittleEndian(&robotData.allRobotHp.redBase, message.data + 12);
    convertFromLittleEndian(&robotData.allRobotHp.blueHero, message.data + 14);
    convertFromLittleEndian(&robotData.allRobotHp.blueEngineer, message.data + 16);
    convertFromLittleEndian(&robotData.allRobotHp.blueSoldier1, message.data + 18);
    convertFromLittleEndian(&robotData.allRobotHp.blueSoldier2, message.data + 20);
    convertFromLittleEndian(&robotData.allRobotHp.blueSoldier3, message.data + 22);
    convertFromLittleEndian(&robotData.allRobotHp.blueSentinel, message.data + 24);
    convertFromLittleEndian(&robotData.allRobotHp.blueBase, message.data + 26);
    return true;
}

bool RefSerial::decodeToRobotStatus(const SerialMessage& message)
{
    if (message.length != 27)
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
    robotData.gimbalHasPower = message.data[26];
    robotData.chassisHasPower = (message.data[26] >> 1);
    robotData.shooterHasPower = (message.data[26] >> 2);

    processReceivedDamage(clock::getTimeMilliseconds(), robotData.previousHp - robotData.currentHp);
    robotData.previousHp = robotData.currentHp;

    return true;
}

bool RefSerial::decodeToPowerAndHeat(const SerialMessage& message)
{
    if (message.length != 16)
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

bool RefSerial::decodeToRobotPosition(const SerialMessage& message)
{
    if (message.length != 16)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.chassis.x, message.data);
    convertFromLittleEndian(&robotData.chassis.y, message.data + 4);
    convertFromLittleEndian(&robotData.chassis.z, message.data + 8);
    convertFromLittleEndian(&robotData.turret.yaw, message.data + 12);
    return true;
}

bool RefSerial::decodeToReceiveDamage(const SerialMessage& message)
{
    if (message.length != 1)
    {
        return false;
    }
    robotData.damagedArmorId = static_cast<ArmorId>(message.data[0]);
    robotData.damageType = static_cast<DamageType>(message.data[0] >> 4);
    return true;
}

bool RefSerial::decodeToProjectileLaunch(const SerialMessage& message)
{
    if (message.length != 7)
    {
        return false;
    }
    robotData.turret.bulletType = static_cast<BulletType>(message.data[0]);
    robotData.turret.launchMechanismID = static_cast<MechanismID>(message.data[1]);
    robotData.turret.firing_freq = message.data[2];
    convertFromLittleEndian(&robotData.turret.bulletSpeed, message.data + 3);
    return true;
}

bool RefSerial::decodeToBulletsRemain(const SerialMessage& message)
{
    if (message.length != 6)
    {
        return false;
    }
    convertFromLittleEndian(&robotData.turret.bulletsRemaining17, message.data);
    convertFromLittleEndian(&robotData.turret.bulletsRemaining42, message.data + 2);
    convertFromLittleEndian(&robotData.remainingCoins, message.data + 4);
    return true;
}

void RefSerial::processReceivedDamage(uint32_t timestamp, int32_t damageTaken)
{
    if (damageTaken > 0)
    {
        // create a new DamageEvent with the damage_taken, and current time
        DamageEvent damageEvent = {static_cast<uint16_t>(damageTaken), timestamp};

        if (receivedDpsTracker.getSize() == REF_DAMAGE_EVENT_SIZE)
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
           clock::getTimeMilliseconds() - receivedDpsTracker.getFront().timestampMs > 1000)
    {
        robotData.receivedDps -= receivedDpsTracker.getFront().damageAmount;
        receivedDpsTracker.removeFront();
    }
}

void RefSerial::configGraphicGenerics(
    GraphicData* graphicData,
    const uint8_t* name,
    AddGraphicOperation operation,
    uint8_t layer,
    GraphicColor color)
{
    memcpy(graphicData->name, name, 3);
    graphicData->operation = operation;
    graphicData->layer = layer;
    graphicData->color = color;
}

void RefSerial::configLine(
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    uint16_t endX,
    uint16_t endY,
    GraphicData* sharedData)
{
    sharedData->type = STRAIGHT_LINE;
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
    GraphicData* sharedData)
{
    sharedData->type = RECTANGLE;
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
    GraphicData* sharedData)
{
    sharedData->type = CIRCLE;
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
    GraphicData* sharedData)
{
    sharedData->type = ELLIPSE;
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
    GraphicData* sharedData)
{
    sharedData->type = ARC;
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
    GraphicData* sharedData)
{
    sharedData->type = FLOATING_NUM;
    sharedData->startAngle = fontSize;
    sharedData->endAngle = decimalPrecision;
    sharedData->lineWidth = width;
    sharedData->startX = startX;
    sharedData->startY = startY;
    // Do this janky stuff to get an int in a bitfield
    int32_t valueInt = 1000 * value;
    sharedData->radius = valueInt & 0x3fff;
    sharedData->endX = (valueInt >> 10) & 0x7ff;
    sharedData->endY = (valueInt >> 21) & 0x7ff;
}

void RefSerial::configInteger(
    uint16_t fontSize,
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    int32_t value,
    GraphicData* sharedData)
{
    sharedData->type = INTEGER;
    sharedData->startAngle = fontSize;
    sharedData->lineWidth = width;
    sharedData->startX = startX;
    sharedData->startY = startY;
    // Do this janky stuff to get an int in a bitfield
    sharedData->radius = value & 0x3fff;
    sharedData->endX = (value >> 10) & 0x7ff;
    sharedData->endY = (value >> 21) & 0x7ff;
}

void RefSerial::updateInteger(int32_t value, GraphicData* sharedData)
{
    sharedData->radius = value & 0x3fff;
    sharedData->endX = (value >> 10) & 0x7ff;
    sharedData->endY = (value >> 21) & 0x7ff;
}

void RefSerial::configCharacterMsg(
    uint16_t fontSize,
    uint16_t charLen,
    uint16_t width,
    uint16_t startX,
    uint16_t startY,
    const char* dataToPrint,
    GraphicCharacterMessage* sharedData)
{
    sharedData->graphicData.type = CHARACTER;
    sharedData->graphicData.startAngle = fontSize;
    sharedData->graphicData.endAngle = charLen;
    sharedData->graphicData.lineWidth = width;
    sharedData->graphicData.startX = startX;
    sharedData->graphicData.startY = startY;
    strncpy(sharedData->msg, dataToPrint, GRAPHIC_MAX_CHARACTERS - 1);
}

void RefSerial::deleteGraphicLayer(DeleteGraphicOperation graphicOperation, uint8_t graphicLayer)
{
    DeleteGraphicLayerMessage msg;
    msg.deleteOperation = graphicOperation;
    msg.layer = graphicLayer;

    configFrameHeader(
        &msg.frameHead,
        sizeof(msg.graphicHead) + sizeof(msg.deleteOperation) + sizeof(msg.layer));

    msg.cmdId = 0x301;

    configGraphicHeader(&msg.graphicHead, 0x100, robotData.robotId);

    msg.crc16 = algorithms::calculateCRC16(
        reinterpret_cast<uint8_t*>(&msg),
        sizeof(DeleteGraphicLayerMessage) - 2);

    drivers->uart.write(
        Uart::Uart6,
        reinterpret_cast<uint8_t*>(&msg),
        sizeof(DeleteGraphicLayerMessage));
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
    uint16_t robotId,
    Drivers* drivers,
    int extraDataLength = 0)
{
    if (configMsgHeader)
    {
        RefSerial::configFrameHeader(
            &graphicMsg->msgHeader,
            sizeof(graphicMsg->graphicData) + sizeof(graphicMsg->graphicHeader) + extraDataLength);

        graphicMsg->cmdId = 0x301;

        RefSerial::configGraphicHeader(&graphicMsg->graphicHeader, cmdId, robotId);

        graphicMsg->crc16 = algorithms::calculateCRC16(
            reinterpret_cast<uint8_t*>(graphicMsg),
            sizeof(GraphicType) - 2);
    }

    if (sendMsg)
    {
        drivers->uart.write(
            Uart::Uart6,
            reinterpret_cast<uint8_t*>(graphicMsg),
            sizeof(GraphicType));
    }
}
void RefSerial::sendGraphic(Graphic1Message* graphicMsg, bool configMsgHeader, bool sendMsg)
{
    sendGraphicHelper(graphicMsg, 0x101, configMsgHeader, sendMsg, robotData.robotId, drivers);
}

void RefSerial::sendGraphic(Graphic2Message* graphicMsg, bool configMsgHeader, bool sendMsg)
{
    sendGraphicHelper(graphicMsg, 0x102, configMsgHeader, sendMsg, robotData.robotId, drivers);
}

void RefSerial::sendGraphic(Graphic5Message* graphicMsg, bool configMsgHeader, bool sendMsg)
{
    sendGraphicHelper(graphicMsg, 0x103, configMsgHeader, sendMsg, robotData.robotId, drivers);
}

void RefSerial::sendGraphic(Graphic7Message* graphicMsg, bool configMsgHeader, bool sendMsg)
{
    sendGraphicHelper(graphicMsg, 0x104, configMsgHeader, sendMsg, robotData.robotId, drivers);
}

void RefSerial::sendGraphic(GraphicCharacterMessage* graphicMsg, bool configMsgHeader, bool sendMsg)
{
    sendGraphicHelper(
        graphicMsg,
        0x110,
        configMsgHeader,
        sendMsg,
        robotData.robotId,
        drivers,
        GRAPHIC_MAX_CHARACTERS);
}

void RefSerial::configFrameHeader(FrameHeader* header, uint16_t msgLen)
{
    header->SOF = 0xa5;
    header->dataLength = msgLen;
    header->seq = 0;
    header->CRC8 = algorithms::calculateCRC8(
        reinterpret_cast<const uint8_t*>(header),
        sizeof(FrameHeader) - 1);
}

void RefSerial::configGraphicHeader(GraphicHeader* header, uint16_t cmdId, uint16_t robotId)
{
    header->dataCmdId = cmdId;
    header->senderId = robotId;
    header->receiverId = getRobotClientID(robotId);
}
}  // namespace serial
}  // namespace aruwlib
