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

#ifndef __REF_SERIAL_HPP__
#define __REF_SERIAL_HPP__

#include <cstdint>

#include "tap/architecture/timeout.hpp"
#include "tap/util_macros.hpp"

#include "modm/container/deque.hpp"

#include "dji_serial.hpp"

namespace tap
{
class Drivers;
namespace serial
{
/**
 * A class meant to communicate with the 2021 version of the RoboMaster
 * referee system.
 *
 * For information about the protocol that this serial parser/decoder uses,
 * view RoboMaster's ref serial website:
 * https://www.robomaster.com/en-US/products/components/referee (in the Document Download tab).
 *
 * @note use the instance stored in the `Drivers` to interact with this class
 *      (you shouldn't be declaring your own `RefSerial` object).
 */
class RefSerial : public DJISerial
{
private:
    static constexpr uint32_t TIME_OFFLINE_REF_DATA_MS = 1000;

    // RX message constants
    static constexpr uint16_t REF_DAMAGE_EVENT_SIZE = 20;

    static constexpr uint16_t CUSTOM_DATA_MAX_LENGTH = 113;
    static constexpr uint16_t CUSTOM_DATA_TYPE_LENGTH = 2;
    static constexpr uint16_t CUSTOM_DATA_SENDER_ID_LENGTH = 2;
    static constexpr uint16_t CUSTOM_DATA_RECIPIENT_ID_LENGTH = 2;

    // TX message constants
    static constexpr uint32_t TIME_BETWEEN_REF_UI_DISPLAY_SEND_MS = 100;

    // RX message type defines, ignored message types commented out
    static constexpr uint16_t REF_MESSAGE_TYPE_GAME_STATUS = 0x1;
    static constexpr uint16_t REF_MESSAGE_TYPE_GAME_RESULT = 0x2;
    static constexpr uint16_t REF_MESSAGE_TYPE_ALL_ROBOT_HP = 0x3;
    // static constexpr uint16_t REF_MESSAGE_TYPE_DART_LAUNCHING_STATUS = 0x4;
    // static constexpr uint16_t REF_MESSAGE_TYPE_SITE_EVENT_DATA =  0X101;
    // static constexpr uint16_t REF_MESSAGE_TYPE_PROJECTILE_SUPPPLIER_SITE_ACTION = 0x102;
    // static constexpr uint16_t REF_MESSAGE_TYPE_PROJECTILE_SUPPLY_REQUESTED = 0X103;
    // static constexpr uint16_t REF_MESSAGE_TYPE_WARNING_DATA = 0X104;
    // static constexpr uint16_t REF_MESSAGE_TYPE_DART_LAUNCH_OPENING_COUNT = 0X105;
    static constexpr uint16_t REF_MESSAGE_TYPE_ROBOT_STATUS = 0x201;
    static constexpr uint16_t REF_MESSAGE_TYPE_POWER_AND_HEAT = 0x202;
    static constexpr uint16_t REF_MESSAGE_TYPE_ROBOT_POSITION = 0x203;
    // static constexpr uint16_t REF_MESSAGE_TYPE_ROBOT_BUFF_STATUS = 0x204;
    // static constexpr uint16_t REF_MESSAGE_TYPE_AERIAL_ENERGY_STATUS = 0x205;
    static constexpr uint16_t REF_MESSAGE_TYPE_RECEIVE_DAMAGE = 0x206;
    static constexpr uint16_t REF_MESSAGE_TYPE_PROJECTILE_LAUNCH = 0x207;
    static constexpr uint16_t REF_MESSAGE_TYPE_BULLETS_REMAIN = 0x208;
    // static constexpr uint16_t REF_MESSAGE_TYPE_RFID_STATUS = 0x209;
    // static constexpr uint16_t REF_MESSAGE_TYPE_DART_INSTRUCTIONS = 0x20A;
    static constexpr uint16_t REF_MESSAGE_TYPE_CUSTOM_DATA = 0x301;
    // static constexpr uint16_t REF_MESSAGE_TYPE_CUSTOM_CONTROLLER = 0x302;
    // static constexpr uint16_t REF_MESSAGE_TYPE_SMALL_MAP = 0x303;

    // TX message type defines
    static constexpr uint16_t REF_CUSTOM_DATA_DRAW_GRAPHIC = 0x301;

    static constexpr int GRAPHIC_MSG_HEADER_LENGTH = 6;
    static constexpr int GRAPHIC_MAX_CHARACTERS = 30;
    static constexpr int GRAPHIC_DATA_LENGTH = 15;

#ifdef ENV_UNIT_TESTS
public:
#endif
    struct __attribute__((packed)) FrameHeader
    {
        uint8_t SOF;
        uint16_t dataLength;
        uint8_t seq;
        uint8_t CRC8;
    };

    struct __attribute__((packed)) GraphicHeader
    {
        uint16_t dataCmdId;
        uint16_t senderId;
        uint16_t receiverId;
    };

    struct __attribute__((packed)) GraphicData
    {
        uint8_t name[3];
        uint32_t operation : 3;
        uint32_t type : 3;
        uint32_t layer : 4;
        uint32_t color : 4;
        uint32_t startAngle : 9;
        uint32_t endAngle : 9;
        uint32_t lineWidth : 10;
        uint32_t startX : 11;
        uint32_t startY : 11;
        uint32_t radius : 10;
        uint32_t endX : 11;
        uint32_t endY : 11;
    };

    struct __attribute__((packed)) DeleteGraphicLayerMessage
    {
        FrameHeader frameHead;
        uint16_t cmdId;
        GraphicHeader graphicHead;
        uint8_t deleteOperation;
        uint8_t layer;
        uint16_t crc16;
    };

public:
    enum GameStages
    {
        PREMATCH = 0,        /// Pre-competition. stage
        SETUP = 1,           /// Setup stage.
        INITIALIZATION = 2,  /// Initialization stage.
        COUNTDOWN = 3,       /// 5-second countdown.
        IN_GAME = 4,         /// In middle of the game.
        END_GAME = 5,        /// Calculating competition results.
    };

    enum GameWinner
    {
        DRAW = 0,  /// Match was a draw.
        RED = 1,   /// Red team won the match.
        BLUE = 2,  /// Blue team won the match.
    };

    enum RobotId
    {
        INVALID = 0,

        RED_HERO = 1,
        RED_ENGINEER = 2,
        RED_SOLDIER_1 = 3,
        RED_SOLDIER_2 = 4,
        RED_SOLDIER_3 = 5,
        RED_DRONE = 6,
        RED_SENTINEL = 7,
        RED_DART = 8,
        RED_RADAR_STATION = 9,

        BLUE_HERO = 101,
        BLUE_ENGINEER = 102,
        BLUE_SOLDIER_1 = 103,
        BLUE_SOLDIER_2 = 104,
        BLUE_SOLDIER_3 = 105,
        BLUE_DRONE = 106,
        BLUE_SENTINEL = 107,
        BLUE_DART = 108,
        BLUE_RADAR_STATION = 109
    };

    enum ArmorId
    {
        FRONT = 0,  /// armor #0 (front).
        LEFT = 1,   /// armor #1 (left).
        REAR = 2,   /// armor #2 (rear).
        RIGHT = 3,  /// armor #3 (right).
        TOP = 4,    /// armor #4 (top).
    };

    enum DamageType
    {
        ARMOR_DAMAGE = 0,           /// Armor damage.
        MODULE_OFFLINE = 1,         /// Module offline.
        BARREL_OVER_SPEED = 2,      /// Firing speed too high.
        BARREL_OVERHEAT = 3,        /// Barrel overheat.
        CHASSIS_POWER_OVERRUN = 4,  /// Chassis power overrun.
        COLLISION = 5,              /// Chassis collision.
    };

    enum DeleteGraphicOperation
    {
        DELETE_GRAPHIC_NO_OP = 0,
        DELETE_GRAPHIC_LAYER = 1,
        DELETE_ALL = 2,
    };

    enum AddGraphicOperation
    {
        ADD_GRAPHIC_NO_OP = 0,
        ADD_GRAPHIC = 1,
        ADD_GRAPHIC_MODIFY = 2,
        ADD_GRAPHIC_DELETE = 3  /// Not sure why you can specify delete when adding a graphic
    };

    enum GraphicType
    {
        STRAIGHT_LINE = 0,
        RECTANGLE = 1,
        CIRCLE = 2,
        ELLIPSE = 3,
        ARC = 4,
        FLOATING_NUM = 5,
        INTEGER = 6,
        CHARACTER = 7,
    };

    enum GraphicColor
    {
        RED_AND_BLUE = 0,
        YELLOW = 1,
        GREEN = 2,
        ORANGE = 3,
        PURPLISH_RED = 4,
        PINK = 5,
        CYAN = 6,
        BLACK = 7,
        WHITE = 8,
    };

    struct __attribute__((packed)) Graphic1Message
    {
        FrameHeader msgHeader;
        uint16_t cmdId;
        GraphicHeader graphicHeader;
        GraphicData graphicData;
        uint16_t crc16;
    };

    struct __attribute__((packed)) Graphic2Message
    {
        FrameHeader msgHeader;
        uint16_t cmdId;
        GraphicHeader graphicHeader;
        GraphicData graphicData[2];
        uint16_t crc16;
    };

    struct __attribute__((packed)) Graphic5Message
    {
        FrameHeader msgHeader;
        uint16_t cmdId;
        GraphicHeader graphicHeader;
        GraphicData graphicData[5];
        uint16_t crc16;
    };

    struct __attribute__((packed)) Graphic7Message
    {
        FrameHeader msgHeader;
        uint16_t cmdId;
        GraphicHeader graphicHeader;
        GraphicData graphicData[7];
        uint16_t crc16;
    };

    struct __attribute__((packed)) GraphicCharacterMessage
    {
        FrameHeader msgHeader;
        uint16_t cmdId;
        GraphicHeader graphicHeader;
        GraphicData graphicData;
        char msg[30];
        uint16_t crc16;
    };

    struct DamageEvent
    {
        uint16_t damageAmount;  /// Amount of damage received
        uint32_t timestampMs;   /// Time when damage was received (in milliseconds).
    };

    enum BulletType
    {
        AMMO_17 = 1,  /// 17 mm projectile ammo.
        AMMO_42 = 2,  /// 42 mm projectile ammo.
    };

    enum MechanismID
    {
        TURRET_17MM_1 = 1,
        TURRET_17MM_2 = 2,
        TURRET_42MM = 3,
    };

    struct GameData
    {
        GameStages gameStage : 4;     /// Current stage in the game.
        uint16_t stageTimeRemaining;  /// Remaining time in the current stage (in seconds).
        GameWinner gameWinner;        /// Results of the match.
    };

    struct ChassisData
    {
        uint16_t volt;         /// Output voltage to the chassis (in mV).
        uint16_t current;      /// Output current to the chassis (in mA).
        float power;           /// Output power to the chassis (in W).
        uint16_t powerBuffer;  /// Chassis power buffer (in J).
        float x, y, z;         /// x, y, z coordinate of the chassis.
        uint16_t powerConsumptionLimit;
    };

    struct TurretData
    {
        BulletType bulletType;           /// 17mm or 42mm last projectile shot.
        MechanismID launchMechanismID;   /// Either 17mm mechanism 1, 3, or 42 mm mechanism.
        uint8_t firing_freq;             /// Firing frequency (in Hz).
        uint16_t heat17ID1;              /// Current 17mm turret heat, ID2.
        uint16_t heat17ID2;              /// ID2 turret heat.
        uint16_t heatCoolingRate17ID1;   /// 17mm turret cooling value per second, ID1.
        uint16_t heatCoolingRate17ID2;   /// ID2.
        uint16_t heatLimit17ID1;         /// 17mm turret heat limit, ID1.
        uint16_t heatLimit17ID2;         /// ID2.
        uint16_t barrelSpeedLimit17ID1;  /// 17mm turret barrel speed limit, ID1.
        uint16_t barrelSpeedLimit17ID2;  /// ID2.
        uint16_t heat42;                 /// Current 42mm turret heat.
        uint16_t heatCoolingRate42;      /// 42mm turret cooling value per second.
        uint16_t heatLimit42;            /// 42mm turret heat limit.
        uint16_t barrelSpeedLimit42;     /// 42mm turret barrel speed.
        uint16_t bulletsRemaining17;     /// Number of bullets remaining in sentinel
                                         /// and drone only (500 max) if in RMUC, or
                                         /// any robot in RMUL.
        uint16_t bulletsRemaining42;     /// Number of bullets remaining in hero if in RMUL
                                         /// or 0 if in RMUC.
        float bulletSpeed;               /// Last bullet speed (in m/s).
        float yaw;                       /// Barrel yaw position (degree).
    };

    struct RobotHpData
    {
        // current HP of all robots
        uint16_t redHero;
        uint16_t redEngineer;
        uint16_t redSoldier1;
        uint16_t redSoldier2;
        uint16_t redSoldier3;
        uint16_t redSentinel;
        uint16_t redBase;
        uint16_t blueHero;
        uint16_t blueEngineer;
        uint16_t blueSoldier1;
        uint16_t blueSoldier2;
        uint16_t blueSoldier3;
        uint16_t blueSentinel;
        uint16_t blueBase;
    };

    struct RobotData
    {
        RobotId robotId;              /// Robot type and team.
        uint8_t robotLevel;           /// Current level of this robot (1-3).
        uint16_t previousHp;          /// Health of this robot before damage was
                                      /// received, used to calculate receivedDps
                                      /// if no damage was received recently,
                                      /// previousHp = currentHp.
        uint16_t currentHp;           /// Current health of this robot.
        uint16_t maxHp;               /// Max health of this robot.
        uint8_t gimbalHasPower : 1;   /// 1 if there is 24V output to gimbal, 0 for 0V.
        uint8_t chassisHasPower : 1;  /// 1 if there is 24V output to chassis, 0 for 0V.
        uint8_t shooterHasPower : 1;  /// 1 if there is 24V output to shooter, 0 for 0V.
        ArmorId damagedArmorId : 4;   /// Armor ID that was damaged.
        DamageType damageType : 4;    /// Cause of damage.
        float receivedDps;            /// Damage per second received.
        ChassisData chassis;          /// Chassis power draw and position data.
        TurretData turret;            /// Turret firing and heat data.
        RobotHpData allRobotHp;       /// Current HP of all the robots.
        uint16_t remainingCoins;      /// Number of remaining coins left to spend.
    };

    /**
     * Constructs a RefSerial class connected to `bound_ports::REF_SERIAL_UART_PORT` with
     * CRC enforcement enabled.
     *
     * @see `DjiSerial`
     */
    RefSerial(Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(RefSerial)
    mockable ~RefSerial() = default;

    /**
     * Handles the types of messages defined above in the RX message handlers section.
     */
    void messageReceiveCallback(const SerialMessage& completeMessage) override;

    mockable bool getRefSerialReceivingData() const;

    /**
     * Returns a reference to the most up to date robot data struct.
     */
    mockable const RobotData& getRobotData() const;

    /**
     * Returns a reference to the most up to date game data struct.
     */
    mockable const GameData& getGameData() const;

    /**
     * Configures the `graphicData` with all data generic to the type of graphic being configured.
     *
     * For sending graphics, the general schema is to create a `Graphic<n>Message` struct, configure
     * the individual `GraphicData` structs in the graphic message using the `configGraphicGenerics`
     * and then `config<Line|Rectangle|Circle|etc.>` functions. Finally, send the graphic message
     * using `sendGraphic`.
     *
     * For example, to configure and send a line graphic (`refSerial` is a pointer to the global
     * `RefSerial` object):
     *
     * ```
     * Graphic1Message msg;
     * refSerial->configGraphicGenerics(&msg.graphicData, "\x00\x00\x01", RefSerial::ADD_GRAPHIC,1,
     * YELLOW); refSerial->configLine(4, 100, 100, 200, 200, &msg.graphicData);
     * refSerial->sendGraphic(&msg);
     * ```
     *
     * @param[out] graphciData The structure where generic data will be stored.
     * @param[in] name The name of the graphic.
     * @param[in] operation The graphic operation to be done (add/remove, etc).
     * @param[in] layer The graphic layer the graphic will be located at.
     * @param[in] color The color of the graphic.
     */
    mockable void configGraphicGenerics(
        GraphicData* graphicData,
        const uint8_t* name,
        AddGraphicOperation operation,
        uint8_t layer,
        GraphicColor color);
    /**
     * Configures `sharedData` with a line graphic whose parameters are specified.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    mockable void configLine(
        uint16_t width,
        uint16_t startX,
        uint16_t startY,
        uint16_t endX,
        uint16_t endY,
        GraphicData* sharedData);
    /**
     * Configures `sharedData` with a rectangle graphic whose parameters are specified.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    mockable void configRectangle(
        uint16_t width,
        uint16_t startX,
        uint16_t startY,
        uint16_t endX,
        uint16_t endY,
        GraphicData* sharedData);
    /**
     * Configures `sharedData` with a circle graphic whose parameters are specified.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    mockable void configCircle(
        uint16_t width,
        uint16_t centerX,
        uint16_t centerY,
        uint16_t radius,
        GraphicData* sharedData);
    /**
     * Configures `sharedData` with an ellipse graphic whose parameters are specified.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    mockable void configEllipse(
        uint16_t width,
        uint16_t centerX,
        uint16_t centerY,
        uint16_t xLen,
        uint16_t yLen,
        GraphicData* sharedData);
    /**
     * Configures `sharedData` with an arc graphic whose parameters are specified.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    mockable void configArc(
        uint16_t startAngle,
        uint16_t endAngle,
        uint16_t width,
        uint16_t centerX,
        uint16_t centerY,
        uint16_t xLen,
        uint16_t yLen,
        GraphicData* sharedData);
    // Recommended font size and line width ratio is 10:1.
    /**
     * Configures `sharedData` with a floating point number.
     *
     * @note This function doesn't work because of known issues in the referee system
     *      server.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    mockable void configFloatingNumber(
        uint16_t fontSize,
        uint16_t decimalPrecision,
        uint16_t width,
        uint16_t startX,
        uint16_t startY,
        float value,
        GraphicData* sharedData);
    /**
     * Configures `sharedData` with an integer.
     *
     * @note This function doesn't display negative numbers properly because of known
     *      issues in the referee system server.
     *
     * @param[out] sharedData The graphic data struct to configure.
     */
    mockable void configInteger(
        uint16_t fontSize,
        uint16_t width,
        uint16_t startX,
        uint16_t startY,
        int32_t value,
        GraphicData* sharedData);
    /**
     * Only updates the integer value stored in the `sharedData` (it is assumed that
     * the `GraphicData` is already configured with `configInteger`).
     *
     * @param[out] sharedData The message with whose value to update.
     */
    mockable void updateInteger(int32_t value, GraphicData* sharedData);
    /**
     * Configures a character message in the passed in `GraphicCharacterMessage`.
     *
     * @param[out] sharedData The message to configure.
     */
    mockable void configCharacterMsg(
        uint16_t fontSize,
        uint16_t charLen,
        uint16_t width,
        uint16_t startX,
        uint16_t startY,
        const char* dataToPrint,
        GraphicCharacterMessage* sharedData);

    /**
     * Deletes an entire graphic layer or the entire screen
     *
     * @param[in] graphicOperation Whether to delete a single layer or the entire screen.
     * @param[in] graphicLayer The layer to remove.
     */
    mockable void deleteGraphicLayer(DeleteGraphicOperation graphicOperation, uint8_t graphicLayer);

    /**
     * This function and the ones below all configure the message header and sends the specified
     * message struct to the referee system unless `configMsgHeader` or `sendMsg` are false.
     *
     * @param[in] graphicMsg The graphic message to send. Note that this struct is updated
     *      with header information in this function.
     * @param[in] configMsgHeader Whether or not to update the `graphicMsg`'s header information.
     * @param[in] sendMsg Whether or not to send the message.
     */
    void sendGraphic(Graphic1Message* graphicMsg, bool configMsgHeader = true, bool sendMsg = true);
    void sendGraphic(Graphic2Message* graphicMsg, bool configMsgHeader = true, bool sendMsg = true);
    void sendGraphic(Graphic5Message* graphicMsg, bool configMsgHeader = true, bool sendMsg = true);
    void sendGraphic(Graphic7Message* graphicMsg, bool configMsgHeader = true, bool sendMsg = true);
    void sendGraphic(
        GraphicCharacterMessage* graphicMsg,
        bool configMsgHeader = true,
        bool sendMsg = true);

    static void configFrameHeader(FrameHeader* header, uint16_t msgLen);
    static void configGraphicHeader(GraphicHeader* header, uint16_t cmdId, uint16_t robotId);

private:
    RobotData robotData;
    GameData gameData;
    modm::BoundedDeque<DamageEvent, REF_DAMAGE_EVENT_SIZE> receivedDpsTracker;
    arch::MilliTimeout refSerialOfflineTimeout;

    /**
     * Given RobotId, returns the client_id that the referee system uses to display
     * the received messages to the given client_id robot.
     *
     * @param[in] robotId the id of the robot received from the referee system
     *      to get the client_id of.
     * @return the client_id of the robot requested.
     */
    static uint16_t getRobotClientID(uint16_t robotId);

    /**
     * Decodes ref serial message containing the game stage and time remaining
     * in the game.
     */
    bool decodeToGameStatus(const SerialMessage& message);
    /**
     * Decodes ref serial message containing the postmatch result of a game.
     */
    bool decodeToGameResult(const SerialMessage& message);
    /**
     * Decodes ref serial message containing the robot HP of all robots in the match.
     */
    bool decodeToAllRobotHP(const SerialMessage& message);
    /**
     * Decodes ref serial message containing the firing/driving heat limits and cooling
     * rates for the robot.
     */
    bool decodeToRobotStatus(const SerialMessage& message);
    /**
     * Decodes ref serial message containing the actual power and heat data for the turret
     * and chassis.
     */
    bool decodeToPowerAndHeat(const SerialMessage& message);
    /**
     * Decodes ref serial message containing the position of the robot on the field and
     * the robot heading.
     */
    bool decodeToRobotPosition(const SerialMessage& message);
    /**
     * Decodes ref serial message containing containing the damaged armor and damage type
     * last taken by the robot.
     */
    bool decodeToReceiveDamage(const SerialMessage& message);
    /**
     * Decodes ref serial message containing the previously fired bullet type and firing
     * frequency.
     */
    bool decodeToProjectileLaunch(const SerialMessage& message);
    /**
     * Decodes ref serial message containing the number of bullets remaining in the robot
     * (only certain match types will send this information).
     */
    bool decodeToBulletsRemain(const SerialMessage& message);

    void updateReceivedDamage();
    void processReceivedDamage(uint32_t timestamp, int32_t damageTaken);
};

}  // namespace serial

}  // namespace tap

#endif
