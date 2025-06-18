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

#ifndef TAPROOT_REF_SERIAL_DATA_HPP_
#define TAPROOT_REF_SERIAL_DATA_HPP_

#include <cinttypes>

#include <modm/architecture/interface/register.hpp>

#include "modm/architecture/utils.hpp"

#include "dji_serial.hpp"

namespace tap::communication::serial
{
/**
 * Contains enum and struct definitions used in the `RefSerial` class.
 */
class RefSerialData
{
public:
    /**
     * When receiving data about other robots or sending data to other robots, they are
     * identified by the numbers below
     */
    enum class RobotId : uint16_t
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

    static inline bool isBlueTeam(RobotId id) { return id >= RobotId::BLUE_HERO; }

    class RobotToRobotMessageHandler
    {
    public:
        RobotToRobotMessageHandler() {}
        virtual void operator()(const DJISerial::ReceivedSerialMessage &message) = 0;
    };

    /**
     * Contains enum and struct definitions specific to receiving data from the referee serial
     * class.
     */
    class Rx
    {
    public:
        enum class GameStage : uint8_t
        {
            PREMATCH = 0,        ///< Pre-competition. stage
            SETUP = 1,           ///< Setup stage.
            INITIALIZATION = 2,  ///< Initialization stage.
            COUNTDOWN = 3,       ///< 5-second countdown.
            IN_GAME = 4,         ///< In middle of the game.
            END_GAME = 5,        ///< Calculating competition results.
        };

        enum class GameWinner : uint8_t
        {
            DRAW = 0,  ///< Match was a draw.
            RED = 1,   ///< Red team won the match.
            BLUE = 2,  ///< Blue team won the match.
        };

        enum class ArmorId : uint8_t
        {
            FRONT = 0,  ///< armor #0 (front).
            LEFT = 1,   ///< armor #1 (left).
            REAR = 2,   ///< armor #2 (rear).
            RIGHT = 3,  ///< armor #3 (right).
            TOP = 4,    ///< armor #4 (top).
        };

        enum class DamageType : uint8_t
        {
            ARMOR_DAMAGE = 0,           ///< Armor damage.
            MODULE_OFFLINE = 1,         ///< Module offline.
            BARREL_OVER_SPEED = 2,      ///< Firing speed too high.
            BARREL_OVERHEAT = 3,        ///< Barrel overheat.
            CHASSIS_POWER_OVERRUN = 4,  ///< Chassis power overrun.
            COLLISION = 5,              ///< Armor plate collision.
        };

        enum class RobotPower : uint8_t
        {
            GIMBAL_HAS_POWER = modm::Bit0,   ///< 1 if there is 24V output to gimbal, 0 for 0V.
            CHASSIS_HAS_POWER = modm::Bit1,  ///< 1 if there is 24V output to chassis, 0 for 0V.
            SHOOTER_HAS_POWER = modm::Bit2,  ///< 1 if there is 24V output to shooter, 0 for 0V.
        };
        MODM_FLAGS8(RobotPower);

        enum class RobotBuffStatus : uint8_t
        {
            ROBOT_HP_RESTORATION_STATUS = modm::Bit0,
            BARREL_HEAT_COOLING_ACCELERATION = modm::Bit1,
            ROBOT_DEFENSE_BUFF = modm::Bit2,
            ROBOT_ATTACK_BUFF = modm::Bit3,
        };
        MODM_FLAGS8(RobotBuffStatus);

        enum class RFIDActivationStatus : uint8_t
        {
            BASE_GAIN_ZONE = modm::Bit0,
            ELEVATED_GROUND_ZONE = modm::Bit1,
            POWER_RUNE_ACTIVATION_POINT = modm::Bit2,
            LAUNCH_RAMP = modm::Bit3,
            OUTPOST_ZONE = modm::Bit4,
            RESOURCE_ISLAND_ZONE = modm::Bit5,
            RESTORATION_ZONE = modm::Bit6,
            ENGINEER_RESTORATION = modm::Bit7,
        };
        MODM_FLAGS8(RFIDActivationStatus);

        struct DamageEvent
        {
            uint16_t damageAmount;  ///< Amount of damage received
            uint32_t timestampMs;   ///< Time when damage was received (in milliseconds).
        };

        enum BulletType
        {
            AMMO_17 = 1,  ///< 17 mm projectile ammo.
            AMMO_42 = 2,  ///< 42 mm projectile ammo.
        };

        enum MechanismID
        {
            TURRET_17MM_1 = 0,
            TURRET_17MM_2 = 1,
            TURRET_42MM = 2,
        };

        struct GameData
        {
            GameStage gameStage;          ///< Current stage in the game.
            uint16_t stageTimeRemaining;  ///< Remaining time in the current stage (in seconds).
            GameWinner gameWinner;        ///< Results of the match.
        };

        /**
         * Current HP of all robots
         */
        struct RobotHpData
        {
            struct RobotHp
            {
                uint16_t hero1;
                uint16_t engineer2;
                uint16_t standard3;
                uint16_t standard4;
                uint16_t standard5;
                uint16_t sentry7;
                uint16_t outpost;
                uint16_t base;
            };

            RobotHp red;
            RobotHp blue;
        };

        struct ChassisData
        {
            uint16_t volt;         ///< Output voltage to the chassis (in mV).
            uint16_t current;      ///< Output current to the chassis (in mA).
            float power;           ///< Output power to the chassis (in W).
            uint16_t powerBuffer;  ///< Chassis power buffer (in J).
            float x, y, z;         ///< x, y, z coordinate of the chassis.
            uint16_t powerConsumptionLimit;
        };

        struct TurretData
        {
            BulletType bulletType;           ///< 17mm or 42mm last projectile shot.
            MechanismID launchMechanismID;   ///< Either 17mm mechanism 1, 3, or 42 mm mechanism.
            uint8_t firingFreq;              ///< Firing frequency (in Hz).
            uint16_t heat17ID1;              ///< Current 17mm turret heat, ID2.
            uint16_t heat17ID2;              ///< ID2 turret heat.
            uint16_t heatCoolingRate17ID1;   ///< 17mm turret cooling value per second, ID1.
            uint16_t heatCoolingRate17ID2;   ///< ID2.
            uint16_t heatLimit17ID1;         ///< 17mm turret heat limit, ID1.
            uint16_t heatLimit17ID2;         ///< ID2.
            uint16_t barrelSpeedLimit17ID1;  ///< 17mm turret barrel speed limit, ID1.
            uint16_t barrelSpeedLimit17ID2;  ///< ID2.
            uint16_t heat42;                 ///< Current 42mm turret heat.
            uint16_t heatCoolingRate42;      ///< 42mm turret cooling value per second.
            uint16_t heatLimit42;            ///< 42mm turret heat limit.
            uint16_t barrelSpeedLimit42;     ///< 42mm turret barrel speed.
            uint16_t bulletsRemaining17;     ///< Number of bullets remaining in sentinel and drone
                                             ///< only (500 max) if in RMUC, or any robot in RMUL.
            uint16_t bulletsRemaining42;  ///< Number of bullets remaining in hero if in RMUL or 0
                                          ///< if in RMUC.
            float bulletSpeed;            ///< Last bullet speed (in m/s).
            float yaw;                    ///< Barrel yaw position (degree).
            uint32_t lastReceivedLaunchingInfoTimestamp;  ///< Last time in milliseconds that the
                                                          ///< real-time launching information
                                                          ///< message was received
        };

        struct RobotData
        {
            RobotId robotId;          ///< Robot type and team.
            uint8_t robotLevel;       ///< Current level of this robot (1-3).
            uint16_t previousHp;      ///< Health of this robot before damage was
                                      ///< received, used to calculate receivedDps
                                      ///< if no damage was received recently,
                                      ///< previousHp = currentHp.
            uint16_t currentHp;       ///< Current health of this robot.
            uint16_t maxHp;           ///< Max health of this robot.
            RobotPower_t robotPower;  ///< Flags indicating which parts of the robot have power
            ArmorId damagedArmorId;   ///< Armor ID that was damaged.
            DamageType damageType;    ///< Cause of damage.
            float receivedDps;        ///< Damage per second received.
            ChassisData chassis;      ///< Chassis power draw and position data.
            TurretData turret;        ///< Turret firing and heat data.
            RobotHpData allRobotHp;   ///< Current HP of all the robots.
            uint16_t remainingCoins;  ///< Number of remaining coins left to spend.
            RobotBuffStatus_t robotBuffStatus;  ///< Status of all buffs on the robot
            uint16_t aerialEnergyStatus;  ///< Countdown timer that indicates how much time the
                                          ///< aerial has left to fire
            RFIDActivationStatus_t rfidStatus;    ///< The current status of which RFID zones
                                                  ///< are being activated by the current robot.
            uint32_t robotDataReceivedTimestamp;  ///< Most recent time at which data with message
                                                  ///< id `REF_MESSAGE_TYPE_ROBOT_STATUS` has been
                                                  ///< received.
        };
    };

    /**
     * Contains enum and struct definitions specific to sending data to the referee serial class.
     * Includes structure for sending different types of graphic messages.
     */
    class Tx
    {
    public:
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
            ADD_GRAPHIC_DELETE = 3,
        };

        enum class GraphicType : uint8_t
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

        enum class GraphicColor : uint8_t
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

        /**
         * Each graphic message has a graphic header inside of the message data.
         */
        struct InteractiveHeader
        {
            uint16_t dataCmdId;
            uint16_t senderId;
            uint16_t receiverId;
        } modm_packed;

        struct GraphicData
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
            union
            {
                struct
                {
                    uint32_t radius : 10;
                    uint32_t endX : 11;
                    uint32_t endY : 11;
                } modm_packed;
                int32_t value : 32;
            } modm_packed;
        } modm_packed;

        struct DeleteGraphicLayerMessage
        {
            DJISerial::FrameHeader frameHeader;
            uint16_t cmdId;
            InteractiveHeader interactiveHeader;
            uint8_t deleteOperation;
            uint8_t layer;
            uint16_t crc16;
        } modm_packed;

        struct Graphic1Message
        {
            DJISerial::FrameHeader frameHeader;
            uint16_t cmdId;
            InteractiveHeader interactiveHeader;
            GraphicData graphicData;
            uint16_t crc16;
        } modm_packed;

        struct RobotToRobotMessage
        {
            DJISerial::FrameHeader frameHeader;
            uint16_t cmdId;
            InteractiveHeader interactiveHeader;
            uint8_t dataAndCRC16[115];
        } modm_packed;

        struct Graphic2Message
        {
            DJISerial::FrameHeader frameHeader;
            uint16_t cmdId;
            InteractiveHeader interactiveHeader;
            GraphicData graphicData[2];
            uint16_t crc16;
        } modm_packed;

        struct Graphic5Message
        {
            DJISerial::FrameHeader frameHeader;
            uint16_t cmdId;
            InteractiveHeader interactiveHeader;
            GraphicData graphicData[5];
            uint16_t crc16;
        } modm_packed;

        struct Graphic7Message
        {
            DJISerial::FrameHeader frameHeader;
            uint16_t cmdId;
            InteractiveHeader interactiveHeader;
            GraphicData graphicData[7];
            uint16_t crc16;
        } modm_packed;

        struct GraphicCharacterMessage
        {
            DJISerial::FrameHeader frameHeader;
            uint16_t cmdId;
            InteractiveHeader interactiveHeader;
            GraphicData graphicData;
            char msg[30];
            uint16_t crc16;
        } modm_packed;

        /**
         * You cannot send messages faster than this speed to the referee system.
         *
         * Source: https://bbs.robomaster.com/forum.php?mod=viewthread&tid=9120
         */
        static constexpr uint32_t MAX_TRANSMIT_SPEED_BYTES_PER_S = 1280;

        /**
         * Get the max wait time after which you can send more data to the client. Sending faster
         * than this time may cause dropped packets.
         *
         * Pass a pointer to some graphic message. For example, if you have a `Graphic1Message`
         * called `msg`, you can call `getWaitTimeAfterGraphicSendMs(&msg)`.
         *
         * @tparam T The type of the graphic message that jas just been sent.
         */
        template <typename T>
        static constexpr uint32_t getWaitTimeAfterGraphicSendMs(T *)
        {
            // Must be a valid graphic message type
            static_assert(
                std::is_same<T, DeleteGraphicLayerMessage>::value ||
                    std::is_same<T, Graphic1Message>::value ||
                    std::is_same<T, RobotToRobotMessage>::value ||
                    std::is_same<T, Graphic2Message>::value ||
                    std::is_same<T, Graphic5Message>::value ||
                    std::is_same<T, Graphic7Message>::value ||
                    std::is_same<T, GraphicCharacterMessage>::value,
                "Invalid type, getWaitTimeAfterGraphicSendMs only takes in ref serial message "
                "types.");

            return sizeof(T) * 1'000 / MAX_TRANSMIT_SPEED_BYTES_PER_S;
        }
    };
};

inline RefSerialData::RobotId operator+(RefSerialData::RobotId id1, RefSerialData::RobotId id2)
{
    return static_cast<RefSerialData::RobotId>(
        static_cast<uint16_t>(id1) + static_cast<uint16_t>(id2));
}

inline RefSerialData::RobotId operator-(RefSerialData::RobotId id1, RefSerialData::RobotId id2)
{
    return static_cast<RefSerialData::RobotId>(
        static_cast<uint16_t>(id1) - static_cast<uint16_t>(id2));
}
}  // namespace tap::communication::serial

#endif  // TAPROOT_REF_SERIAL_DATA_HPP_
