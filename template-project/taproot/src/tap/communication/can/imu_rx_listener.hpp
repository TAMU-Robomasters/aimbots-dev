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

#ifndef IMU_RX_LISTENER_HPP_
#define IMU_RX_LISTENER_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/mpu6500/mpu6500.hpp"

#include "can_rx_listener.hpp"

namespace modm::can
{
class Message;
}

namespace tap
{
class Drivers;

namespace can
{
class ImuRxListener
{
public:
    ImuRxListener(Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(ImuRxListener);

    void init();

    inline float getYaw() const { return yaw; }
    inline float getGz() const
    {
        return static_cast<float>(rawGz) / sensors::Mpu6500::LSB_D_PER_S_TO_D_PER_S;
    }
    inline float isConnected() const
    {
        return !imuConnectedTimeout.isExpired() && !imuConnectedTimeout.isStopped();
    }

private:
    using ImuRxListenerFunc = void (ImuRxListener::*)(const modm::can::Message& message);

    static constexpr uint32_t ANGLE_GYRO_MESSAGE_CAN_ID = 0x203;
    static constexpr tap::can::CanBus IMU_MSG_CAN_BUS = tap::can::CanBus::CAN_BUS1;
    static constexpr uint32_t DISCONNECT_TIMEOUT_PERIOD = 100;

    class ImuRxHandler : public CanRxListener
    {
    public:
        ImuRxHandler(
            Drivers* drivers,
            uint32_t id,
            CanBus cB,
            ImuRxListener* msgHandler,
            ImuRxListenerFunc funcToCall);
        void processMessage(const modm::can::Message& message) override;

    private:
        ImuRxListener* msgHandler;
        ImuRxListenerFunc funcToCall;
    };

    Drivers* drivers;

    float yaw;
    int16_t rawGz;

    ImuRxHandler angleGyroMessageHandler;

    arch::MilliTimeout imuConnectedTimeout;

    void handleAngleGyroMessage(const modm::can::Message& message);
};
}  // namespace can
}  // namespace tap

#endif  // IMU_RX_LISTENER_HPP_
