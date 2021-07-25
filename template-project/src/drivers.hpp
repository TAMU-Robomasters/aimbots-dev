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

#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/architecture/profiler.hpp"
#include "tap/mock/analog_mock.hpp"
#include "tap/mock/can_mock.hpp"
#include "tap/mock/can_rx_handler_mock.hpp"
#include "tap/mock/command_mapper_mock.hpp"
#include "tap/mock/command_scheduler_mock.hpp"
#include "tap/mock/control_operator_interface_mock.hpp"
#include "tap/mock/digital_mock.hpp"
#include "tap/mock/dji_motor_terminal_serial_handler_mock.hpp"
#include "tap/mock/dji_motor_tx_handler_mock.hpp"
#include "tap/mock/error_controller_mock.hpp"
#include "tap/mock/imu_rx_listener_mock.hpp"
#include "tap/mock/leds_mock.hpp"
#include "tap/mock/mpu6500_mock.hpp"
#include "tap/mock/pwm_mock.hpp"
#include "tap/mock/ref_serial_mock.hpp"
#include "tap/mock/remote_mock.hpp"
#include "tap/mock/scheduler_terminal_handler_mock.hpp"
#include "tap/mock/terminal_serial_mock.hpp"
#include "tap/mock/uart_mock.hpp"
/* Start user mock includes */
/* End user mock includes */
#else
#include "tap/architecture/profiler.hpp"
#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_rx_handler.hpp"
#include "tap/communication/can/imu_rx_listener.hpp"
#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/gpio/leds.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "tap/communication/remote.hpp"
#include "tap/communication/sensors/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/terminal_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/control_operator_interface.hpp"
#include "tap/control/scheduler_terminal_handler.hpp"
#include "tap/errors/error_controller.hpp"
#include "tap/motor/dji_motor_terminal_serial_handler.hpp"
#include "tap/motor/dji_motor_tx_handler.hpp"
/* Start user mock includes */
/* End user mock includes */
#endif

namespace tap
{
class Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers()
        : can(),
          canRxHandler(this),
          analog(),
          digital(),
          leds(),
          pwm(),
          remote(this),
          mpu6500(this),
          uart(),
          refSerial(this),
#ifdef ENV_UNIT_TESTS
          commandScheduler(this),
#else
          commandScheduler(this, true),
#endif
          controlOperatorInterface(this),
          commandMapper(this),
          errorController(this),
          terminalSerial(this),
          djiMotorTxHandler(this),
          profiler(),
          djiMotorTerminalSerialHandler(this),
          schedulerTerminalHandler(this),
          imuRxHandler(this)
    /* Begin user constructor defines */
    /* End user mock drivers defines */ {}

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    testing::NiceMock<mock::CanMock> can;
    testing::NiceMock<mock::CanRxHandlerMock> canRxHandler;
    testing::NiceMock<mock::AnalogMock> analog;
    testing::NiceMock<mock::DigitalMock> digital;
    testing::NiceMock<mock::LedsMock> leds;
    testing::NiceMock<mock::PwmMock> pwm;
    testing::NiceMock<mock::RemoteMock> remote;
    testing::NiceMock<mock::Mpu6500Mock> mpu6500;
    testing::NiceMock<mock::UartMock> uart;
    testing::NiceMock<mock::RefSerialMock> refSerial;
    testing::NiceMock<mock::CommandSchedulerMock> commandScheduler;
    testing::NiceMock<mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    testing::NiceMock<mock::CommandMapperMock> commandMapper;
    mock::ErrorControllerMock errorController;
    testing::NiceMock<mock::TerminalSerialMock> terminalSerial;
    testing::NiceMock<mock::DjiMotorTxHandlerMock> djiMotorTxHandler;
    arch::Profiler profiler;
    testing::NiceMock<mock::DjiMotorTerminalSerialHandlerMock> djiMotorTerminalSerialHandler;
    testing::NiceMock<mock::SchedulerTerminalHandlerMock> schedulerTerminalHandler;
    testing::NiceMock<mock::ImuRxListenerMock> imuRxHandler;
/* Begin user mock drivers defines */
/* End user mock drivers defines */
#else
public:
    can::Can can;
    can::CanRxHandler canRxHandler;
    gpio::Analog analog;
    gpio::Digital digital;
    gpio::Leds leds;
    gpio::Pwm pwm;
    Remote remote;
    sensors::Mpu6500 mpu6500;
    serial::Uart uart;
    serial::RefSerial refSerial;
    control::CommandScheduler commandScheduler;
    control::ControlOperatorInterface controlOperatorInterface;
    control::CommandMapper commandMapper;
    errors::ErrorController errorController;
    communication::serial::TerminalSerial terminalSerial;
    motor::DjiMotorTxHandler djiMotorTxHandler;
    arch::Profiler profiler;
    motor::DjiMotorTerminalSerialHandler djiMotorTerminalSerialHandler;
    control::SchedulerTerminalHandler schedulerTerminalHandler;
    can::ImuRxListener imuRxHandler;
/* Begin user drivers defines */
/* End user drivers defines */
#endif
};  // class Drivers

}  // namespace tap

#endif  // DRIVERS_HPP_
