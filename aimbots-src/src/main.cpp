/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aimbots-src.
 *
 * aimbots-src is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aimbots-src is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aimbots-src.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED
/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

#include "modm/architecture/interface/delay.hpp"
#include "tap/board/board.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "drivers.hpp"
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"
//
#include "robots/robot_control.hpp"
#include "utils/custom_imu/magnetometer/ist8310_data.hpp"

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);

//bmi088 is at 1000Hz.. coincidence? I think not!!11!
static constexpr float SAMPLE_FREQUENCY = 1000.0f;

int main() {
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    src::Drivers *drivers = src::DoNotUse_getDrivers();

    Board::initialize();

    //desperate test code
    //with magic numbers included
    tap::arch::PeriodicMilliTimer mainLoopTimeout(1000.0f / SAMPLE_FREQUENCY);

    initializeIo(drivers);
    src::Control::initializeSubsystemCommands(drivers);

#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    while (1) {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));

        //every 1ms...
        if (mainLoopTimeout.execute())
        {
            drivers->bmi088.periodicIMUUpdate();
        }
        if (sendMotorTimeout.execute()) {
            // PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(src::Drivers *drivers) {
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();
    // drivers->mpu6500.init();
    //drivers->imu.initialize(100.0f);
    drivers->bmi088.initialize(SAMPLE_FREQUENCY,0.1f,0.0f);
    //drivers->imu.requestRecalibration();
    drivers->bmi088.requestRecalibration();

    drivers->refSerial.initialize();
    drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
    drivers->magnetometer.init();
}

float yaw, pitch, roll;
float magX, magY, magZ;
tap::communication::sensors::imu::ImuInterface::ImuState imuStatus;

static void updateIo(src::Drivers *drivers) {
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();

    // if (drivers->imu.getImuState() == tap::communication::sensors::imu::ImuInterface::ImuState::IMU_CALIBRATED) {
    //drivers->imu.periodicIMUUpdate();
    //drivers->bmi088.periodicIMUUpdate();

    //imu data with nxp alg
    // yaw = drivers->imu.getYaw();
    // pitch = drivers->imu.getPitch();
    // roll = drivers->imu.getRoll();

    yaw = drivers->bmi088.getYaw();
    pitch = drivers->bmi088.getRoll();
    roll = drivers->bmi088.getPitch();
    imuStatus = drivers->bmi088.getImuState();

    magX = drivers->magnetometer.getX();
    magY = drivers->magnetometer.getY();
    magZ = drivers->magnetometer.getZ();
    // }
}
