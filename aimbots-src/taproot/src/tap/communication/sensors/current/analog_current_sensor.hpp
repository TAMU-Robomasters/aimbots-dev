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

#ifndef TAPROOT_ANALOG_CURRENT_SENSOR_HPP_
#define TAPROOT_ANALOG_CURRENT_SENSOR_HPP_

#include "tap/communication/gpio/analog.hpp"

#include "current_sensor_interface.hpp"

namespace tap::communication::sensors::current
{
/**
 * Linear analog current sensor that reads current from a sensor attached to an analog pin,
 * applying a moving average to data read each time `getCurrentMa` is called. As such, for optimal
 * performance, call `getCurrentMa()` periodically at some consistent rate.
 */
class AnalogCurrentSensor : public CurrentSensorInterface
{
public:
    /**
     * Parameters for the analog current sensor.
     */
    struct Config
    {
        const tap::gpio::Analog *analogDriver;
        const tap::gpio::Analog::Pin analogSensorPin;
        /**
         * The conversion factor from millivolts (read in by the analog driver) to current in
         * milliamps
         *
         * The sensor returns an analog reading in mV. `currentSensorMaPerMv` has units mA/mV, so
         * multiplying the analog reading by `currentSensorMaPerMv` results in a value in mV.
         */
        const float currentSensorMaPerMv;
        /**
         * When zero milliamps are being read, the raw analog value that the sensor reports in
         * millivolts
         */
        const float currentSensorZeroMv;
        /**
         * Alpha gain to be used to low pass filter the raw analog data.
         */
        const float currentSensorLowPassAlpha;
    };

    AnalogCurrentSensor(const Config &config);

    float getCurrentMa() const override;

    void update() override;

private:
    const Config config;

    float prevCurrent = 0.0f;
};

}  // namespace tap::communication::sensors::current

#endif  // TAPROOT_ANALOG_CURRENT_SENSOR_HPP_
