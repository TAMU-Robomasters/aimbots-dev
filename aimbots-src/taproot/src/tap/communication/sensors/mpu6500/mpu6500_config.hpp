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

#ifndef TAPROOT_MPU6500_CONFIG_HPP_
#define TAPROOT_MPU6500_CONFIG_HPP_

// See register datasheet for more information about configuring the mpu6500:
// https://3cfeqx1hf82y3xcoull08ihx-wpengine.netdna-ssl.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf

#define BIT_SHIFT(data, shiftAmnt) ((data) << (shiftAmnt))
#define BIT_MASK(data, mask) ((data) & (mask))

/////////////// MPU6500_CONFIG register data ///////////////

// [6]
// When FIFO is full, additional writes written to FIFO, replacing oldest data (if 1, don't replace
// old data)
#define MPU6500_CONFIG_FIFO_MODE 0b0

// [5:3]
// Disable FSYNC pin data to be sampled
#define MPU6500_CONFIG_EXT_SYNC_SET 0b000

// [2:0]
// Low pass filter settings for gyro and temperature. Gyro bandwidth of 92 Hz. Delay of 3.9 ms
// Temperature sensor bandwidth of 98 Hz. Delay of 2.8 ms
#define MPU6500_CONFIG_DLPF_CONFIG 0b010

#define MPU6500_CONFIG_DATA                                      \
    (BIT_MASK(BIT_SHIFT(MPU6500_CONFIG_FIFO_MODE, 7), 0x40) |    \
     BIT_MASK(BIT_SHIFT(MPU6500_CONFIG_EXT_SYNC_SET, 3), 0x38) | \
     BIT_MASK(BIT_SHIFT(MPU6500_CONFIG_DLPF_CONFIG, 0), 0x07))

/////////////// MPU6500_GYRO_CONFIG ///////////////

// [4:3]
// Gyro full scale select, +/- 2000 dps
#define MPU6500_GYRO_CONFIG_GYRO_FS_SEL 0b11

// [1:0]
// Used to bypass DLPF, set to 11, 01, or 10 to bypass
#define MPU6500_GYRO_CONFIG_FCHOICE_B 0b00

#define MPU6500_GYRO_CONFIG_DATA                                     \
    (BIT_MASK(BIT_SHIFT(MPU6500_GYRO_CONFIG_GYRO_FS_SEL, 3), 0x18) | \
     BIT_MASK(BIT_SHIFT(MPU6500_GYRO_CONFIG_FCHOICE_B, 0), 0x03))

/////////////// MPU6500_ACCEL_CONFIG ///////////////

// [4:3]
// Accel full scale select, +/- 8 g
#define MPU6500_ACCEL_CONFIG_ACCEL_FS_SEL 0b10

#define MPU6500_ACCEL_CONFIG_DATA (BIT_MASK(BIT_SHIFT(MPU6500_ACCEL_CONFIG_ACCEL_FS_SEL, 3), 0x18))

/////////////// MPU6500_ACCEL_CONFIG_2 ///////////////

// [3]
// Used to bypass DLPF, set to 1 to bypass
#define MPU6500_ACCEL_CONFIG_2_ACCEL_FCHOICE_B 0b0

// [2:0]
// Low pass filter settings for accelerometer. Bandwidth of 460 Hz. Delay of 1.94 ms
#define MPU6500_ACCEL_CONFIG_2_A_DLPF_CONFIG 0b000

#define MPU6500_ACCEL_CONFIG_2_DATA                                         \
    (BIT_MASK(BIT_SHIFT(MPU6500_ACCEL_CONFIG_2_ACCEL_FCHOICE_B, 3), 0x08) | \
     BIT_MASK(BIT_SHIFT(MPU6500_ACCEL_CONFIG_2_A_DLPF_CONFIG, 0), 0x07))

////////////// MPU6500_USER_CTRL ///////////////

// [7]
// Enables DMP (digital motion processing) features if 1
#define MPU6500_USER_CTRL_DMP_EN 0b0

// [6]
// Disable FIFO operation mode if 1
#define MPU6500_USER_CTRL_FIFO_EN 0b0

// [5]
// 1 – Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated
// from pins SDA/SDI and SCL/ SCLK.
// 0 – Disable I2C Master I/F module; pins ES_DA and ES_SCL are logically
// driven by pins SDA / SDI and SCL / SCLK
#define MPU6500_USER_CTRL_I2C_MST_EN 0b1

// [4]
// 1 – Reset I2C Slave module and put the serial interface in SPI mode only.
// This bit auto clears after one clock cycle of the internal 20MHz clock.
#define MPU6500_USER_CTRL_I2C_IF_DIS 0b0

// [3]
// Reset the DMP when set to 1 while DMP_EN equals 0, automatically set to 0 when reset triggered
#define MPU6500_USER_CTRL_DMP_RST 0b0

// [2]
// Reset FIFO module if 1
#define MPU6500_USER_CTRL_FIFO_RST 0b0

// [1]
// Reset I2C master module if 1
#define MPU6500_USER_CTRL_I2C_MST_RST 0b0

// [0]
// 1 - Reset all gyro digital signal path, accel digital signal path, and temp
// digital signal path.
#define MPU6500_USER_CTRL_SIG_COND_RST 0b0

#define MPU6500_USER_CTRL_DATA                                                \
    (BIT_MASK(BIT_SHIFT(MPU6500_USER_CTRL_DMP_EN, 0), BIT_SHIFT(1, 0)) |      \
     BIT_MASK(BIT_SHIFT(MPU6500_USER_CTRL_FIFO_EN, 1), BIT_SHIFT(1, 1)) |     \
     BIT_MASK(BIT_SHIFT(MPU6500_USER_CTRL_I2C_MST_EN, 2), BIT_SHIFT(1, 2)) |  \
     BIT_MASK(BIT_SHIFT(MPU6500_USER_CTRL_I2C_IF_DIS, 3), BIT_SHIFT(1, 3)) |  \
     BIT_MASK(BIT_SHIFT(MPU6500_USER_CTRL_DMP_RST, 4), BIT_SHIFT(1, 4)) |     \
     BIT_MASK(BIT_SHIFT(MPU6500_USER_CTRL_FIFO_RST, 5), BIT_SHIFT(1, 5)) |    \
     BIT_MASK(BIT_SHIFT(MPU6500_USER_CTRL_I2C_MST_RST, 6), BIT_SHIFT(1, 6)) | \
     BIT_MASK(BIT_SHIFT(MPU6500_USER_CTRL_SIG_COND_RST, 7), BIT_SHIFT(1, 7)))

////////////// MPU6500_PWR_MGMT_1 //////////////

// NOTE: formatting slightly different for this register since we would like
// to set individual bits at a time instead of configuring a register once with some settings
// Instead of specifying the setting for each bit, either 0 or 1, for MPU6500_PWR_MGMT_1 and 2,
// you should create your own config setting based on your preferences.

// [7]
// Reset the internal registers and restores the default settings. Write a 1 to
// set the reset, the bit will auto clear.
#define MPU6500_PWR_MGMT_1_DEVICE_RESET_BIT BIT_SHIFT(0b1, 7)

// [6]
// When set, the chip is set to sleep mode.
#define MPU6500_PWR_MGMT_1_SLEEP_BIT BIT_SHIFT(0b1, 6)

// [5]
// When set, and SLEEP and STANDBY are not set, the chip will cycle
// between sleep and taking a single sample at a rate determined by
// LP_ACCEL_ODR (MPU-6500 mode) or LP_WAKE_CTRL (MPU-6050
// compatible mode)
// NOTE: When all accelerometer axis are disabled via PWR_MGMT_2
// register bits and cycle is enabled, the chip will wake up at the rate
// determined by the respective registers above, but will not take any samples.
#define MPU6500_PWR_MGMT_1_CYCLE_BIT BIT_SHIFT(0b1, 5)

// [4]
// When set, the gyro drive and pll circuitry are enabled, but the sense paths
// are disabled. This is a low power mode that allows quick enabling of the
// gyros.
#define MPU6500_PWR_MGMT_1_GYRO_STANDBY_BIT BIT_SHIFT(0b1, 4)

// [3]
// When set to 1, this bit disables the temperature sensor.
#define MPU6500_PWR_MGMT_1_TEMP_DIS_BIT BIT_SHIFT(0b1, 3)

// [2:0]
// Auto selects the best available clock source – PLL if ready, else
// use the Internal oscillator
#define MPU6500_PWR_MGMT_1_CLKSEL 0b010

/////////////// SIGNAL_PATH_RESET ///////////////

// [2]
// Reset gyro if 1
#define MPU6500_SIGNAL_PATH_RESET_GYRO_RST BIT_SHIFT(0b1, 2)

// [1]
// Reset accel if 1
#define MPU6500_SIGNAL_PATH_RESET_ACCEL_RST BIT_SHIFT(0b1, 1)

// [0]
// Reset temperature sensor if 1
#define MPU6500_SIGNAL_PATH_RESET_TEMP_RST BIT_SHIFT(0b1, 0)

// Reset all sensors
#define MPU6500_SIGNAL_PATH_RESET_ALL                                           \
    (MPU6500_SIGNAL_PATH_RESET_GYRO_RST | MPU6500_SIGNAL_PATH_RESET_ACCEL_RST | \
     MPU6500_SIGNAL_PATH_RESET_TEMP_RST)

#endif  // TAPROOT_MPU6500_CONFIG_HPP_
