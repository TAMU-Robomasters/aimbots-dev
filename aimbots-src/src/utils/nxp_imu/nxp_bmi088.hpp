#pragma once

#include "modm/processing/protothread.hpp"
#include "tap/algorithms/MahonyAHRS.h"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/sensors/imu/bmi088/bmi088_data.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/communication/sensors/imu_heater/imu_heater.hpp"
#include "tap/util_macros.hpp"
#include "utils/nxp_imu/nxp_fusion.hpp"

namespace src {
class Drivers;
}

namespace utils {

using namespace tap::communication::sensors;
using namespace tap::communication::sensors::imu;
using namespace tap::communication::sensors::imu::bmi088;

class NXPBMI088 : public Bmi088Data, public ImuInterface {
   public:
    static constexpr Acc::AccRange_t ACC_RANGE = Acc::AccRange::G3;
    static constexpr Gyro::GyroRange_t GYRO_RANGE = Gyro::GyroRange::DPS2000;

    /**
     * The maximum angular velocity in degrees / second that the gyro can read based on GYRO_RANGE
     * specified above.
     */
    static constexpr float GYRO_RANGE_MAX_DS = 2000.0f;

    static constexpr float BMI088_TEMP_FACTOR = 0.125f;
    static constexpr float BMI088_TEMP_OFFSET = 23.0f;

    /**
     * Used to convert raw gyro values to units of degrees / second. Ratio has units
     * (degrees / second) / gyro counts.
     */
    static constexpr float GYRO_DS_PER_GYRO_COUNT = GYRO_RANGE_MAX_DS / 32767.0f;

    /**
     * Refer to page 27 of the bmi088 datasheet for explination of this equation.
     * Used to convert raw accel values to units m/s^2. Ratio has units (m/s^2) / acc counts.
     */
    static constexpr float ACC_G_PER_ACC_COUNT =
        modm::pow(2, ACC_RANGE.value + 1) * 1.5f * tap::algorithms::ACCELERATION_GRAVITY / 32768.0f;

    /**
     * The number of samples we take in order to determine the mpu offsets.
     */
    static constexpr float BMI088_OFFSET_SAMPLES = 1000;

    NXPBMI088(src::Drivers *drivers);

    /**
     * Starts and configures the bmi088. Blocks for < 200 ms.
     */
    mockable void initialize(float sampleFrequency);

    /**
     * Call this function at 500 Hz. Reads IMU data and performs the mahony AHRS algorithm to
     * compute pitch/roll/yaw.
     *
     * @note This function blocks for 129 microseconds to read registers from the BMI088.
     */
    mockable void periodicIMUUpdate();

    /**
     * Returns the state of the IMU. Can be not connected, connected but not calibrated, or
     * calibrated. When not connected, IMU data will be garbage. When not calibrated, IMU data is
     * valid but the computed yaw angle data will drift. When calibrating, the IMU data is invalid.
     * When calibrated, the IMU data is valid and assuming proper calibration the IMU data should
     * not drift.
     *
     * To be safe, whenever you call the functions below, call this function to ensure
     * the data you are about to receive is not garbage.
     */
    mockable ImuState getImuState() const;

    /**
     * When this function is called, the bmi088 enters a calibration state during which time,
     * gyro/accel calibration offsets will be computed and the mahony algorithm reset. When
     * calibrating, angle, accelerometer, and gyroscope values will return 0. When calibrating
     * the BMI088 should be level, otherwise the IMU will be calibrated incorrectly.
     */
    mockable void requestRecalibration();

    inline const char *getName() const final_mockable { return "bmi088"; }

    mockable inline float getYaw() final_mockable { return nxpAlgorithm.getYaw(); }
    mockable inline float getPitch() final_mockable { return nxpAlgorithm.getPitch(); }
    mockable inline float getRoll() final_mockable { return nxpAlgorithm.getRoll(); }

    mockable inline float getGx() final_mockable { return data.gyroDegPerSec[ImuData::X]; }  // gyro
    mockable inline float getGy() final_mockable { return data.gyroDegPerSec[ImuData::Y]; }
    mockable inline float getGz() final_mockable { return data.gyroDegPerSec[ImuData::Z]; }

    mockable inline float getAx() final_mockable { return data.accG[ImuData::X]; }  // aceleramoter
    mockable inline float getAy() final_mockable { return data.accG[ImuData::Y]; }
    mockable inline float getAz() final_mockable { return data.accG[ImuData::Z]; }

    mockable inline float getTemp() final_mockable { return data.temperature; }

    mockable inline uint32_t getPrevIMUDataReceivedTime() const { return prevIMUDataReceivedTime; }

   private:
    static constexpr uint16_t RAW_TEMPERATURE_TO_APPLY_OFFSET = 1023;
    /// Offset parsed temperature reading by this amount if > RAW_TEMPERATURE_TO_APPLY_OFFSET.
    static constexpr int16_t RAW_TEMPERATURE_OFFSET = -2048;

    struct ImuData {
        enum Axis {
            X = 0,
            Y = 1,
            Z = 2,
        };

        float accRaw[3] = {};
        float gyroRaw[3] = {};
        float accOffsetRaw[3] = {};
        float gyroOffsetRaw[3] = {};
        float accG[3] = {};
        float gyroDegPerSec[3] = {};

        float temperature;
    } data;

    src::Drivers *drivers;

    ImuState imuState = ImuState::IMU_NOT_CONNECTED;

    // new algorithim
    NXPSensorFusion nxpAlgorithm;

    imu_heater::ImuHeater imuHeater;

    int calibrationSample = 0;

    uint32_t prevIMUDataReceivedTime = 0;

    void initializeAcc();
    void initializeGyro();

    void computeOffsets();

    void setAndCheckAccRegister(Acc::Register reg, Acc::Registers_t value);

    void setAndCheckGyroRegister(Gyro::Register reg, Gyro::Registers_t value);

    /**
     * Parses raw temperature data from the IMU.
     *
     * tempMsg and tempLsb are registers containing the temperature sensor data output. The data is
     * stored in an 11-bit value in 2's complement format. The resolution is 0.125 deg C / LSB. The
     * temperature returned is as a float in degrees C.
     */
    static inline float parseTemp(uint8_t tempMsb, uint8_t tempLsb) {
        uint16_t temp =
            (static_cast<uint16_t>(tempMsb) * 8) + (static_cast<uint16_t>(tempLsb) / 32);

        int16_t shiftedTemp = static_cast<int16_t>(temp);

        if (temp > RAW_TEMPERATURE_TO_APPLY_OFFSET) {
            shiftedTemp += RAW_TEMPERATURE_OFFSET;
        }

        return static_cast<float>(shiftedTemp) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }
};

}  // namespace utils