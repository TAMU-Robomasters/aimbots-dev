    inline Vector3f getChassisIMUOrientationAtTime(uint32_t time_ms) {
        // assume 2 ms delay between gimbal updates
        int index = std::min(time_ms / KINEMATIC_REFRESH_RATE, CHASSIS_IMU_BUFFER_SIZE - 1);
        return chassisIMUHistoryBuffer[index];
    }