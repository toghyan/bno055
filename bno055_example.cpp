#include <iostream>
#include <string>
#include <cstdint>
#include <cmath>
#include "bno055.h"


int main() {
    Bno055 bno055("/dev/i2c-0");  // Open the I2C bus
    uint8_t chip_id = bno055.ReadChipId();
    printf("Chip ID: 0x%02X\n", chip_id);

    uint8_t opration_mode = bno055.ReadOperationMode();
    printf("Operation Mode: 0x%02X\n", opration_mode);
    // Set operation mode to NDOF.
    bool result = bno055.SetOperationMode(0x0C);

    opration_mode = bno055.ReadOperationMode();
    printf("Operation Mode: 0x%02X\n", opration_mode);

    result = bno055.ChangePageId(0x01);

    if (!result) {
        std::cout << "Something went wrong!" << std::endl;
    }

    uint8_t acceleration_config = bno055.ReadAccelerationConfig();
    printf("Acceleration Config: 0x%02X\n", acceleration_config);

    result = bno055.ChangePageId(0x00);

    if (!result) {
        std::cout << "Something went wrong!" << std::endl;
    }

    SensorData imu_data;
    result = bno055.ReadData(&imu_data);
    if (!result) {
        std::cout << "Something went wrong!" << std::endl;
    }

    std::cout << "Accelerometer: "
              << "x=" << imu_data.raw_accel_x << ", "
              << "y=" << imu_data.raw_accel_y << ", "
              << "z=" << imu_data.raw_accel_z << std::endl;

    double magnitude = std::sqrt(imu_data.raw_accel_x * imu_data.raw_accel_x +
                                 imu_data.raw_accel_y * imu_data.raw_accel_y +
                                 imu_data.raw_accel_z * imu_data.raw_accel_z);

    std::cout << "Accelerometer Magnitude: "
              << magnitude << std::endl;

    std::cout << "Gyro: "
              << "x=" << imu_data.gyro_x << ", "
              << "y=" << imu_data.gyro_y << ", "
              << "z=" << imu_data.gyro_z << std::endl;

    std::cout << "Linear Acceleration: "
              << "x=" << imu_data.linear_accel_x << ", "
              << "y=" << imu_data.linear_accel_y << ", "
              << "z=" << imu_data.linear_accel_z << std::endl;

    std::cout  << "Quaternion: "
               << "W=" << imu_data.quaternion_w << ", "
               << "X=" << imu_data.quaternion_x << ", "
               << "Y=" << imu_data.quaternion_y << ", "
               << "Z=" << imu_data.quaternion_z << ", " << std::endl;

    std::cout << "Temperature (C)=" << imu_data.temperature_c << std::endl; 

    return 0;
}
