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
    result = bno055.ReadImuData(&imu_data);
    if (!result) {
        std::cout << "Something went wrong!" << std::endl;
    }

    std::cout << "Accelerometer: "
              << "x=" << imu_data.accelerometer[0] << ", "
              << "y=" << imu_data.accelerometer[1] << ", "
              << "z=" << imu_data.accelerometer[2] << std::endl;

    double magnitude = std::sqrt(imu_data.accelerometer[0] * imu_data.accelerometer[0] +
                                 imu_data.accelerometer[1] * imu_data.accelerometer[1] +
                                 imu_data.accelerometer[2] * imu_data.accelerometer[2]);

    std::cout << "Accelerometer Magnitude: "
              << magnitude << std::endl;

    std::cout << "Gyro: "
              << "x=" << imu_data.gyro[0] << ", "
              << "y=" << imu_data.gyro[1] << ", "
              << "z=" << imu_data.gyro[2] << std::endl;

    return 0;
}
