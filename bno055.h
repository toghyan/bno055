/**
 * @file BNO055.h
 * @brief Header file for the BNO055 driver.
 * 
 * This file contains the class definition and associated enums and structs
 * for interfacing with the BNO055 IMU sensor over I2C.
 */

#ifndef BNO055_H
#define BNO055_H

#include <cstdint>
#include <vector>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <exception>
#include <string>

/**
 * @class I2COpenException
 * @brief Exception class for I2C communication errors.
 */
class I2COpenException : public std::exception {
private:
    std::string message;

public:
    /**
     * @brief Constructor for I2COpenException.
     * @param msg Error message.
     */
    I2COpenException(const std::string& msg) : message(msg) {}

    /**
     * @brief Returns the error message.
     * @return const char* Error message.
     */
    virtual const char* what() const noexcept override {
        return message.c_str();
    }
};

/**
 * @enum ReadRegisterAddress
 * @brief Enumeration of readable register addresses.
 */
enum ReadRegisterAddress {
    kChipId = 0x00,       /**< Chip ID register. */
    kPageId = 0x07,       /**< Page ID register. */
    kAccDataXLsb = 0x08,  /**< Accelerometer data X LSB. */
    kOperationMode = 0x3D /**< Operation mode register. */
};

/**
 * @enum WriteRegisterAddress
 * @brief Enumeration of writable register addresses.
 */
enum WriteRegisterAddress {
    kPageIdWrite = 0x07,       /**< Page ID write register. */
    kOperationModeWrite = 0x3D /**< Operation mode write register. */
};

/**
 * @struct SensorData
 * @brief Structure to store IMU sensor data.
 */
struct SensorData {
    double raw_accel_x, raw_accel_y, raw_accel_z;
    double magnetometer_x, magnetometer_y, magnetometer_z;
    double gyro_x, gyro_y, gyro_z;
    double euler_x, euler_y, euler_z;
    double quaternion_w, quaternion_x, quaternion_y, quaternion_z;
    double linear_accel_x, linear_accel_y, linear_accel_z;
    double gravity_x, gravity_y, gravity_z;
    double temperature_c;

    /**
     * @brief Default constructor initializes all values to 0.
     */
    SensorData() 
        : raw_accel_x(0.0), raw_accel_y(0.0), raw_accel_z(0.0),
          magnetometer_x(0.0), magnetometer_y(0.0), magnetometer_z(0.0),
          gyro_x(0.0), gyro_y(0.0), gyro_z(0.0),
          euler_x(0.0), euler_y(0.0), euler_z(0.0),
          quaternion_w(0.0), quaternion_x(0.0), quaternion_y(0.0), quaternion_z(0.0),
          linear_accel_x(0.0), linear_accel_y(0.0), linear_accel_z(0.0),
          gravity_x(0.0), gravity_y(0.0), gravity_z(0.0), temperature_c(0.0) {}
};

/**
 * @enum ImuDataOffsets
 * @brief Enumeration of data register offsets from X acceleration LSB register.
 */
enum ImuDataOffsets {
    ACCEL_X_LSB = 0, ACCEL_X_MSB = 1,
    ACCEL_Y_LSB = 2, ACCEL_Y_MSB = 3,
    ACCEL_Z_LSB = 4, ACCEL_Z_MSB = 5,
    MAG_X_LSB = 6, MAG_X_MSB = 7,
    MAG_Y_LSB = 8, MAG_Y_MSB = 9,
    MAG_Z_LSB = 10, MAG_Z_MSB = 11,
    GYRO_X_LSB = 12, GYRO_X_MSB = 13,
    GYRO_Y_LSB = 14, GYRO_Y_MSB = 15,
    GYRO_Z_LSB = 16, GYRO_Z_MSB = 17,
    EULER_YAW_LSB = 18, EULER_YAW_MSB = 19,
    EULER_ROLL_LSB = 20, EULER_ROLL_MSB = 21,
    EULER_PITCH_LSB = 22, EULER_PITCH_MSB = 23,
    QUAT_W_LSB = 24, QUAT_W_MSB = 25,
    QUAT_X_LSB = 26, QUAT_X_MSB = 27,
    QUAT_Y_LSB = 28, QUAT_Y_MSB = 29,
    QUAT_Z_LSB = 30, QUAT_Z_MSB = 31,
    LIN_ACCEL_X_LSB = 32, LIN_ACCEL_X_MSB = 33,
    LIN_ACCEL_Y_LSB = 34, LIN_ACCEL_Y_MSB = 35,
    LIN_ACCEL_Z_LSB = 36, LIN_ACCEL_Z_MSB = 37,
    GRAV_X_LSB = 38, GRAV_X_MSB = 39,
    GRAV_Y_LSB = 40, GRAV_Y_MSB = 41,
    GRAV_Z_LSB = 42, GRAV_Z_MSB = 43,
    TEMP = 44
};

/**
 * @class Bno055
 * @brief BNO055 driver class.
 */
class Bno055 {
public:
    Bno055(const char *i2c_bus);
    ~Bno055();

    uint8_t ReadChipId();
    uint8_t ReadOperationMode();
    bool SetOperationMode(uint8_t mode);
    bool ReadData(SensorData *imu_data);
    uint8_t ReadAccelerationConfig();
    bool ChangePageId(uint8_t page_id);

private:
    int ReadRegister(uint8_t reg, uint8_t *data, size_t length);
    int WriteRegister(uint8_t reg, const uint8_t *data, size_t length);

    int bus_fd_; /**< File descriptor for I2C bus. */
    const uint8_t device_address_ = 0x28; /**< Default I2C address of BNO055. */
    const uint8_t chip_id_ = 0xA0; /**< Expected chip ID for BNO055. */
    const uint8_t data_buffer_read_size_ = 45; /**< Size of data buffer for sensor reads. */
    const double accel_lsb_to_ms2_ = 0.01; /**< Acceleration LSB to meters per second squared. */
    const double mag_lsb_to_ut_ = 0.0625; /**< Magnetometer LSB to micro Tesla. */
    const double gyro_lsb_to_dps_ = 0.061045402; /**< Gyroscope LSB to degrees per second. */
    const double euler_lsb_to_degree_ = 0.0625; /**< Euler angles LSB to degrees. */
    const double lsb_to_qauternion_ = 1 / 16384.0; /**< Quaternion LSB conversion. */
    const double grav_lsb_to_ms2_ = 0.01; /**< Gravity LSB to meters per second squared. */
};

#endif // BNO055_H
