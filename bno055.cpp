#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "bno055.h"


/**
 * @brief Constructor for the Bno055 class.
 * 
 * Opens the I2C bus and verifies the BNO055 chip ID.
 * @param i2c_bus Path to the I2C bus device file.
 * @throws I2COpenException if the bus cannot be opened or the chip ID is incorrect.
 */
Bno055::Bno055(const char *i2c_bus) {
    bus_fd_ = open(i2c_bus, O_RDWR);
    if (bus_fd_ < 0) {
        throw I2COpenException("Failed to open I2C bus on " + std::string(i2c_bus));
    }

    if (ioctl(bus_fd_, I2C_SLAVE, device_address_) < 0) {
        close(bus_fd_);
        throw I2COpenException("Failed to set I2C slave address.");
    }

    uint8_t value = ReadChipId();
    if (value != chip_id_) {
        close(bus_fd_);
        std::ostringstream msg;
        msg << "I2C open failed: Incorrect chip value (Read: 0x"
            << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<unsigned int>(value)
            << ", Expected: 0x"
            << static_cast<unsigned int>(chip_id_) << ")";
        throw I2COpenException(msg.str());
    }
}

/**
 * @brief Destructor for the Bno055 class.
 * 
 * Closes the I2C bus connection if open.
 */
Bno055::~Bno055() {
    if (bus_fd_ >= 0) {
        close(bus_fd_);
    }
}

/**
 * @brief Reads the BNO055 chip ID.
 * 
 * @return The chip ID if successful, 0 otherwise.
 */
uint8_t Bno055::ReadChipId() {
    uint8_t buffer;
    if (ReadRegister(kChipId, &buffer, 1)) {
        return buffer;
    }
    return 0;
}

/**
 * @brief Reads the current operation mode of the BNO055.
 * 
 * @return The operation mode if successful, 0 otherwise.
 */
uint8_t Bno055::ReadOperationMode() {
    uint8_t buffer;
    if (ReadRegister(kOperationMode, &buffer, 1)) {
        return buffer;
    }
    return 0;
}

/**
 * @brief Sets the operation mode of the BNO055.
 * 
 * @param mode The operation mode to set.
 * @return True if successful, false otherwise.
 */
bool Bno055::SetOperationMode(uint8_t mode) {
    return WriteRegister(kOperationMode, &mode, 1);
}

/**
 * @brief Reads sensor data from the BNO055 and populates the provided SensorData structure.
 * 
 * This function reads raw acceleration, magnetometer, gyroscope, Euler angles, quaternion, 
 * linear acceleration, gravity, and temperature data from the BNO055 sensor. The retrieved 
 * data is then converted to standard units and stored in the provided SensorData structure.
 * 
 * @param[out] data Pointer to a SensorData structure where the retrieved sensor values will be stored.
 * @return True if the data was successfully read and stored, false otherwise.
 * 
 * @note The function assumes the sensor is properly configured and in the correct operating mode.
 *       If the I2C read operation fails, the function returns false.
 */
bool Bno055::ReadData(SensorData *data) {
    uint8_t buffer[data_buffer_read_size_];
    if (!ReadRegister(kAccDataXLsb, buffer, sizeof(buffer))) {
        return false;
    }

    data->raw_accel_x = (static_cast<int16_t>(static_cast<int8_t>(buffer[ACCEL_X_MSB])) << 8 | buffer[ACCEL_X_LSB]) * accel_lsb_to_ms2_;
    data->raw_accel_y = (static_cast<int16_t>(static_cast<int8_t>(buffer[ACCEL_Y_MSB])) << 8 | buffer[ACCEL_Y_LSB]) * accel_lsb_to_ms2_;
    data->raw_accel_z = (static_cast<int16_t>(static_cast<int8_t>(buffer[ACCEL_Z_MSB])) << 8 | buffer[ACCEL_Z_LSB]) * accel_lsb_to_ms2_;

    data->magnetometer_x = (static_cast<int16_t>(static_cast<int8_t>(buffer[MAG_X_MSB])) << 8 | buffer[MAG_X_LSB]) * mag_lsb_to_ut_;
    data->magnetometer_y = (static_cast<int16_t>(static_cast<int8_t>(buffer[MAG_Y_MSB])) << 8 | buffer[MAG_Y_LSB]) * mag_lsb_to_ut_;
    data->magnetometer_z = (static_cast<int16_t>(static_cast<int8_t>(buffer[MAG_Z_MSB])) << 8 | buffer[MAG_Z_LSB]) * mag_lsb_to_ut_;

    data->gyro_x = (static_cast<int16_t>(static_cast<int8_t>(buffer[GYRO_X_MSB])) << 8 | buffer[GYRO_X_LSB]) * gyro_lsb_to_dps_;
    data->gyro_y = (static_cast<int16_t>(static_cast<int8_t>(buffer[GYRO_Y_MSB])) << 8 | buffer[GYRO_Y_LSB]) * gyro_lsb_to_dps_;
    data->gyro_z = (static_cast<int16_t>(static_cast<int8_t>(buffer[GYRO_Z_MSB])) << 8 | buffer[GYRO_Z_LSB]) * gyro_lsb_to_dps_;

    data->euler_x = (static_cast<int16_t>(static_cast<int8_t>(buffer[EULER_ROLL_MSB])) << 8 | buffer[EULER_ROLL_LSB]) * euler_lsb_to_degree_;
    data->euler_y = (static_cast<int16_t>(static_cast<int8_t>(buffer[EULER_PITCH_MSB])) << 8 | buffer[EULER_PITCH_LSB]) * euler_lsb_to_degree_;
    data->euler_z = (static_cast<int16_t>(static_cast<int8_t>(buffer[EULER_YAW_MSB])) << 8 | buffer[EULER_YAW_LSB]) * euler_lsb_to_degree_;

    data->quaternion_w = (static_cast<int16_t>(static_cast<int8_t>(buffer[QUAT_W_MSB])) << 8 | buffer[QUAT_W_LSB]) * lsb_to_qauternion_;
    data->quaternion_x = (static_cast<int16_t>(static_cast<int8_t>(buffer[QUAT_X_MSB])) << 8 | buffer[QUAT_X_LSB]) * lsb_to_qauternion_;
    data->quaternion_y = (static_cast<int16_t>(static_cast<int8_t>(buffer[QUAT_Y_MSB])) << 8 | buffer[QUAT_Y_LSB]) * lsb_to_qauternion_;
    data->quaternion_z = (static_cast<int16_t>(static_cast<int8_t>(buffer[QUAT_Z_MSB])) << 8 | buffer[QUAT_Z_LSB]) * lsb_to_qauternion_;

    data->linear_accel_x = (static_cast<int16_t>(static_cast<int8_t>(buffer[LIN_ACCEL_X_MSB])) << 8 | buffer[LIN_ACCEL_X_LSB]) * accel_lsb_to_ms2_;
    data->linear_accel_y = (static_cast<int16_t>(static_cast<int8_t>(buffer[LIN_ACCEL_Y_MSB])) << 8 | buffer[LIN_ACCEL_Y_LSB]) * accel_lsb_to_ms2_;
    data->linear_accel_z = (static_cast<int16_t>(static_cast<int8_t>(buffer[LIN_ACCEL_Z_MSB])) << 8 | buffer[LIN_ACCEL_Z_LSB]) * accel_lsb_to_ms2_;

    data->linear_accel_x = (static_cast<int16_t>(static_cast<int8_t>(buffer[LIN_ACCEL_X_MSB])) << 8 | buffer[LIN_ACCEL_X_LSB]) * accel_lsb_to_ms2_;
    data->linear_accel_y = (static_cast<int16_t>(static_cast<int8_t>(buffer[LIN_ACCEL_Y_MSB])) << 8 | buffer[LIN_ACCEL_Y_LSB]) * accel_lsb_to_ms2_;
    data->linear_accel_z = (static_cast<int16_t>(static_cast<int8_t>(buffer[LIN_ACCEL_Z_MSB])) << 8 | buffer[LIN_ACCEL_Z_LSB]) * accel_lsb_to_ms2_;

    data->gravity_x = (static_cast<int16_t>(static_cast<int8_t>(buffer[GRAV_X_MSB])) << 8 | buffer[GRAV_X_LSB]) * grav_lsb_to_ms2_;
    data->gravity_y = (static_cast<int16_t>(static_cast<int8_t>(buffer[GRAV_Y_MSB])) << 8 | buffer[GRAV_Y_LSB]) * grav_lsb_to_ms2_;
    data->gravity_z = (static_cast<int16_t>(static_cast<int8_t>(buffer[GRAV_Z_MSB])) << 8 | buffer[GRAV_Z_LSB]) * grav_lsb_to_ms2_;

    data->temperature_c = buffer[TEMP];

    return true;
}

/**
 * @brief Reads the acceleration configuration register.
 * 
 * @return The acceleration configuration value.
 */
uint8_t Bno055::ReadAccelerationConfig() {
    uint8_t buffer;
    if (ReadRegister(0x08, &buffer, 1)) {
        return buffer;
    }
    return 0;
}

/**
 * @brief Changes the Page ID of the BNO055.
 * 
 * @param page_id The new page ID to set.
 * @return True if successful, false otherwise.
 */
bool Bno055::ChangePageId(uint8_t page_id) {
    return WriteRegister(kPageId, &page_id, 1);
}

/**
 * @brief Reads a register from the BNO055.
 * 
 * @param reg The register address to read from.
 * @param data Pointer to the buffer to store read data.
 * @param length Number of bytes to read.
 * @return True if successful, false otherwise.
 */
int Bno055::ReadRegister(uint8_t reg, uint8_t *data, size_t length) {
    if (write(bus_fd_, &reg, 1) != 1) {
        return false;
    }
    if (read(bus_fd_, data, length) != static_cast<ssize_t>(length)) {
        return false;
    }
    return true;
}

/**
 * @brief Writes data to a register on the BNO055.
 * 
 * @param reg The register address to write to.
 * @param data Pointer to the data buffer to write.
 * @param length Number of bytes to write.
 * @return True if successful, false otherwise.
 */
int Bno055::WriteRegister(uint8_t reg, const uint8_t *data, size_t length) {
    uint8_t buffer[length + 1];
    buffer[0] = reg;
    memcpy(&buffer[1], data, length);
    return write(bus_fd_, buffer, length + 1) == static_cast<ssize_t>(length + 1);
}
