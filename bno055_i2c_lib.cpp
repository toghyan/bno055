#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>
#include <cstdint>
#include "bno055_i2c_lib.h"


Bno055I2c::Bno055I2c(const char *i2c_bus) {
    int bus_fd = i2c_open("/dev/i2c-0");  // Open the I2C bus
    if (bus_fd == -1) {
        throw I2COpenException("Failed to open I2C bus on " + std::string(i2c_bus));
    }
    // Ensure all the bits re zero.
    memset(&i2c_device_, 0, sizeof(i2c_device_));
    // i2c initiallization. Needs to be called before reading/writing. 
    i2c_init_device(&i2c_device_);

    // Assign the device address and bus number.
    i2c_device_.bus = bus_fd;
    i2c_device_.addr = device_address_;

    // Validate that we can read the chip ID.
    uint8_t value = ReadChipId();
    if (value != chip_id_){
        throw I2COpenException("Chip ID is incorrect.");
        std::ostringstream msg;
        msg << "I2C open failed: Incorrect chip value (Read: 0x"
        << std::hex << std::uppercase << std::setw(2) << std::setfill('0') 
        << static_cast<unsigned int>(value)  // Cast to unsigned int for correct formatting
        << ", chip ID: 0x"
        << static_cast<unsigned int>(chip_id_) << ")";  // Cast chipID as well for consistency
        throw I2COpenException(msg.str());
    }
}

Bno055I2c::~Bno055I2c() {
    i2c_close(i2c_device_.bus);
}

uint8_t Bno055I2c::ReadChipId() {
    uint8_t buffer[1];               // Buffer to store the read data.
    ssize_t bytes_read = i2c_read(&i2c_device_, kChipId, buffer, sizeof(buffer));
    return buffer[0];
}

uint8_t Bno055I2c::ReadOperationMode() {
    uint8_t buffer[1];               // Buffer to store the read data.
    ssize_t bytes_read = i2c_read(&i2c_device_, kOperationMode, buffer, sizeof(buffer));
    return buffer[0];
}

bool Bno055I2c::SetOperationMode(uint8_t buffer) {
    ssize_t result = i2c_write(&i2c_device_, kOperationMode, &buffer, 1);
    return result != -1; 
}

bool Bno055I2c::ReadImuData(SensorData *imu_data) {
    uint8_t buffer[18];
    ssize_t bytes_read = i2c_read(&i2c_device_, kAccDataXLsb, buffer, sizeof(buffer));
    if (bytes_read == -1) return false;
    // Calculate the accelerometer and gyro data.
    for (int i = 0; i < 3; i++){
        imu_data->accelerometer[i] = ConvertToTwosComplement14Bit(buffer[2 * i + 1], buffer[2 * i]) * accelerometer_lsb_to_ms2_;
        int16_t raw_gyro = (static_cast<int16_t>(buffer[2 * i + 13]) << 8) | buffer[2 * i + 12];
        imu_data->gyro[i] = raw_gyro * gyro_lsb_to_dps_;
    }
    return true;
}

uint8_t Bno055I2c::ReadAccelerationConfig() {
    uint8_t buffer[1];               // Buffer to store the read data.
    ssize_t bytes_read = i2c_read(&i2c_device_, 0x08, buffer, 1);
    return buffer[0];
}

bool Bno055I2c::ChangePageId(uint8_t page_id) {
    ssize_t result = i2c_write(&i2c_device_, kPageId, &page_id, 1);
    return result != -1; 
}

int16_t Bno055I2c::ConvertToTwosComplement14Bit(const uint8_t msb, const uint8_t lsb) {
    // Combine MSB and LSB to form a 16-bit number
    uint16_t raw = (msb << 8) | lsb;
    // Mask the combined value to ensure only 14 bits are used
    raw &= 0x3FFF;
    // Convert to a 14-bit two's complement value
    int16_t twos_complement = (int16_t)(raw << 2) / 4;

    return twos_complement;
}