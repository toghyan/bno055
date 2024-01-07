#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>
#include <cstdint>
#include "bno055_i2c_lib.h"


Bno055I2c::Bno055I2c(const char *i2c_bus){
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
    i2c_device_.addr = kDeviceAddress;

    // Validate that we can read the chip ID.
    uint8_t value = ReadChipId();
    if (value != kChipId){
        throw I2COpenException("Chip ID is incorrect.");
        std::ostringstream msg;
        msg << "I2C open failed: value is not 0xA0 (value: 0x"
        << std::hex << std::uppercase << std::setw(2) << std::setfill('0') 
        << static_cast<unsigned int>(value)  // Cast to unsigned int for correct formatting
        << ", chip ID: 0x"
        << static_cast<unsigned int>(kChipId) << ")";  // Cast chipID as well for consistency
        throw I2COpenException(msg.str());
    }
}

Bno055I2c::~Bno055I2c(){
    i2c_close(i2c_device_.bus);
}

uint8_t Bno055I2c::ReadChipId(){
    unsigned int register_address = 0x00;  // Chip ID register address.
    uint8_t buffer[1];               // Buffer to store the read data.
    size_t length_to_read = sizeof(buffer);

    ssize_t bytes_read = i2c_read(&i2c_device_, register_address, buffer, length_to_read);

    return buffer[0];
}