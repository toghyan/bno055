#ifndef BNO055_I2C_LIB_H
#define BNO055_I2C_LIB_H

#include <cstdint>
#include "i2c.h"


class I2COpenException : public std::exception {
private:
    std::string message;

public:
    I2COpenException(const std::string& msg) : message(msg) {}

    virtual const char* what() const noexcept override {
        return message.c_str();
    }
};


class Bno055I2c {
public:
    Bno055I2c(const char *i2c_bus);   // Constructor
    ~Bno055I2c();  // Destructor

    // Public methods for interacting with the BNO055 sensor
    unsigned char ReadChipId();

private:
    // Member variables
    const uint8_t kDeviceAddress = 0x28;  // I2C address of the BNO055 sensor
    const uint8_t kChipIdRegisterAddress = 0x00;
    const uint8_t kChipId = 0xA0;
    I2CDevice i2c_device_;
};

#endif // BNO055_I2C_LIB_H
