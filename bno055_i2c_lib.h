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

enum ReadRegisterAddress{
    kChipId = 0x00,
    kAccId = 0x01,
    kMagId = 0x02,
    kGyrId = 0x03,
    kSwRevIdLsb = 0x04,
    kSwRevIdMsb = 0x05,
    kBootLoaderVersion = 0x06,
    kPageId = 0x07,
    kAccDataXLsb = 0x08,
    kAccDataXMsb = 0x09,
    kAccDataYLsb = 0x0A,
    kAccDataYMsb = 0x0B,
    kAccDataZLsb = 0x0C,
    kAccDataZMsb = 0x0D,
    kMagDataXLsb = 0x0E,
    kMagDataXMsb = 0x0F,
    kMagDataYLsb = 0x10,
    kMagDataYMsb = 0x11,
    kMagDataZLsb = 0x12,
    kMagDataZMsb = 0x13,
    kGyrDataXLsb = 0x14,
    kGyrDataXMsb = 0x15,
    kGyrDataYLsb = 0x16,
    kGyrDataYMsb = 0x17,
    kGyrDataZLsb = 0x18,
    kGyrDataZMsb = 0x19,
    kEulHeadingLsb = 0x1A,
    kEulHeadingMsb = 0x1B,
    kEulRollLsb = 0x1C,
    kEulRollLsb = 0x1D,
    kEulPitchLsb = 0x1E,
    kEulPitchMsb = 0x1F,
    kQuatDataWLsb = 0x20,
    kQuatDataWMsb = 0x21,
    kQuatDataXLsb = 0x22,
    kQuatDataXMsb = 0x23,
    kQuatDataYLsb = 0x24,
    kQuatDataYMsb = 0x25,
    kQuatDataZLsb = 0x26,
    kQuatDataZMsb = 0x27,
    kLinearAccDataXLsb = 0x28,
    kLinearAccDataXMsb = 0x29,
    kLinearAccDataXLsb = 0x2A,
    kLinearAccDataXMsb = 0x2B,
    kLinearAccDataXLsb = 0x2C,
    kLinearAccDataXMsb = 0x2D,
    kGravityDataXLsb = 0x2E,
    kGravityDataXMsb = 0x2F,
    kGravityDataYLsb = 0x30,
    kGravityDataYMsb = 0x31,
    kGravityDataZLsb = 0x32,
    kGravityDataZMsb = 0x33,
    kTemperature = 0x34,
    kCalibrationStatus = 0x35,
    kSelfTestResult = 0x36,
    kInterruptStatus = 0x37,
    kSystemStatusCode = 0x39,
    kStstemErrorCode = 0x3A,
};

enum WriteRegisterAddress {
    kUnitSelection = 0x3B,
    kOperationMode = 0x3D,
    kPowerMode = 0x3E,
    kSystemTrigger = 0x3F,
    kTempSource = 0x40,
    kAxisMapConfig = 0x41,
    kAxisMapSign = 0x42,
    kAccOffsetXLsb = 0x55,
    kAccOffsetXMsb = 0x56,
    kAccOffsetYLsb = 0x57,
    kAccOffsetYMsb = 0x58,
    kAccOffsetZLsb = 0x59,
    kAccOffsetZMsb = 0x5A,
    kMagOffsetXLsb = 0x5B,
    kMagOffsetXMsb = 0x5C,
    kMagOffsetYLsb = 0x5D,
    kMagOffsetYMsb = 0x5E,
    kMagOffsetZLsb = 0x5F,
    kMagOffsetZMsb = 0x60,
    kGyrOffsetXLsb = 0x61,
    kGyrOffsetXMsb = 0x62,
    kGyrOffsetYLsb = 0x63,
    kGyrOffsetYMsb = 0x64,
    kGyrOffsetZLsb = 0x65,
    kGyrOffsetZMsb = 0x66,
    kAccRadiusLsb = 0x67,
    kAccRadiusMsb = 0x68,
    kMagRadiusLsb = 0x69,
    kMagRadiusMsb = 0x6A,
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
