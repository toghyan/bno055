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


class I2COpenException : public std::exception {
private:
    std::string message;

public:
    I2COpenException(const std::string& msg) : message(msg) {}

    virtual const char* what() const noexcept override {
        return message.c_str();
    }
};

enum ReadRegisterAddress {
    kChipId = 0x00,
    kPageId = 0x07,
    kAccDataXLsb = 0x08,
    kOperationMode = 0x3D
};

enum WriteRegisterAddress {
    kPageIdWrite = 0x07,
    kOperationModeWrite = 0x3D
};

struct SensorData {
    std::vector<double> accelerometer;
    std::vector<double> magnetometer;
    std::vector<double> gyro;

    SensorData() : accelerometer(3, 0.0), magnetometer(3, 0.0), gyro(3, 0.0) {}
};

class Bno055 {
public:
    Bno055(const char *i2c_bus);
    ~Bno055();

    uint8_t ReadChipId();
    uint8_t ReadOperationMode();
    bool SetOperationMode(uint8_t mode);
    bool ReadImuData(SensorData *imu_data);
    uint8_t ReadAccelerationConfig();
    bool ChangePageId(uint8_t page_id);

private:
    int ReadRegister(uint8_t reg, uint8_t *data, size_t length);
    int WriteRegister(uint8_t reg, const uint8_t *data, size_t length);
    int16_t ConvertToTwosComplement14Bit(const uint8_t msb, const uint8_t lsb);

    int bus_fd_;
    const uint8_t device_address_ = 0x28;
    const uint8_t chip_id_ = 0xA0;
    const double accelerometer_lsb_to_ms2_ = 0.009580662;
    const double gyro_lsb_to_dps_ = 0.061045402;
};

#endif // BNO055_H
