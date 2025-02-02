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

Bno055::~Bno055() {
    if (bus_fd_ >= 0) {
        close(bus_fd_);
    }
}

uint8_t Bno055::ReadChipId() {
    uint8_t buffer;
    if (ReadRegister(kChipId, &buffer, 1)) {
        return buffer;
    }
    return 0;
}

uint8_t Bno055::ReadOperationMode() {
    uint8_t buffer;
    if (ReadRegister(kOperationMode, &buffer, 1)) {
        return buffer;
    }
    return 0;
}

bool Bno055::SetOperationMode(uint8_t mode) {
    return WriteRegister(kOperationMode, &mode, 1);
}

bool Bno055::ReadImuData(SensorData *imu_data) {
    uint8_t buffer[18];
    if (!ReadRegister(kAccDataXLsb, buffer, sizeof(buffer))) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        imu_data->accelerometer[i] = ConvertToTwosComplement14Bit(buffer[2 * i + 1], buffer[2 * i]) * accelerometer_lsb_to_ms2_;
        int16_t raw_gyro = (static_cast<int16_t>(buffer[2 * i + 13]) << 8) | buffer[2 * i + 12];
        imu_data->gyro[i] = raw_gyro * gyro_lsb_to_dps_;
    }
    return true;
}

uint8_t Bno055::ReadAccelerationConfig() {
    uint8_t buffer;
    if (ReadRegister(0x08, &buffer, 1)) {
        return buffer;
    }
    return 0;
}

bool Bno055::ChangePageId(uint8_t page_id) {
    return WriteRegister(kPageId, &page_id, 1);
}

int Bno055::ReadRegister(uint8_t reg, uint8_t *data, size_t length) {
    if (write(bus_fd_, &reg, 1) != 1) {
        return false;
    }
    if (read(bus_fd_, data, length) != static_cast<ssize_t>(length)) {
        return false;
    }
    return true;
}

int Bno055::WriteRegister(uint8_t reg, const uint8_t *data, size_t length) {
    uint8_t buffer[length + 1];
    buffer[0] = reg;
    memcpy(&buffer[1], data, length);
    return write(bus_fd_, buffer, length + 1) == static_cast<ssize_t>(length + 1);
}

int16_t Bno055::ConvertToTwosComplement14Bit(const uint8_t msb, const uint8_t lsb) {
    uint16_t raw = (msb << 8) | lsb;
    raw &= 0x3FFF;
    return static_cast<int16_t>(raw << 2) / 4;
}
