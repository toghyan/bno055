#include <iostream>
#include <string.h>
#include <i2c.h>

int main() {
    int bus_fd = i2c_open("/dev/i2c-0");  // Open the I2C bus
    if (bus_fd == -1) {
        std::cerr << "Failed to open I2C bus!" << std::endl;
        return -1;
    }

    I2CDevice bno055;
    memset(&bno055, 0, sizeof(bno055));
    i2c_init_device(&bno055);

    bno055.bus = bus_fd;
    bno055.addr = 0x28;   // Device address

    unsigned int register_address = 0x00;  // Chip ID register address.
    unsigned char buffer[1];               // Buffer to store the read data.
    size_t length_to_read = sizeof(buffer);

    ssize_t bytes_read = i2c_read(&bno055, register_address, buffer, length_to_read);
    if (bytes_read < 0) {
        std::cerr << "Failed to read from the I2C device" << std::endl;
        i2c_close(bus_fd);  // Close the bus using the file descriptor
        return -1;
    }

    printf("Chip ID: 0x%02X\n", buffer[0]);

    i2c_close(bus_fd);
    return 0;
}
