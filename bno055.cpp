#include <iostream>
#include <string.h>
#include <cstdint>
#include "bno055_i2c_lib.h"

int main() {
    Bno055I2c bno055("/dev/i2c-0");  // Open the I2C bus
    uint8_t chip_id = bno055.ReadChipId();

    printf("Chip ID: 0x%02X\n", chip_id);
    return 0;
}
