#include <iostream>
#include <string.h>
#include <i2c.h>

int main() {

    if(i2c_open("/dev/i2c-0") == -1){
        std::cerr << "Failed!" << std::endl;
        return -1;
    }

    I2CDevice bno055;
    memset(&bno055, 0, sizeof(bno055));

    i2c_init_device(&bno055);

    bno055.bus = 0;
    bno055.addr = 0x28;
    i2c_close(0);

    return 0;
}