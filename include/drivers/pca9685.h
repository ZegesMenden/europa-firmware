#include "hardware/i2c.h"
#include "hardware/gpio.h"

class pca9685 {

private:

    uint8_t addr;
    i2c_inst_t *inst;

    // internal default frequency, allegedly
    uint32_t osc_freq = 25000000;

    int write_to_device(uint8_t addr, uint8_t data) {
        uint8_t buf[2] = { addr, data };
        int ret = i2c_write_blocking(inst, addr, buf, 2, false);
        return ret == 2;
    }

    int write_buf_to_device(uint8_t addr, uint8_t *data, int nbytes) {
        int reta = i2c_write_blocking(inst, addr, &addr, 1, false);
        int retb = i2c_write_blocking(inst, addr, data, nbytes, false);
        return (reta == 1)&(retb==nbytes);
    }



public:

    pca9685() {}

    bool init() {

        // restart device
        write_to_device(0x0, 0x80);
    }

    bool set_freq(int freq) {

    }

};