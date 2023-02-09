#include "hardware/spi.h"
#include "hardware/gpio.h"

class lis2mdl {

private:

    spi_inst_t *inst = NULL;
    uint cs = -1;

    /// @brief writes a byte to device
    /// @param addr address to write to
    /// @param data byte to be written
    /// @return 1 if correct number of bytes were written, 0 otherwise
    int write_to_device(uint8_t addr, uint8_t data) {
        // bit 7 must be 0 because write?
        uint8_t buf[2] = {uint8_t(addr & 0x7f), data};

        gpio_put(cs, 1);
        gpio_put(cs, 0);

        int ret = spi_write_blocking(inst, buf, 2);

        gpio_put(cs, 1);

        return ret == 2;
    }

    /// @brief writes a buffer to device
    /// @param addr address to write to
    /// @param buf buffer containing write data
    /// @param nbytes number of bytes to write
    /// @return 1 if correct number of bytes were written, 0 otherwise
    int write_buf_to_device(uint8_t addr, uint8_t *buf, int nbytes) {
        // bit 7 must be 0 because write?
        uint8_t _addr = addr & 0x7f;
        
        gpio_put(cs, 1);
        gpio_put(cs, 0);
        
        int reta = spi_write_blocking(inst, &_addr, 1);
        int retb = spi_write_blocking(inst, buf, nbytes);

        gpio_put(cs, 1);

        return (reta == 1) & (retb == nbytes);
    }

    /// @brief reads bytes from device
    /// @param addr address to read from
    /// @param buf pointer to return buffer
    /// @param nbytes number of bytes to read
    /// @return 1 if correct number of bytes were written / read, 0 otherwise
    int read_from_device(uint8_t addr, uint8_t *buf, int nbytes = 1) {
        // bit 7 must be 1 because reading?
        uint8_t _addr = addr | 0x80;
        uint8_t repeat_tx = 0x00;

        gpio_put(cs, 1);
        gpio_put(cs, 0);

        // send address
        spi_write_blocking(inst, &_addr, 1);

        // write 0xff repeatedly and read data
        for ( int i = 0; i < nbytes; i++ ) {
            spi_write_read_blocking(inst, &repeat_tx, &buf[i], 1);
        }

        gpio_put(cs, 1);

        return 1;
    }

public:

    lis2mdl() {};
    lis2mdl(spi_inst_t *_spi_inst, uint _cs) {
        inst = _spi_inst;
        cs = _cs;
    }

    /// @brief initiate communication with the device
    /// @return 1 if communication was successful, 0 otherwise
    bool init() {
        // if cs and inst were not set (might be redundant)
        if ( cs == -1 || inst == NULL ) { return 0; }
        
        // read who_am_i register
        uint8_t buf = 0;
        if ( !read_from_device(0x4f, &buf, 1) ) { return 0; }
        if ( buf != 0x40 ) { return 0; }

        // disable i2c and use 4-wire SPI
        write_to_device(0x62, 0b00100100);

        // read ctrl9 and disable i3c (also might fix problems?)
        read_from_device(0x19, &buf, 1);
        write_to_device(0x19, buf | 0b10);

        return 1;
    }
    
    /// @brief sets accelerometer ODR and range
    /// @param odr odr value
    /// @return true if write was successful 
    bool set_accel_settings(uint8_t odr, uint8_t range) {
        write_to_device(0x10, ((odr & 0x0f) << 4) | ((range & 0x03) << 2));
        uint8_t sts = 0;
        read_from_device(0x10, &sts, 1);
        return sts == (((odr & 0x0f) << 4) | ((range & 0x03) << 2));
    }

    void read_accel_data() {
        uint8_t buf[6];

        read_from_device(0x28, buf, 6);
        // ret->data[0] = buf[0] | buf[1]<<8;
        // ret->data[1] = buf[2] | buf[3]<<8;
        // ret->data[2] = buf[4] | buf[5]<<8;
    }

};