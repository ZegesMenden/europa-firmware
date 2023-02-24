#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

#pragma once

class bmp581 {

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
        uint8_t repeat_tx = 0xff;

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

    bmp581() {};
    bmp581(spi_inst_t *_spi_inst, uint _cs) {
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
        if ( !read_from_device(0x01, &buf, 1) ) { return 0; }
        if ( buf != 0x50 ) { return 0; }

        // turn on device, probably
        write_to_device(0x37, 1);
        write_to_device(0x36, 0x40);

        return 1;
    }

    /// @brief sets ODR (output data rate) of the barometer
    /// @param odr odr value
    /// @return true if write was successful 
    bool set_baro_odr(uint8_t odr) {
        write_to_device(0x37, ((odr & 0x0f) << 2) | 1);
        uint8_t sts = 0;
        read_from_device(0x37, &sts, 1);

        uint8_t osr_is_valid = 0;
        read_from_device(0x38, &osr_is_valid, 1);

        return ( sts == (((odr & 0x0f) << 2) | 1) ) & ( (osr_is_valid&0x80) == 0x80 );
    }

    /// @brief sets ODR (output data rate) of the barometer
    /// @param odr odr value
    /// @return true if write was successful 
    bool set_osr(uint8_t osr_pres, uint8_t osr_temp) {
        write_to_device(0x36, ((osr_pres & 7) << 3) | (osr_temp&7) | 0x40 );
        uint8_t sts = 0;
        read_from_device(0x36, &sts, 1);
        
        uint8_t osr_is_valid = 0;
        read_from_device(0x38, &osr_is_valid, 1);

        return ( sts == (((osr_pres & 7) << 3) | (osr_temp&7) | 0x40) ) & ( (osr_is_valid&0x80) == 0x80 );
    }

    bool data_ready() {
        uint8_t buf;
        read_from_device(0x7, &buf, 1);
        return buf&1;
    }

    void read_pres_data(uint32_t *ret) {
        uint8_t buf[3] = {0};
        read_from_device(0x20, buf, 3);
        *ret = (buf[2]<<16) | (buf[1]<<8) | buf[0];
    }

    void read_tmp_data(uint32_t *ret) {
        uint8_t buf[3] = {0};
        read_from_device(0x1d, buf, 3);
        *ret = (buf[2]<<16) | (buf[1]<<8) | buf[0];
    }

    void read_all_data(int32_t *temp, int32_t *pres) {
        uint8_t buf[6] = {0};
        read_from_device(0x1d, buf, 6);
        *temp = (buf[2]<<16) | (buf[1]<<8) | buf[0];
        *pres = (buf[5]<<16) | (buf[4]<<8) | buf[3];   
    }

};