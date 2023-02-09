#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

enum lsm6dso32_fifo_regs {
    FIFO_CTRL1 = 0x07,
    FIFO_CTRL2 = 0x08,
    FIFO_CTRL3 = 0x09,
    FIFO_CTRL4 = 0x0a,

    FIFO_STATUS1 = 0x3a,
    FIFO_STATUS2 = 0x3b
};

enum lsm6dso32_ctrl_regs {
    CTRL1_XL = 0x10,
    CTRL2_G = 0x11,
    CTRL3_C = 0x12,
    CTRL4_C = 0x13,
    CTRL5_C = 0x14,
    CTRL6_C = 0x15,
    CTRL7_G = 0x16,
    CTRL8_XL = 0x17,
    CTRL9_XL = 0x18,
    CTRL10_C = 0x19
};

enum lsm6dso32_data_regs {
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H = 0x21,

    // orientation data start (6 bytes, xl-xh, yl-yh, zl-zh)
    OUTX_L_G = 0x22,

    // acceleration data start
    OUTX_L_A = 0x28
};

enum lsm6dso32_fifo_tag {
    Gyroscope_NC = 0x01,
    Accelerometer_NC = 0x02,
    Temperature = 0x03,
    Timestamp = 0x04,
    CFG_Change = 0x05,
    Accelerometer_NC_T_2 = 0x06,
    Accelerometer_NC_T_1 = 0x07,
    Accelerometer_2xC = 0x08,
    Accelerometer_3xC = 0x09,
    Gyroscope_NC_T_2 = 0x0A,
    Gyroscope_NC_T_1 = 0x0B,
    Gyroscope_2xC = 0x0C,
    Gyroscope_3xC = 0x0D,
    Sensor_Hub_Slave_0 = 0x0E,
    Sensor_Hub_Slave_1 = 0x0F,
    Sensor_Hub_Slave_2 = 0x10,
    Sensor_Hub_Slave_3 = 0x11,
    Step_Counter = 0x12,
    Sensor_Hub_Nack = 0x19
};

struct fifo_data_t {
    uint8_t tag;
    uint8_t data[6];
};

struct raw_data_t {
    int16_t data[3];
};

class lsm6dso32 {

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

    lsm6dso32() {};
    lsm6dso32(spi_inst_t *_spi_inst, uint _cs) {
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
        if ( !read_from_device(0x0f, &buf, 1) ) { return 0; }
        if ( buf != 0x6c ) { return 0; }

        // reset device (doesnt work)
        // read_from_device(0x12, &buf, 1);
        // write_to_device(0x12, buf|1);

        // while ( 1 ) {
        //     read_from_device(0x12, &buf, 1);
        //     if (buf & 1 != 1) { break; }
        //     sleep_ms(1);
        // }

        // read ctrl3 and set bdu to 1 (might fix problems?)
        read_from_device(0x12, &buf, 1);
        write_to_device(0x12, buf | 0b00100000);

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

    /// @brief sets gyroscope ODR and range
    /// @param odr odr value
    /// @return true if write was successful 
    bool set_gyro_settings(uint8_t odr, uint8_t range) {
        return write_to_device(0x11, ((odr&0x0f)<<4) | ((range & 0x03) << 2) );
        uint8_t sts = 0;
        read_from_device(0x11, &sts, 1);
        return sts == (((odr&0x0f)<<4) | ((range & 0x03) << 2));
    }

    bool set_gyro_lpf(uint8_t lpf) {
        return write_to_device(0x15, (lpf&0b00000111) );
        uint8_t sts = 0;
        read_from_device(0x15, &sts, 1);
        return sts == (lpf&0b00000111);
        
    }

    bool set_fifo_batch_rate(uint8_t accel_batch_rate, uint8_t gyro_batch_rate) {
        return write_to_device(FIFO_CTRL3, ((gyro_batch_rate&0x0f)<<4) | (accel_batch_rate&0x0f));
        uint8_t sts = 0;
        read_from_device(FIFO_CTRL3, &sts, 1);
        return sts == (((gyro_batch_rate&0x0f)<<4) | (accel_batch_rate&0x0f));
    }

    bool set_fifo_watermark(uint8_t wtm) {
        return write_to_device(FIFO_CTRL1, wtm);
        uint8_t sts = 0;
        read_from_device(FIFO_CTRL1, &sts, 1);
        return sts == wtm;
    }

    bool read_fifo_data(fifo_data_t *buf, int nframes) {

        uint8_t rx[2];
        read_from_device(0x3a, rx, 2);
        uint16_t n_undread_fifo_frames =  ((rx[1]&3) << 8) | rx[0];
        if ( n_undread_fifo_frames > nframes ) {
            for ( int i = 0; i < nframes; i++ ) {
                read_from_device(0x78, (uint8_t*)buf, 7);
            }
        }

        return 1;
    }

    void read_accel_data(raw_data_t *ret) {
        uint8_t buf[6];

        read_from_device(0x28, buf, 6);
        ret->data[0] = buf[0] | buf[1]<<8;
        ret->data[1] = buf[2] | buf[3]<<8;
        ret->data[2] = buf[4] | buf[5]<<8;
    }

    void read_gyro_data(raw_data_t *ret) {
        uint8_t buf[6];

        read_from_device(0x22, buf, 6);
        ret->data[0] = buf[0] | buf[1]<<8;
        ret->data[1] = buf[2] | buf[3]<<8;
        ret->data[2] = buf[4] | buf[5]<<8;
    }

    void read_accel_and_gyro(raw_data_t *accel, raw_data_t *gyro) {
        
        uint8_t buf[12];

        read_from_device(0x22, buf, 12);

        gyro->data[0] = buf[0] | buf[1]<<8;
        gyro->data[1] = buf[2] | buf[3]<<8;
        gyro->data[2] = buf[4] | buf[5]<<8;

        accel->data[0] = buf[7]  | buf[6] <<8;
        accel->data[1] = buf[9]  | buf[8] <<8;
        accel->data[2] = buf[11] | buf[10]<<8;

    }

    bool dma_init_read(raw_data_t *gyro, raw_data_t *accel, uint16_t *temp) {
        
        
        uint8_t buf[14];

        // attempt to claim a channel
        int dma_chan = dma_claim_unused_channel(true);
        // if ( dma_chan == -1 ) { Serial.println("") }
        dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
        channel_config_set_dreq(&cfg, spi_get_dreq(spi0, true));
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_write_increment(&cfg, true);
        
        dma_channel_configure(	dma_chan,     			// dma channel 
                                &cfg,					// config value?
                                buf,	                // TO here
                                &spi_get_hw(spi0)->dr,	// FROM here
                                256,					// number of bytes to transfer?
                                false);					// dont immediately start

        dma_channel_start(dma_chan);

        // for debugging
        dma_channel_wait_for_finish_blocking(dma_chan);

        return 1;
    }

};