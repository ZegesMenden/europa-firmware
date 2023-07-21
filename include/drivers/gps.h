#include "core.h"
#include "hardware/i2c.h"

#pragma once

namespace gps {

    // ==================================================
    // typedefs and buffers

    // i2c address of gps
    const uint8_t gps_addr = 0x42;

	typedef struct {
		
		uint8_t sync_a;
		uint8_t sync_b;
		uint8_t cls;
		uint8_t id;
		uint16_t len;
		uint8_t* payload;
		uint8_t checksum_a;
		uint8_t checksum_b;

	} ubx_message_t;

    typedef union {
        ubx_message_t message;
        uint8_t raw[sizeof(ubx_message_t)];
    } ubx_buf_t;

	const static ubx_message_t default_ubx_message = {0xb5, 0x62};    
	
    // UBX-NAV-POSECEF
    // 0x1, 0x1, len=20
    typedef struct {
        uint32_t iTow;
        int32_t ecef_x;
        int32_t ecef_y;
        int32_t ecef_z;
        uint32_t pacc;
    } nav_posecef_t;

    // UBX-NAV-VELECEF
    // 0x1, 0x11, len=20
    typedef struct {
        uint32_t iTow;
        int32_t ecef_vx;
        int32_t ecef_vy;
        int32_t ecef_vz;
        uint32_t pacc;
    } nav_velecef_t;

    // UBX-NAV-COV
    typedef struct {

        uint32_t iTow;
        uint8_t version;
        uint8_t pos_cov_valid;
        uint8_t vel_cov_valid;
        uint8_t reserved_1[9];
        
        float pos_cov_nn;
        float pos_cov_ne;
        float pos_cov_nd;

        float pos_cov_ee;
        float pos_cov_ed;
        float pos_cov_dd;

        float vel_cov_nn;
        float vel_cov_ne;
        float vel_cov_nd;

        float vel_cov_ee;
        float vel_cov_ed;
        float vel_cov_dd;

    } nav_cov_t;

    // UBX-NAV-PVT
    // 0x1, 0x7
    typedef struct {

        uint32_t itow;
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t min;
        uint8_t sec;
        uint8_t valid;
        uint32_t time_accuracy;
        int32_t nanosecond;

        uint8_t fix_type;
        uint8_t flags_1;
        uint8_t flags_2;
        uint8_t n_sat;

        int32_t lon;
        int32_t lat;
        int32_t h_ellipsoid;
        int32_t h_msl;

        int32_t h_acc;
        int32_t v_acc;

        int32_t NED_v_n;
        int32_t NED_v_e;
        int32_t NED_v_d;

        int32_t groung_speed;
        int32_t speed_heading;

        uint32_t speed_acc;
        uint32_t heading_acc;
        uint16_t pdop;
        uint16_t flags_3;

        uint8_t reserved[4];

        int32_t vehicle_heading;

        int16_t mag_dec;
        int16_t mag_acc;

    } nav_pvt_t;

	uint8_t i2c_rx_buf[1024];
	uint16_t i2c_rx_buf_pos = 0;

	uint8_t message_tx_buf[1024];
    uint8_t payload_tx_buf[256];
    uint8_t payload_rx_buf[256];

    nav_pvt_t gps_pvt_raw;

    // ==================================================
    // message processing functons

    void message_to_buffer(ubx_message_t *msg, uint8_t *buf) {

        memcpy(buf+6, msg->payload, msg->len);

		uint8_t len_msb = msg->len>>8;
		uint8_t len_lsb = msg->len&0xff;

		buf[0] = msg->sync_a;
		buf[1] = msg->sync_b;
		buf[2] = msg->cls;
		buf[3] = msg->id;
		buf[4] = len_lsb;
		buf[5] = len_msb;
		buf[6+msg->len] = msg->checksum_a;
		buf[6+msg->len+1] = msg->checksum_b;

    }

	void calculate_checksum(uint8_t *msgbuf) {

        uint8_t checksum_a = 0;
        uint8_t checksum_b = 0;

        uint16_t msglen = ((msgbuf[5]<<8) | msgbuf[4]) + 6;

		for ( int i = 2; i < msglen; i++ ) {
			checksum_a += msgbuf[i];
			checksum_b += checksum_a;
		}

        msgbuf[msglen] = checksum_a;
        msgbuf[msglen+1] = checksum_b;

	}

	bool valid_checksum(uint8_t *msgbuf) {

		uint8_t checksum_a = 0;
        uint8_t checksum_b = 0;

        uint16_t msglen = ((msgbuf[5]<<8) | msgbuf[4]) + 6;

		for ( int i = 2; i < msglen; i++ ) {
			checksum_a += msgbuf[i];
			checksum_b += checksum_a;
		}

		return (checksum_a==msgbuf[msglen])&(checksum_b==msgbuf[msglen+1]);

	}

	bool packet_is_complete(uint8_t *packet_start, uint16_t packet_len, bool *size_exceeds_packet) {
		
		uint16_t msg_len = (packet_start[5]<<8) | packet_start[4];
	
		// add the bytes for sync, class, id, length, and checksum
		msg_len += 8;

		if ( msg_len > packet_len && size_exceeds_packet != NULL ) { *size_exceeds_packet = true; }
		if ( msg_len <= packet_len ) { return true; }
		return false; 

	}

    bool process_message(ubx_message_t *msg, uint8_t *buf) {

        // check frame starts and checksum
        if ( (buf[0] != 0xb5) || (buf[1] != 0x62) ) { return false; }
        if (!valid_checksum(buf)) { return false; }

        msg->checksum_a = buf[0];
        msg->checksum_b = buf[1];

        msg->cls = buf[2];
        msg->id = buf[3];

        msg->len = (uint16_t)(buf[5]<<8) | buf[4];
        
        memcpy(payload_rx_buf, buf+6, msg->len);
        msg->payload = payload_rx_buf;

        msg->checksum_a = buf[msg->len+6];
        msg->checksum_a = buf[msg->len+7];

        return true;

    }

    uint32_t buf_to_u32(uint8_t *buf) {
        return ( buf[3]<<24 ) | ( buf[2]<<16 ) | ( buf[1]<<8 ) | buf[0];
    }
    
    int32_t buf_to_i32(uint8_t *buf) {
        return (int32_t)(( (int32_t)buf[3]<<24 ) | ( (int32_t)buf[2]<<16 ) | ( (int32_t)buf[1]<<8 ) | (int32_t)buf[0]);
    }

    uint16_t buf_to_u16(uint8_t *buf) {
        return ( buf[1]<<8 ) | buf[0];
    }

    int16_t buf_to_i16(uint8_t *buf) {
        return (int16_t)(( (int16_t)buf[1]<<8 ) | (int16_t)buf[0]);
    }
    
    float buf_to_float(uint8_t *buf) {
        return *(float*)((uint32_t*)(buf_to_u32(buf)));
    }

    void process_nav_pvt(nav_pvt_t *out, uint8_t *buf) {
        
        uint8_t *buf_pos = buf;

        out->itow          = buf_to_u32(buf_pos); buf_pos+=4;
        out->year          = buf_to_u16(buf_pos); buf_pos+=4;
        out->month         = *(buf_pos); buf_pos+=1;
        out->day           = *(buf_pos); buf_pos+=1;
        out->hour          = *(buf_pos); buf_pos+=1;
        out->min           = *(buf_pos); buf_pos+=1;
        out->sec           = *(buf_pos); buf_pos+=1;
        out->valid         = *(buf_pos); buf_pos+=1;
        out->time_accuracy = buf_to_u32(buf_pos); buf_pos+=4;
        out->nanosecond    = buf_to_i32(buf_pos); buf_pos+=4;

        out->fix_type   = *buf_pos++;
        out->flags_1    = *buf_pos++;
        out->flags_2    = *buf_pos++;
        out->n_sat      = *buf_pos++;

        out->lon         = buf_to_i32(buf_pos); buf_pos+=4;
        out->lat         = buf_to_i32(buf_pos); buf_pos+=4;
        out->h_ellipsoid = buf_to_i32(buf_pos); buf_pos+=4;
        out->h_msl       = buf_to_i32(buf_pos); buf_pos+=4;

        out->h_acc          = buf_to_i32(buf_pos); buf_pos+=4;
        out->v_acc          = buf_to_i32(buf_pos); buf_pos+=4;

        out->NED_v_n        = buf_to_i32(buf_pos); buf_pos+=4;
        out->NED_v_e        = buf_to_i32(buf_pos); buf_pos+=4;
        out->NED_v_d        = buf_to_i32(buf_pos); buf_pos+=4;

        out->groung_speed   = buf_to_i32(buf_pos); buf_pos+=4;
        out->speed_heading  = buf_to_i32(buf_pos); buf_pos+=4;

        out->speed_acc    = buf_to_i32(buf_pos); buf_pos+=4;
        out->heading_acc  = buf_to_i32(buf_pos); buf_pos+=4;
        out->pdop         = buf_to_i16(buf_pos); buf_pos+=2;
        out->flags_3      = buf_to_i16(buf_pos); buf_pos+=2;

        buf_pos += 4;

        out->vehicle_heading  = buf_to_i32(buf_pos); buf_pos+=4;

        out->mag_dec = buf_to_i16(buf_pos); buf_pos+=2;
        out->mag_acc = buf_to_i16(buf_pos); buf_pos+=2;

    }

    // ==================================================
    // hardware interface functions

    uint16_t get_available_bytes() {

		uint8_t val = 0xfd;
		uint8_t rx_buf[2] = {0, 0};

		i2c_write_blocking(i2c0, gps_addr, &val, 1, true);
		i2c_read_blocking(i2c0, gps_addr, rx_buf, 2, false);

		return ((rx_buf[0]&0x7f)<<8)|rx_buf[1];
		
	}	

	void send_msg(ubx_message_t *msg) {

        message_to_buffer(msg, message_tx_buf);
        calculate_checksum(message_tx_buf);

        uint16_t msglen = ((message_tx_buf[5]<<8) | message_tx_buf[4]) + 8;

        i2c_write_blocking(i2c0, gps_addr, message_tx_buf, msglen, false);
		
	}

    // ==================================================
    // init and update

	bool init() {

		gpio_set_function(13, GPIO_FUNC_I2C);
		gpio_set_function(12, GPIO_FUNC_I2C);
		
		i2c_init(i2c0, 400000);

        i2c_set_baudrate(i2c0, 400000);

		uint8_t val = 0xfd;
		uint8_t rx_buf[2];

		// perform a dummy read of the number of bytes available to see if the device is there

        debugprintf_gps("finding device...");

		int err = i2c_write_blocking(i2c0, gps_addr, &val, 1, true);
		if ( err != 1 ) { debugprintf_gps("[FAILED]\n"); return false; }
		err = i2c_read_blocking(i2c0, gps_addr, rx_buf, 2, false);
		if ( err != 2 ) { debugprintf_gps("[FAILED]\n"); return false; }

        debugprintf_gps("[OK]\n");

		ubx_message_t cfg_msg;

        // ==========================================
        // configure protocols

        debugprintf_gps("configuring port protocol...");

        cfg_msg.sync_a = 0xb5;
        cfg_msg.sync_b = 0x62;
        
		cfg_msg.cls = 0x06;
		cfg_msg.id = 0x00;
		cfg_msg.len = 20;
		cfg_msg.payload = payload_tx_buf;
		
		// port ID (0 for i2c)
		payload_tx_buf[0] = 0;

        // reserved
		payload_tx_buf[1] = 0;

		// tx ready pin (interrupt pin? idrk what this does so il set it to zero)
		payload_tx_buf[3] = 0;
		payload_tx_buf[2] = 0;

		//  set i2c addr
		payload_tx_buf[7] = 0;
		payload_tx_buf[6] = 0;
		payload_tx_buf[5] = 0;
		payload_tx_buf[4] = gps_addr<<1;
		
        // set protocols used over i2c
        payload_tx_buf[13] = 0;
        payload_tx_buf[12] = 1;

        payload_tx_buf[15] = 0;
        payload_tx_buf[14] = 1;

        // tx pin timeout flag
        payload_tx_buf[17] = 0;
        payload_tx_buf[16] = 0;

        // reserved
        payload_tx_buf[19] = 0;
        payload_tx_buf[18] = 0;

		send_msg(&cfg_msg);

		sleep_ms(100);

        uint16_t available_bytes = get_available_bytes();
        i2c_read_blocking(i2c0, gps_addr, i2c_rx_buf, available_bytes, false);

        if ( process_message(&cfg_msg, i2c_rx_buf) ) {
            if ( cfg_msg.cls == 5 && cfg_msg.id == 1) { debugprintf_gps("[OK]\n"); }
            else { debugprintf_gps("[FAILED]\n"); return false;}
        } else { debugprintf_gps("[FAILED]\n");  return false;}

        // ==========================================
        // message rate cfg

        debugprintf_gps("configuring nav update rate...");
        
        cfg_msg.sync_a = 0xb5;
        cfg_msg.sync_b = 0x62;
        
		cfg_msg.cls = 0x06;
		cfg_msg.id = 0x08;
		cfg_msg.len = 6;
		cfg_msg.payload = payload_tx_buf;
		
		// set measure rate to 20Hz
		payload_tx_buf[1] = 0;
		payload_tx_buf[0] = 50;

		// set nav rate to max
		payload_tx_buf[3] = 0;
		payload_tx_buf[2] = 1;

		// set time alignment (idrk what this is)
		payload_tx_buf[5] = 0;
		payload_tx_buf[4] = 0;

		send_msg(&cfg_msg);

        sleep_ms(100);

        available_bytes = get_available_bytes();
        i2c_read_blocking(i2c0, gps_addr, i2c_rx_buf, available_bytes, false);

        if ( process_message(&cfg_msg, i2c_rx_buf) ) {
            if ( cfg_msg.cls == 5 && cfg_msg.id == 1) { debugprintf_gps("[OK]\n"); }
            else { debugprintf_gps("[FAILED]\n"); return false;}
        } else { debugprintf_gps("[FAILED]\n");  return false;}

        // ==========================================
        // poll rate cfg

        debugprintf_gps("configuring nav poll rate...");

        cfg_msg.sync_a = 0xb5;
        cfg_msg.sync_b = 0x62;
        
		cfg_msg.cls = 0x06;
		cfg_msg.id = 0x01;
		cfg_msg.len = 3;
		cfg_msg.payload = payload_tx_buf;
		
        // target message class and id
        payload_tx_buf[0] = 0x01;
		payload_tx_buf[1] = 0x07;

		// set rate to max
		payload_tx_buf[2] = 1;

		send_msg(&cfg_msg);

        sleep_ms(100);
        available_bytes = get_available_bytes();
        i2c_read_blocking(i2c0, gps_addr, i2c_rx_buf, available_bytes, false);

        if ( process_message(&cfg_msg, i2c_rx_buf) ) {
            if ( cfg_msg.cls == 5 && cfg_msg.id == 1) { debugprintf_gps("[OK]\n"); }
            else { debugprintf_gps("[FAILED]\n"); return false;}
        } else { debugprintf_gps("[FAILED]\n");  return false;}

        i2c_rx_buf_pos = 0;
        
        // ==========================================
		i2c_set_baudrate(i2c0, 1000000);
		return true;
	}

	void update() {	
        i2c_set_baudrate(i2c0, 400000);
		uint16_t available_bytes = get_available_bytes();
        if ( available_bytes == 0 ) { return; }
        debugprintf_gps("%i\n", available_bytes);
        // limit available bytes to remaining buffer size
        if ( available_bytes + i2c_rx_buf_pos > 1024) { available_bytes = 1024 - i2c_rx_buf_pos; }

        // read all available bytes
        i2c_read_blocking(i2c0, gps_addr, &i2c_rx_buf[i2c_rx_buf_pos], available_bytes, false);
        i2c_rx_buf_pos += available_bytes;

        bool multiple_packets = false;

        if ( packet_is_complete(i2c_rx_buf, i2c_rx_buf_pos, &multiple_packets) ) {
            
            ubx_message_t msg;
            if (!process_message(&msg, i2c_rx_buf)) {
                debugprintf_gps("couldnt process message\n");
                return;
            } else {

                debugprintf_gps("message class: %i\n", msg.cls);
                debugprintf_gps("message id: %i\n", msg.id);

                debugprintf_gps("payload 0: %i\n", msg.payload[0]);
                debugprintf_gps("payload 1: %i\n", msg.payload[1]);
                
                // if this message is a PVT fix
                if ( msg.cls==0x01 && msg.id==0x07 ) {
                    process_nav_pvt(&gps_pvt_raw, msg.payload);
                    flags::nav_flags::gps_drdy = true;
                }

                memcpy(i2c_rx_buf, i2c_rx_buf+msg.len+8, msg.len+8);
                i2c_rx_buf_pos -= msg.len+8;

            }

        } else {
            debugprintf_gps("invalid packet\n");
        }

        if ( multiple_packets ) { debugprintf_gps("multiple packets found\n"); }

        i2c_set_baudrate(i2c0, 1000000);
	}
    
};