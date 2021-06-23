/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  implementation of Robotis Dynamixel 2.0 protocol for controlling servos
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <termios.h>

class px4_robotis_servo {
public:
    px4_robotis_servo();

    /* Do not allow copies */
    px4_robotis_servo(const px4_robotis_servo &other) = delete;
    px4_robotis_servo &operator=(const px4_robotis_servo&) = delete;

    void update();
    void stop();

private:
    int uart;
    uint32_t baudrate;
    uint32_t us_per_byte;
    uint32_t us_gap;

    void init(void);
    void detect_servos();

    uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
    void add_stuffing(uint8_t *packet);
    void send_packet(uint8_t *txpacket);
    void read_bytes();
    void process_packet(const uint8_t *pkt, uint8_t length);
    void send_command(uint8_t id, uint16_t reg, uint32_t value, uint8_t len);
    void configure_servos(void);

    // auto-detected mask of available servos, from a broadcast ping
    uint16_t servo_mask = 0;
    uint8_t detection_count = 0;
    uint8_t configured_servos = 0;
    bool initialised = false;

    uint8_t pktbuf[64];
    uint8_t pktbuf_ofs = 0;

    // servo position limits
    int32_t pos_min = 0;
    int32_t pos_max = 0;

    // current pos of all servo
    int32_t pos = 0;
    int32_t d_pos = 10;

    hrt_abstime last_send_us = 0;
    hrt_abstime delay_time_us = 0;

    char device_name[16];
    struct termios uart_config_original;
};
