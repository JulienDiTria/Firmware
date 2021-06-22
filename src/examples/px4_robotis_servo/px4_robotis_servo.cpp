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

  Portions of this code are based on the dynamixel_sdk code:
  https://github.com/ROBOTIS-GIT/DynamixelSDK
  which is under the following license:

* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

// px4_config and posix are needed for just about every PX4 user application.
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#include <drivers/drv_hrt.h>
#include <termios.h>

#include "px4_robotis_servo.hpp"

#define BROADCAST_ID 0xFE
#define MAX_ID 0xFC

// DXL protocol common commands
#define INST_PING          1
#define INST_READ          2
#define INST_WRITE         3
#define INST_REG_WRITE     4
#define INST_ACTION        5
#define INST_FACTORY_RESET 6
#define INST_CLEAR        16
#define INST_SYNC_WRITE  131
#define INST_BULK_READ   146

// 2.0 protocol commands
#define INST_REBOOT       8
#define INST_STATUS      85
#define INST_SYNC_READ  130
#define INST_BULK_WRITE 147

// 2.0 protocol packet offsets
#define PKT_HEADER0     0
#define PKT_HEADER1     1
#define PKT_HEADER2     2
#define PKT_RESERVED    3
#define PKT_ID          4
#define PKT_LENGTH_L    5
#define PKT_LENGTH_H    6
#define PKT_INSTRUCTION 7
#define PKT_ERROR       8
#define PKT_PARAMETER0  8

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

// register offsets
#define REG_OPERATING_MODE 11
#define   OPMODE_CURR_CONTROL    0
#define   OPMODE_VEL_CONTROL     1
#define   OPMODE_POS_CONTROL     3
#define   OPMODE_EXT_POS_CONTROL 4

#define REG_TORQUE_ENABLE  64

#define REG_STATUS_RETURN  68
#define   STATUS_RETURN_NONE 0
#define   STATUS_RETURN_READ 1
#define   STATUS_RETURN_ALL  2

#define REG_GOAL_POSITION 116

// how many times to send servo configure msgs
#define CONFIGURE_SERVO_COUNT 4

// how many times to send servo detection
#define DETECT_SERVO_COUNT 4

/*
const AP_Param::GroupInfo px4_robotis_servo::var_info[] = {

    // @Param: POSMIN
    // @DisplayName: Robotis servo position min
    // @Description: Position minimum at servo min value. This should be within the position control range of the servos, normally 0 to 4095
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("POSMIN",  1, px4_robotis_servo, pos_min, 0),

    // @Param: POSMAX
    // @DisplayName: Robotis servo position max
    // @Description: Position maximum at servo max value. This should be within the position control range of the servos, normally 0 to 4095
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("POSMAX",  2, px4_robotis_servo, pos_max, 4095),

    AP_GROUPEND
};
*/

// Export main function so that we can call on it in PX4
extern "C" __EXPORT int px4_robotis_servo_main(int argc, char *argv[]);

static int robotis_open_uart(const char *uart_name, struct termios *uart_config, struct termios *uart_config_original);
static void set_uart_single_wire(int uart, bool single_wire);

/*
Very useful introduction
https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
*/
static int robotis_open_uart(const char *uart_name, struct termios *uart_config, struct termios *uart_config_original)
{
	/* Open UART */
	const int uart = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (uart < 0) {
		PX4_ERR("Error opening port: %s (%i)", uart_name, errno);
		return -1;
	}

	/* Back up the original UART configuration to restore it after exit */
	int termios_state;

	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		PX4_ERR("tcgetattr %s: %d\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(uart, uart_config);

	/* Disable output post-processing */
	uart_config->c_oflag &= ~OPOST;

	uart_config->c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	uart_config->c_cflag &= ~CSIZE;
	uart_config->c_cflag |= CS8;         /* 8-bit characters */
	uart_config->c_cflag &= ~PARENB;     /* no parity bit */
	uart_config->c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	uart_config->c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	uart_config->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	uart_config->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	/* Set baud rate */
	// todo : dynamixel works on 9600 57600(default) 115200 1M 2M 3M 4M 4.5M
	const speed_t uart_speed = B57600;

	if (cfsetispeed(uart_config, uart_speed) < 0 || cfsetospeed(uart_config, uart_speed) < 0) {
		PX4_ERR("%s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, uart_config)) < 0) {
		PX4_ERR("%s (tcsetattr)\n", uart_name);
		close(uart);
		return -1;
	}

	return uart;
}

static void set_uart_single_wire(int uart, bool single_wire)
{
	if (ioctl(uart, TIOCSSINGLEWIRE, single_wire ? (SER_SINGLEWIRE_ENABLED | SER_SINGLEWIRE_PUSHPULL |
			SER_SINGLEWIRE_PULLDOWN) : 0) < 0) {
		PX4_WARN("setting TIOCSSINGLEWIRE failed");
	}
}

// constructor
px4_robotis_servo::px4_robotis_servo(void)
{
    // set defaults from the parameter table
    // AP_Param::setup_object_defaults(this, var_info);
    init();
}

void px4_robotis_servo::init(void)
{
    PX4_INFO("px4_robotis_servo::init on ttyS2");
	const char* device_name = "/dev/ttyS2"; /* default USART3	/dev/ttyS2	TELEM2 */

	/* Open UART */
	struct termios uart_config_original;
	struct termios uart_config;
	uart = robotis_open_uart(device_name, &uart_config, &uart_config_original);

	if (uart < 0) {
		device_name = NULL;
		return;
	}
	set_uart_single_wire(uart, true);

	if (uart) {
		baudrate = B57600;
		us_per_byte = 10 * 1e6 / baudrate;
		us_gap = 4 * 1e6 / baudrate;
	}
    PX4_INFO("px4_robotis_servo::init done");
}

/*
  calculate Robotis protocol CRC
 */
uint16_t px4_robotis_servo::update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t i;
    static const uint16_t crc_table[256] = {0x0000,
                                            0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                                            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
                                            0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
                                            0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
                                            0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
                                            0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
                                            0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
                                            0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
                                            0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                                            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
                                            0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
                                            0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
                                            0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
                                            0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
                                            0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
                                            0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
                                            0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                                            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
                                            0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
                                            0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
                                            0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
                                            0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
                                            0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
                                            0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
                                            0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                                            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
                                            0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
                                            0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
                                            0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
                                            0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
                                            0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
                                            0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
                                            0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                                            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
                                            0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
                                            0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
                                            0x820D, 0x8207, 0x0202 };

    for (uint16_t j = 0; j < data_blk_size; j++) {
        i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

/*
  addStuffing() from Robotis SDK. This pads the packet as required by the protocol
*/
void px4_robotis_servo::add_stuffing(uint8_t *packet)
{
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;

    if (packet_length_in < 8) {
        // INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
        return;
    }

    uint8_t *packet_ptr;
    uint16_t packet_length_before_crc = packet_length_in - 2;
    for (uint16_t i = 3; i < packet_length_before_crc; i++) {
        packet_ptr = &packet[i+PKT_INSTRUCTION-2];
        if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD) {
            packet_length_out++;
        }
    }

    if (packet_length_in == packet_length_out) {
        // no stuffing required
        return;
    }

    uint16_t out_index  = packet_length_out + 6 - 2;  // last index before crc
    uint16_t in_index   = packet_length_in + 6 - 2;   // last index before crc

    while (out_index != in_index) {
        if (packet[in_index] == 0xFD && packet[in_index-1] == 0xFF && packet[in_index-2] == 0xFF) {
            packet[out_index--] = 0xFD; // byte stuffing
            if (out_index != in_index) {
                packet[out_index--] = packet[in_index--]; // FD
                packet[out_index--] = packet[in_index--]; // FF
                packet[out_index--] = packet[in_index--]; // FF
            }
        } else {
            packet[out_index--] = packet[in_index--];
        }
    }

    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}

/*
  send a protocol 2.0 packet
 */
void px4_robotis_servo::send_packet(uint8_t *txpacket)
{
    add_stuffing(txpacket);

    // check max packet length
    uint16_t total_packet_length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7;

    // make packet header
    txpacket[PKT_HEADER0]   = 0xFF;
    txpacket[PKT_HEADER1]   = 0xFF;
    txpacket[PKT_HEADER2]   = 0xFD;
    txpacket[PKT_RESERVED]  = 0x00;

    // add CRC16
    uint16_t crc = update_crc(0, txpacket, total_packet_length - 2);    // 2: CRC16
    txpacket[total_packet_length - 2] = DXL_LOBYTE(crc);
    txpacket[total_packet_length - 1] = DXL_HIBYTE(crc);

    write(uart, txpacket, total_packet_length);

    delay_time_us += total_packet_length * us_per_byte + us_gap;
}

/*
  use a broadcast ping to find attached servos
 */
void px4_robotis_servo::detect_servos(void)
{
    uint8_t txpacket[10] {};

    txpacket[PKT_ID] = BROADCAST_ID;
    txpacket[PKT_LENGTH_L] = 3;
    txpacket[PKT_LENGTH_H] = 0;
    txpacket[PKT_INSTRUCTION] = INST_PING;

    send_packet(txpacket);

    // give plenty of time for replies from all servos
    last_send_us = hrt_absolute_time();
    delay_time_us += 1000 * us_per_byte;
}

/*
  broadcast configure all servos
 */
void px4_robotis_servo::configure_servos(void)
{
	// disable torque control
	send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 0, 1);

	// disable replies unless we read
	send_command(BROADCAST_ID, REG_STATUS_RETURN, STATUS_RETURN_READ, 1);

	// use position control mode
	send_command(BROADCAST_ID, REG_OPERATING_MODE, OPMODE_POS_CONTROL, 1);

	// enable torque control
	send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 1, 1);
}


/*
  send a command to a single servo, changing a register value
 */
void px4_robotis_servo::send_command(uint8_t id, uint16_t reg, uint32_t value, uint8_t len)
{
    uint8_t txpacket[16] {};

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH_L] = 5 + len;
    txpacket[PKT_LENGTH_H] = 0;
    txpacket[PKT_INSTRUCTION] = INST_WRITE;
    txpacket[PKT_INSTRUCTION+1] = DXL_LOBYTE(reg);
    txpacket[PKT_INSTRUCTION+2] = DXL_HIBYTE(reg);
    memcpy(&txpacket[PKT_INSTRUCTION+3], &value, MIN(len,4));

    send_packet(txpacket);
}

/*
  read response bytes
 */
void px4_robotis_servo::read_bytes(void)
{
    uint8_t buf = 0;

    uint32_t n = read(uart, &buf, sizeof(&buf));
    if (n == 0 && pktbuf_ofs < PKT_INSTRUCTION) {
        return;
    }

    if (n > sizeof(pktbuf) - pktbuf_ofs) {
        n = sizeof(pktbuf) - pktbuf_ofs;
    }
    for (uint8_t i=0; i<n; i++) {
        pktbuf[pktbuf_ofs++] = buf;
    }

    // discard bad leading data. This should be rare
    while (pktbuf_ofs >= 4 &&
           (pktbuf[0] != 0xFF || pktbuf[1] != 0xFF || pktbuf[2] != 0xFD || pktbuf[3] != 0x00)) {
        memmove(pktbuf, &pktbuf[1], pktbuf_ofs-1);
        pktbuf_ofs--;
    }

    if (pktbuf_ofs < 10) {
        // not enough data yet
        return;
    }

    const uint16_t total_packet_length = DXL_MAKEWORD(pktbuf[PKT_LENGTH_L], pktbuf[PKT_LENGTH_H]) + PKT_INSTRUCTION;
    if (total_packet_length > sizeof(pktbuf)) {
        pktbuf_ofs = 0;
        return;
    }
    if (pktbuf_ofs < total_packet_length) {
        // more data needed
        return;
    }

    // check CRC
    const uint16_t crc = DXL_MAKEWORD(pktbuf[total_packet_length-2], pktbuf[total_packet_length-1]);
    const uint16_t calc_crc = update_crc(0, pktbuf, total_packet_length - 2);
    if (calc_crc != crc) {
        memmove(pktbuf, &pktbuf[total_packet_length], pktbuf_ofs - total_packet_length);
        pktbuf_ofs -= total_packet_length;
        return;
    }

    // process full packet
    process_packet(pktbuf, total_packet_length);

    memmove(pktbuf, &pktbuf[total_packet_length], pktbuf_ofs - total_packet_length);
    pktbuf_ofs -= total_packet_length;
}

/*
  process a packet from a servo
 */
void px4_robotis_servo::process_packet(const uint8_t *pkt, uint8_t length)
{
    uint8_t id = pkt[PKT_ID];
    if (id > 16 || id < 1) {
        // discard packets from servos beyond max or min. Note that we
        // don't allow servo 0, to make mapping to SERVOn_* parameters
        // easier
        return;
    }
    uint16_t id_mask = (1U<<(id-1));
    if (!(id_mask & servo_mask)) {
        // mark the servo as present
        servo_mask |= id_mask;
	PX4_INFO("Robotis: new servo %u\n", id);
    }
}


void px4_robotis_servo::update()
{
    if (!initialised) {
        initialised = true;
        init();
        last_send_us = hrt_absolute_time();
        return;
    }

    if (uart < 0) {
        return;
    }

    read_bytes();

    const hrt_abstime now = hrt_absolute_time();
    if (last_send_us != 0 && now - last_send_us < delay_time_us) {
        // waiting for last send to complete
        return;
    }

    if (detection_count < DETECT_SERVO_COUNT) {
        detection_count++;
        detect_servos();
    }

    if (servo_mask == 0) {
        return;
    }

    if (configured_servos < CONFIGURE_SERVO_COUNT) {
        configured_servos++;
        last_send_us = now;
        configure_servos();
        return;
    }

    last_send_us = now;
    delay_time_us = 0;

    pos += d_pos;

    if(pos>pos_max){
	    d_pos = -10;
	    pos = pos_max;
    }
    if(pos < pos_min){
	    d_pos = 10;
	    pos = pos_min;
    }

    // loop for all 16 channels
    //for (uint8_t i=0; i<PWM_OUTPUT_MAX_CHANNELS; i++) {
    for (uint8_t i=0; i<16; i++) {
        if (((1U<<i) & servo_mask) == 0) {
            continue;
        }
        // todo : get radio channel in
	    // for now use one turn looping on pos
        send_command(i+1, REG_GOAL_POSITION, pos, 4);
    }
}


int px4_robotis_servo_main(int argc, char *argv[]){
	PX4_INFO("px4_robotis_servo_main starting");
	px4_robotis_servo servo;

	while(true){
		servo.update();
	}
}
