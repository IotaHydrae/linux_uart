/* vim: set et ts=8 sw=8: */
/* uart_utils.h
 *
 * Copyright 2021 Zheng Hua, Inc.
 *
 * Linux_UART is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * Linux_UART is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along
 * with Geoclue; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Authors: Zheng Hua <writeforever@foxmail.com>
 */

#ifndef _UART_UTILS_H
#define _UART_UTILS_H


 
/**********************
 *      Defines
 **********************/
 #ifndef ENABLE_DEFAULT_SETTINGS
 #define ENABLE_DEFAULT_SETTINGS 1
 
#define DEFAULT_TTY_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD_RATE 115200
#define DEFAULT_DATA_BIT 8
#define DEFAULT_PAIRTY 'N'
#define DEFAULT_STOP_BIT 1

#define DEFAULT_MODE_1
#endif

#ifndef 
#define INIT_ON_ALLOC 1
#endif

#ifndef INIT_ON_ALLOC
#define INIT_ON_ALLOC 1
#endif

#define DEBUG

/**********************
 *      Typedefs
 **********************/
 typedef uint8_t err_t;


/**********************
 *      Struct
 **********************/
struct uart_config{
	uint8_t *tty_port;
	uint32_t baud_rate;
	uint8_t  data_bit;
	uint8_t *pairty;
	uint8_t  stop_bit;
	
};

struct my_uart_ops{
	err_t (*open_port)(uint8_t port);
	err_t (*close_port)(uint32_t fd);
	uint32_t (*read_port)(uint8_t fd, uint8_t *buf, uint32_t size);
	uint32_t (*write_port)(uint8_t fd, uint8_t *buf, uint32_t size);
	err_t (*set_port)(uint8_t fd, struct uart_config *conf);
	err_t (*set_baud_rate)(struct termios *options, const uint32_t baud_rate);
	err_t (*set_data_length)(struct termios *options, const uint8_t data_length);
};

/*struct uart_message{
	mutex uart_lock;
	uint8_t *msg_buf_read[512];
	uint8_t *msg_buf_write[512];
}*/

/**********************
 *      Prototype
 **********************/
void print_help(void);
#endif	/* End of _UART_UTILS_H */
