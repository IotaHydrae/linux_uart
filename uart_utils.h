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

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdint.h>  /* Standard typedefs */
#include <stdlib.h>  /* Standard library */
#include <time.h>
#include <sys/ioctl.h>
 
/**********************
 *      Defines
 **********************/
#define DEFAULT_TTY_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD_RATE 115200
#define DEFAULT_DATA_BIT 8
#define DEFAULT_PAIRTY 'N'
#define DEFAULT_STOP_BIT 1

#define DEBUG

//#define NODELAY_MODE

#ifdef DEBUG
#define debug_info(msg) fprintf(stderr, "%s: %s\n", __func__, msg)
#else
#define debug_info(msg)
#endif

#define BAUD(b) (B##b)

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

/*struct uart_message{
	mutex uart_lock;
	uint8_t *msg_buf_read[512];
	uint8_t *msg_buf_write[512];
}*/

/**********************
 *      Typedefs
 **********************/
 
/**********************
 *      Prototype
 **********************/
void print_help(void);
uint8_t open_port(char *port);
uint8_t set_port(int fd, uint32_t baud_rate, uint8_t data_bits, uint8_t parity_checking, uint8_t stop_bit);

uint8_t set_baud_rate(struct termios *options, const uint32_t baud_rate);
uint8_t set_data_length(struct termios *options, uint8_t length);
uint8_t set_parity(struct termios *options, uint8_t enable);
uint8_t set_stop_bit(struct termios *options, uint8_t length);
uint8_t set_hardware_flow_control(struct termios *options, uint8_t enable);
#endif	/* End of _UART_UTILS_H */
