/* vim: set et ts=8 sw=8: */
/* serial_rw.c
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

/*
 * Default define values.
 */
#define DEFAULT_TTY_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD_RATE 115200
#define DEFAULT_DATA_BIT 8
#define DEFAULT_PAIRTY 'N'
#define DEFAULT_STOP_BIT 1

#define DEBUG

#ifdef DEBUG
#define debug_info(msg) fprintf(stderr, "%s: %s\n", __func__, msg)
#else
#define debug_info(msg)
#endif

#define BAUD(b) (B##b)

/*
 * `check_if_supported` - set the baud rate
 * @src
 * @target
 * Return 0 on success or -1 on error.
 */
static inline uint8_t check_if_supported(const uint32_t *src, uint32_t target)
{
    while (src != NULL)
    {
        if (*src == target)
            return 0;
        else
            src++;
    }
    return -1;
}

/*
 * `set_baud_rate` - set the baud rate
 * @options
 * @baud_rate
 * Return 0 on success or -1 on error.
 */
uint8_t set_baud_rate(struct termios *options, const uint32_t baud_rate)
{
    int ret;

    const uint32_t supported_baud_rate_list[] = {
        9600,
        19200,
        38400,
        57600,
        115200,
    };

    /*do Fast check*/
    ret = check_if_supported(supported_baud_rate_list, baud_rate);
    if (ret < 0)
    {
        return -1;
    }
    debug_info("Have match the baud rate. Setting it...");
    /* Setting the Baud Rate. */
    switch (baud_rate)
    {
    default:
        cfsetspeed(options, BAUD(115200));
        break;
    case 9600:
        cfsetspeed(options, BAUD(9600));
        break;
    case 19200:
        cfsetspeed(options, BAUD(19200));
        break;
    case 38400:
        cfsetspeed(options, BAUD(38400));
        break;
    case 57600:
        cfsetspeed(options, BAUD(57600));
        break;
    case 115200:
        cfsetspeed(options, BAUD(115200));
        break;
    }

    return 0;
}

/*
 * `set_data_length` - set the baud rate
 *
 * Return 0 on success or -1 on error.
 */
uint8_t set_data_length(struct termios *options, uint8_t length)
{
    int ret;
    uint8_t supported_data_bit_length[] = {
        7,
        8,
    };
    ret = check_if_supported(supported_data_bit_length, length);
    if (ret < 0)
    {
        return -1;
    }
    debug_info("Have match the data bit length, Setting it...");

    switch (length)
    {
    default:
        options.c_cflag |= CS8; /* 8 data bits */
        break;
    case 7:
        options.c_cflag |= CS7; /* 7 data bits */
        break;
    case 8:
        options.c_cflag |= CS8; /* 8 data bits */
        break;
    }

    return 0;
}

/*
 * `set_parity` - set the parity
 *
 * Return 0 on success or -1 on error.
 */
uint8_t set_parity(struct termios *options, bool enable)
{
    if (enable)
    {
        options.c_cflag |= PARENB; /* Enable parity bit */
    }
    else if (!enable)
    {
        options.c_cflag &= ~PARENB; /* Disable parity bit */
    }
    else
    {
        /* Default for disable. */
        options.c_cflag &= ~PARENB; /* Disable parity bit */
    }

    return 0;
}

/*
 * `set_stop_bit` - set the baud rate
 *
 * Return 0 on success or -1 on error.
 */
uint8_t set_stop_bit(struct termios *options, uint8_t length)
{
    if (length == 1)
    {
        options.c_cflag &= ~CSTOPB; /* Setting stop bits to 1 */
    }
    else if (length == 2)
    {
        options.c_cflag |= CSTOPB; /* Setting stop bits to 1 */
    }
    else
    {
    }

    return 0;
}

/*
 * `set_hardware_flow_control` - set the baud rate
 *
 * Return 0 on success or -1 on error.
 */
uint8_t set_hardware_flow_control(struct termios *options, bool enable)
{
    if (enable)
    {
        options.c_cflag |= CRTSCTS; /* Enable hardware flow control. */
    }
    else if (!enable)
    {
        options.c_cflag &= ~CRTSCTS; /* Disable hardware flow control. */
    }
    else
    {
    }
    return 0;
}

/*
 * `set_software_flow_control` - set the baud rate
 *
 * Return 0 on success or -1 on error.
 */
uint8_t set_software_flow_control()
{
    return 0;
}

/*
 * `set_io_mode` - set the baud rate
 *
 * Return 0 on success or -1 on error.
 */
uint8_t set_io_mode()
{
    return 0;
}

/*
 * `set_read_timeouts` - set the baud rate
 *
 * Return 0 on success or -1 on error.
 */
uint8_t set_read_timeouts()
{
    return 0;
}

/*
* `open_port(char *port)` - Open the port
*
* Returns the file descriptor on success or -1 on error.
*/
int open_port(char *port)
{
    int fd; /* File descriptor for this port */
    int status;

    //fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(port, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        /* Could not open the port */
        perror("open_port: Unable to open port");
    }
    else
    {
        status = fcntl(fd, F_SETFL, 0);
    }
    if (status < 0)
    {
        debug_info("fcntl failed!");
        return -1;
    }

    ioctl(fd, TCOFLUSH, &status);

    return fd;
}

/*
 * `set_port` - set the port
 *
 * Return 0 on success or -1 on error.
 */
uint8_t set_port(int fd, uint32_t baud_rate, uint8_t data_bits, uint8_t parity_checking, uint8_t stop_bit)
{
    int ret;
    struct termios options;
    tcgetattr(fd, &options);

    //    cfsetspeed(&options, B9600);	/* Setting baud rate as in */
    ret = set_baud_rate(&options, 9600);
    if (ret < 0)
    {
        debug_info("Error: Have not match the baud rate!");
        return -1;
    }
    options.c_cflag |= (CLOCAL | CREAD); /* Enable the receiver and set local mode */

    /* Setting the Character Size. */
    options.c_cflag &= ~CSIZE; /* Mask the chracter size bits */
    options.c_cflag |= CS8;    /* 8 data bits */
    debug_info("Setting the Character Size.");

    /* Setting Parity Checking */
    options.c_cflag &= ~PARENB; /* Disable parity bit */
    options.c_cflag &= ~CSTOPB; /* Setting stop bits to 1 */
    debug_info("Setting Parity Checking.");

    /* Setting Hardware Flow Control. */
    options.c_cflag &= ~CRTSCTS; /* Disable hardware flow control. */
    debug_info("Setting Hardware Flow Control.");

    /* Choosing Raw Input&Output */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*
     * When OPOST option is disabled, all other option
     * bits in c_oflag are ignored.
     */
    options.c_oflag &= ~OPOST;
    debug_info("Choosing Raw Input&Output.");

    /* Setting Software Flow Control */
    options.c_iflag &= ~(IXON | IXOFF | IXANY); /* Disable software flow control. */
    debug_info("Setting Software Flow Control.");

    /*
     * Setting Read Timeouts
     * Timeouts are ignored in canonical input mode
     * or when the NDELAY option is set on the file
     * via open or fcntl.
     */
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 1;
    debug_info("Setting Read Timeouts.");

    tcflush(fd, TCIFLUSH);

    /*
     * TCSANOW
     * Make change now without waiting for data to complete.
     */
    tcsetattr(fd, TCSANOW, &options);
    debug_info("Making options effect.");
}

/*
 * `serial_rw_help` - print the full usage.
 *
 * None returns.
 */
void serial_rw_help(void)
{
    /*
     * FULL usage for newers.
	 */
     printf("
		Usage: serial_rw [<tty>|<ttyS>|<ttyUSB>] [data] [-f <file>]\n
		[-s <baud-rate>] [-]\n
		\n
		These commands used in serial_rw:\n
		\n
		
	 ");
}

int main(int argc, char **argv)
{
    /* Usage: ./serial_sw <ttyN> text */
    if (argc < 2)
    {
        printf("Usage: ./serial_sw <tty>|<ttyS>|<ttyUSB> [data]\n
        		try '--help' for more info.");
        return -1;
    }

    int fd;
    int ret;
    uint8_t r_buf[512], w_buf[64];
    uint8_t command[20];
    /* Opening a serial port */
    fd = open_port(argv[1]);
    if (fd < 0)
    {
        debug_info("Error on opening port.");
        return -1;
    }

    /* Setting the struct termios */
    ret = set_port(fd, 115200, 8, 'N', 1);
    if (ret < 0)
    {
        debug_info("Error on setting port.");
    }

    /*sprintf(w_buf, "%s\r\n", "VER;");
    printf("Sending: %s \nSize:%ld\n", w_buf,sizeof(w_buf));
    write(fd, w_buf, sizeof(w_buf));
    read(fd, r_buf, sizeof(r_buf));
    printf("Recv: %s\n", r_buf);*/
    // sleep(1);
    /*for(int i=0;i<64;i++){

    	sprintf(w_buf, "CLR(%d);\r\n", i);
    	printf("Sending: %s \nSize:%ld\n", w_buf,sizeof(w_buf));
    	write(fd, w_buf, sizeof(w_buf));
    	ret = read(fd, r_buf, sizeof(r_buf));
    	while(r_buf[0]!='O');
    	{
    		printf("Command ok!run next.\n");
    	}
    	printf("Recv: %s", r_buf);
    	sleep(0.5);
    }

    for(int i=0;i<64;i++){

    	sprintf(w_buf, "BOX(0,0,%d,160,%d);\r\n", i*2,i);
    	printf("Sending: %s \nSize:%ld\n", w_buf,sizeof(w_buf));
    	write(fd, w_buf, sizeof(w_buf));
    	ret = read(fd, r_buf, sizeof(r_buf));
    	while(r_buf[0]!='O');
    	{
    		printf("Command ok!run next.\n");
    	}
    	printf("Recv: %s", r_buf);
    	sleep(0.5);
    }*/

    //Reading raw data from GPS Module.
    while (1)
    {
        /*printf("Command: ");
        scanf("%s",command);
        sprintf(w_buf, "%s\r", command);
        printf("Sending: %sSize:%ld\n", w_buf,sizeof(w_buf));
        write(fd, w_buf, sizeof(w_buf));*/
        ret = read(fd, r_buf, sizeof(r_buf));
        printf("\nRev:%s", r_buf);
    }
    close(fd);
    return 0;
}
