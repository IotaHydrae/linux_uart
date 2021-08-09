#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdint.h>  /* Standard typedefs */

#define set_baud_rate(fd, opt, br) \
	tcgetattr(fd, &opt);		\
	cfsetspeed(&opt, B115200);	\
	opt.c_cflag |= (CLOCAL | CREAD);	\
	tcsetattr(fd, TCSANOW, &opt)

#define debug_info(msg)//##fprintf(stderr,"%s\n",msg)

#define BAUD(ori)(B##ori)

/*
* `open_port(char *port)` - Open the port
*
* Returns the file descriptor on success or -1 on error.
*/
int open_port(char *port)
{
    int fd; /* File descriptor for this port */

    fd = open(port, O_RDWR | O_NOCTTY);
    if(fd == -1) {
        /* Could not open the port */
        perror("open_port: Unable to open port");
    } else {
        //fcntl(fd, F_SETFL, 0);
    }

    return fd;
}

int setting_port(int fd, int baud_rate, int char_size, int parity_checking, int hard_flow_c)
{
    struct termios options;
    tcgetattr(fd, &options);

    /* Setting the Baud Rate. */
    cfsetspeed(&options, B115200);	/* Setting baud rate as 115200 */
    options.c_cflag |= (CLOCAL | CREAD);	/* Enable the receiver and set local mode */
	debug_info("Setting the Baud Rate.");

    /* Setting the Character Size. */
    options.c_cflag &= ~CSIZE;		/* Mask the chracter size bits */
    options.c_cflag |= CS8;			/* 8 data bits */
	debug_info("Setting the Character Size.");

	/* Setting Parity Checking */
	options.c_cflag &= ~PARENB;		/* Disable parity bit */
    options.c_cflag &= ~CSTOPB;		/* Setting stop bits to 1 */
	debug_info("Setting Parity Checking.");

    /* Setting Hardware Flow Control. */
    options.c_cflag &= ~CRTSCTS;	/* Disable hardware flow control. */
	debug_info("Setting Hardware Flow Control.");

    /* Choosing Raw Input&Output */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	/*
	 * When OPOST option is disabled, all other option
	 * bits in c_oflag are ignored.
	 */
	options.c_oflag	&= ~OPOST;	
	debug_info("Choosing Raw Input&Output.");

	/* Setting Software Flow Control */
	options.c_iflag &= ~(IXON | IXOFF | IXANY);		/* Disable software flow control. */
	debug_info("Setting Software Flow Control.");

	/* 
	 * Setting Read Timeouts
	 * Timeouts are ignored in canonical input mode
	 * or when the NDELAY option is set on the file
	 * via open or fcntl.
	 */
	options.c_cc[VTIME]=1;
	options.c_cc[VMIN]=0;
	debug_info("Setting Read Timeouts.");

	/* 
	 *	TCSANOW: Make change now without waiting for data to complete. 
	 */
    tcsetattr(fd, TCSANOW, &options);	
	debug_info("Making options effect.");

}

int main(int argc, char **argv)
{
    /* Usage: ./serial_sw <ttyN> text */
    if(argc < 2) {
        printf("Usage: ./serial_sw <ttyN> text\n");
        return -1;
    }

    int fd;
    uint8_t r_buf[256], w_buf[256];
    /* Opening a serial port */
    fd = open_port(argv[1]);

    /* Setting the struct termios */
    setting_port(fd, 115200, 8, 0, 0);
	
    write(fd, "AT\r", 3);
    read(fd, r_buf, 3);

    printf("%s\n", r_buf);

    close(fd);
    return 0;
}
