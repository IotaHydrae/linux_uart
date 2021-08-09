#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdint.h>  /* Standard typedefs */
#include <stdlib.h>  /* Standard library */
#include <sys/ioctl.h>

/*
 * Default define values.
 */
#define DEFAULT_TTY_PORT  "/dev/ttyUSB0"
#define DEFAULT_BAUD_RATE 115200
#define DEFAULT_DATA_BIT  8
#define DEFAULT_PAIRTY	  'N'
#define DEFAULT_STOP_BIT  1

#define debug_info(msg)fprintf(stderr,"%s: %s\n",__func__,msg)
#define BAUD(ori)(B##ori)

/*
 * `set_baud_rate` - set the baud rate
 *
 * Return 0 on success or -1 on error.
 */
uint8_t set_baud_rate(struct termios *options, uint32_t baud_rate)
{

	uint32_t supported_baud_rate_list[] = {
		9600,
		19200,
		38400,
		57600,
		115200,
	};
    /* Setting the Baud Rate. */
	for(int i=0;i<sizeof(supported_baud_rate_list)/sizeof(uint32_t);i++){
		if(baud_rate == supported_baud_rate_list[i]){
			debug_info("Have match the baud rate.");
			break;
		}else{
			return -1;
		}
	}
	
	cfsetspeed(options, BAUD(115200));
	return 0;
}

uint8_t set_character_size(struct termios *options, uint8_t data_bit)
{
	uint8_t supported_data_bit_length[] = {
		
	};

	return 0;
}

uint8_t set_parity_checking()
{
	return 0;
}

uint8_t set_hardware_flow_control()
{
	return 0;
}

uint8_t set_software_flow_control()
{
	return 0;
}

uint8_t set_io_mode()
{
	return 0;
}

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
    if(fd == -1) {
        /* Could not open the port */
        perror("open_port: Unable to open port");
    } else {
        status = fcntl(fd, F_SETFL, 0);
    }
	if(status < 0){
		debug_info("fcntl failed!");
		return -1;
	}

	ioctl(fd, TCOFLUSH, &status);
	
    return fd;
}

uint8_t set_port(int fd, uint32_t baud_rate, uint8_t data_bits, uint8_t parity_checking, uint8_t stop_bit)
{
    struct termios options;
    tcgetattr(fd, &options);


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
	//options.c_iflag &= ~(IXON | IXOFF | IXANY);		/* Disable software flow control. */
	//debug_info("Setting Software Flow Control.");

	/* 
	 * Setting Read Timeouts
	 * Timeouts are ignored in canonical input mode
	 * or when the NDELAY option is set on the file
	 * via open or fcntl.
	 */
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN]  = 1;
	debug_info("Setting Read Timeouts.");

	tcflush(fd,TCIFLUSH);

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
        printf("Usage: ./serial_sw <ttyN> <data>\n");
        return -1;
    }

    int fd;
	int ret;
    uint8_t r_buf[256], w_buf[256];
	uint8_t c;
    /* Opening a serial port */
    fd = open_port(argv[1]);
	if(fd<0){
		debug_info("Error on opening port,");
		return -1;
	}

    /* Setting the struct termios */
    set_port(fd, 115200, 8, 'N', 1);
	
	printf("Enter a char: ");
	while (1)
	{
		fgets(w_buf, sizeof(w_buf), stdin);
		write(fd, w_buf, sizeof(w_buf));
		read(fd, r_buf, sizeof(r_buf));
		printf("Get: %s\n", r_buf);
		close(fd);
		break;
		/*scanf("%c", &c);
		ret=write(fd, &c, 1);
		ret=read(fd, &c, 1);
		if(ret==1){
			printf("Get: %c\n", c);
			close(fd);
			break;
		}*/
	}

    
    return 0;
}
