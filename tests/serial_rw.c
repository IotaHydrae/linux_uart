#include "../uart_utils.h"

#ifdef DEBUG
#define debug_info(msg) fprintf(stderr, "%s: %s\n", __func__, msg)
#else
#define debug_info(msg)
#endif

int main(int argc, char **argv)
{
    /* Usage: ./serial_sw <ttyN> text */
    if (argc < 2) {
        printf("Usage: ./serial_sw <tty>|<ttyS>|<ttyUSB> [data]\n"
               "%s --help for more info.\n", argv[0]);
        return -1;
    }

	if(strcmp(argv[1], "--help")==0){
		print_help();
		return 0;
	}

	int fd;
	int ret;
	uint8_t r_buf[512];
	
    /* Opening a serial port */
    fd = open_port(argv[1]);
    if (fd < 0) {
        debug_info("Error on opening port.");
        return -1;
    }

	struct uart_config myconfig;
	myconfig.baud_rate=115200;
	myconfig.data_bit=8;
	myconfig.pairty='N';
	myconfig.stop_bit=1;

    /* Setting the struct termios */
    ret = set_port(fd, &myconfig);
    if (ret < 0) {
        debug_info("Error on setting port.");
    }

    //Reading raw data from GPS Module.
    while (1) {
	sleep(1);
        ret = read(fd, r_buf, sizeof(r_buf));
        printf("\nRev:%s", r_buf);

    }
    close(fd);
    return 0;
}

