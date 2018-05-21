#include "txtserialport.h"
#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */

#define MS_SERIALPORT "/dev/ttyO2"

int open_ms_serialport() {
	int fd = open(MS_SERIALPORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd == 1) {
		printf("Error in opening ttyO2\n");
	} else {
		printf("Opened Successfully\n");
	}
	struct termios serial_config; /* Create the structure                          */

	tcgetattr(fd, &serial_config); /* Get the current attributes of the Serial port */
	cfsetispeed(&serial_config, B230400);
	cfsetospeed(&serial_config, B230400);
	serial_config.c_cc[VMIN] = 0;
	serial_config.c_cc[VTIME] = 0;
	serial_config.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
	serial_config.c_oflag &= ~(OPOST | ONLCR);
	serial_config.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK
			| ECHOCTL | ECHOKE);
	if ((tcsetattr(fd, TCSANOW, &serial_config)) != 0) /* Set the attributes to the termios structure*/
		printf("\n  ERROR ! in Setting attributes");
	else
		printf("\n  BaudRate = B230400");

	/*------------------------------- Read data from serial port -----------------------------*/
	tcflush(fd, TCIFLUSH);
	return fd;
}

int read_ms_serialport(int fd, char* buffer, int size) {
	int bytes_read = read(fd, buffer, size);
	return bytes_read;
}

int write_ms_serialport(int fd, char* buffer, int size) {
	int bytes_written = write(fd, buffer, size);
	return bytes_written;
}

void close_ms_serialport(int fd) {
	close(fd);
}
