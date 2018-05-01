#include "txtserialport.h"
#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */

#define MS_SERIALPORT "/dev/ttyO2"

int open_ms_serialport() {
	int fd = open(MS_SERIALPORT, O_RDWR | O_NOCTTY);
	if (fd == 1) {
		printf("Error in opening ttyO2\n");
	} else {
		printf("Opened Successfully\n");
	}
	struct termios SerialPortSettings; /* Create the structure                          */

	tcgetattr(fd, &SerialPortSettings); /* Get the current attributes of the Serial port */

	/* Setting the Baud rate */
	cfsetispeed(&SerialPortSettings, B230400);

	if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
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
