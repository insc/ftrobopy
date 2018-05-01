#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */


int writeBuf(int fd, char* buffer, int size);
int readBuf(int fd, char* buffer, int size);

int main() {
	int fd = open("/dev/ttyO2", O_RDWR | O_NOCTTY);
	if (fd == 1) {
		printf("Error! in Opening ttyO2\n");
	} else {
		printf("ttyO2 Opened Successfully\n");
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
	char write_buffer[] = { 0x51, 0x01, 0x00, 0x11, 0x11, 0x11, 0x11, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	char buffer[15] = { 0x00 };
	for (int i = 0; i < 15; i++)
		printf("%02x ", buffer[i]);
	printf("\n +----------------------------------+\n");

	while (1) {
		int bytes_written = writeBuf(fd, write_buffer, 15);
		printf("\n  Bytes written %d", bytes_written);
		int bytes_read = readBuf(fd, buffer, 15);
		printf("\n  Bytes read %d", bytes_read); /* Print the number of bytes read */
		printf("\n  ");
		for (int i = 0; i < bytes_read; i++) /*printing only the received characters*/
			printf("%02x ", buffer[i]);
		printf("\n +----------------------------------+\n");
		usleep(100000);
	}
	close(fd);
	return 0;
}

int readBuf(int fd, char* buffer, int size) {
	int bytes_read = read(fd, buffer, size);
	return bytes_read;
}

int writeBuf(int fd, char* buffer, int size) {
	int bytes_written = write(fd, buffer, size);
	return bytes_written;
}
