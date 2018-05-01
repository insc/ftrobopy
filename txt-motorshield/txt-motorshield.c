#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
#include "lib/txtserialport.h"

int main() {
	int fd = open_ms_serialport();
	char write_buffer[] = { 0x51, 0x01, 0x00, 0x11, 0x11, 0x11, 0x11, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	char buffer[15] = { 0x00 };
	for (int i = 0; i < 15; i++)
		printf("%02x ", buffer[i]);

	printf("\n +----------------------------------+\n");

	while (1) {
		int bytes_written = write_ms_serialport(fd, write_buffer, 15);
		printf("\n  Bytes written %d", bytes_written);
		int bytes_read = read_ms_serialport(fd, buffer, 15);
		printf("\n  Bytes read %d", bytes_read); /* Print the number of bytes read */
		printf("\n  ");
		for (int i = 0; i < bytes_read; i++) /*printing only the received characters*/
			printf("%02x ", buffer[i]);
		printf("\n +----------------------------------+\n");
		usleep(100000);
	}
	close_ms_serialport(fd);
	return 0;

}


