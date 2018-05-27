#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
#include "lib/txtserialport.h"

typedef struct txt_io_config {
	char cmd;
	char cycle;
	char master; // not used
	char ftX1_uni[4];
	char crc_1; // not used;
	char crc_2; // not used;
	char empty[6];
} txt_io_config;

int main() {
	int fd = open_ms_serialport();
	char write_buffer[] = { 0x51, 0x01, 0x00, 0x11, 0x11, 0x11, 0x11, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	char buffer[15] = { 0x00 };

	txt_io_config io_config = { .cmd = 0x51, .cycle = 0x01, .master = 0x00, .ftX1_uni = { 0x0, 0x00, 0x00, 0x00 }, .crc_1 = 0x00, .crc_2 = 0x00, .empty = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };

	io_config.cycle = 0x02;

	printf("\n +----------------------------------+\n");
	printf("\n Struct io_config %d \n", sizeof(io_config));
	char* p = (char *) &io_config;
	for (int i = 0; i < sizeof(io_config); i++) {
		printf("%02x ", *p++);
	}
	printf("\n");
	for (int i = 0; i < 15; i++)
		printf("%02x ", write_buffer[i]);

	printf("\n +----------------------------------+\n");

	for (int i = 0; i < 5; i++) {
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


