#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
#include "lib/txtserialport.h"
#include "lib/txtmotorshield.h"


int main() {
	int fd = open_ms_serialport();
	char buffer[15] = { 0x00 };

	config_switch(1);
	config_resistor(3);
	config_ultra_sonic(8);
	txt_io_config io_config = *get_txt_io_config();
	printf("\n +----------------------------------+\n");
	printf("\n Struct io_config %d \n", sizeof(io_config));
	char* p = (char *) &io_config;
	for (int i = 0; i < sizeof(io_config); i++) {
		printf("%02x ", *p++);
	}
	printf("\n +----------------------------------+\n");

	for (int i = 0; i < 5; i++) {
		char* pbuffer = (char *) &io_config;
		int bytes_written = write_ms_serialport(fd, pbuffer, 15);
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


