#ifndef TXTSERIALPORT_H_
#define TXTSERIALPORT_H_


/**
 * Dictionary
 * ms =  motorshield
 */

int open_ms_serialport();

int read_ms_serialport(int fd, char* buffer, int size);

int write_ms_serialport(int fd, char* buffer, int size);

void close_ms_serialport(int fd);

#endif /* TXTSERIALPORT_H_ */
