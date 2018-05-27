#ifndef TXTSERIALPORT_H_
#define TXTSERIALPORT_H_


/**
 * Dictionary
 * ms =  motorshield
 */

#define C_VOLTAGE  0
#define C_SWITCH  1
#define C_RESISTOR  1
#define C_ULTRASONIC  3
#define C_ANALOG  0
#define C_DIGITAL  1
#define C_OUTPUT  0
#define C_MOTOR  1

// command codes for TXT motor shield
#define C_MOT_CMD_CONFIG_IO  0x51
#define C_MOT_CMD_EXCHANGE_DATA  0x54

// input configuration codes for TXT motor shield
#define C_MOT_INPUT_DIGITAL_VOLTAGE  0
#define C_MOT_INPUT_DIGITAL_5K  1
#define C_MOT_INPUT_ANALOG_VOLTAGE  2
#define C_MOT_INPUT_ANALOG_5K  3
#define C_MOT_INPUT_ULTRASONIC  4

int open_ms_serialport();

int read_ms_serialport(int fd, char* buffer, int size);

int write_ms_serialport(int fd, char* buffer, int size);

void close_ms_serialport(int fd);

#endif /* TXTSERIALPORT_H_ */
