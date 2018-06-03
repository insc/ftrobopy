#include "txtmotorshield.h"
#include "txtserialport.h"
#include <stdio.h>
#include <stdbool.h>

/* private */
/*
 *
 * TrailFollower -->  C_MOT_INPUT_DIGITAL_VOLTAGE  --> 0
 * Switch        -->  C_MOT_INPUT_DIGITAL_5K       --> 1
 * Voltage       -->  C_MOT_INPUT_ANALOG_VOLTAGE   --> 2
 * ColorSensor   -->  C_MOT_INPUT_ANALOG_VOLTAGE   --> 2
 * Resistor      -->  C_MOT_INPUT_ANALOG_5K        --> 3
 * UltraSonic    -->  C_MOT_INPUT_ULTRASONIC       --> 4
 *
 */

// command codes for TXT motor shiel
#define C_MOT_CMD_CONFIG_IO  0x51
#define C_MOT_CMD_EXCHANGE_DATA  0x54

// input configuration codes for TXT motor shield
#define C_MOT_INPUT_DIGITAL_VOLTAGE  0
#define C_MOT_INPUT_DIGITAL_5K  1
#define C_MOT_INPUT_ANALOG_VOLTAGE  2
#define C_MOT_INPUT_ANALOG_5K  3
#define C_MOT_INPUT_ULTRASONIC  4

char cycle_count = 0x00;
char last_cycle_count = 0x00;
int fd_serial_port;

txt_io_config io_config = { .cmd = C_MOT_CMD_CONFIG_IO, .cycle = 0x00, .master = 0x00, .ftX1_uni = { 0x11, 0x11, 0x11, 0x11 }, .crc_1 = 0x00, .crc_2 = 0x00, .empty = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };

void increment_cycle_count() {
	if (last_cycle_count == cycle_count) {
		last_cycle_count = cycle_count++;
		cycle_count = cycle_count % 16;
	}
}

void set_cycle_count() {
	io_config.cycle = cycle_count;
}

void set_input(int port, char config) {
	int index = (port - 1) / 2;
	bool high = (port - 1) % 2;
	io_config.ftX1_uni[index] = (io_config.ftX1_uni[index] & (high == true ? 0x0F : 0xF0)) | (config << (4 * (high == true ? 1 : 0)));
	increment_cycle_count();
	set_cycle_count();
}



/*public */
void open_init_motorshield() {
	fd_serial_port = open_ms_serialport();
	increment_cycle_count();
	io_config.cycle = cycle_count;
}
void config_trail_follower(int port) {
	set_input(port, C_MOT_INPUT_DIGITAL_VOLTAGE);
}

void config_switch(int port) {
	set_input(port, C_MOT_INPUT_DIGITAL_5K);
}

void config_voltage(int port) {
	set_input(port, C_MOT_INPUT_ANALOG_VOLTAGE);
}

void config_color_sensor(int port) {
	set_input(port, C_MOT_INPUT_ANALOG_VOLTAGE);
}

void config_resistor(int port) {
	set_input(port, C_MOT_INPUT_ANALOG_5K);
}

void config_ultra_sonic(int port) {
	set_input(port, C_MOT_INPUT_ULTRASONIC);
}

txt_io_config* get_txt_io_config() {
	return &io_config;
}

void close_motorshield() {
	close_ms_serialport(fd_serial_port);
}
