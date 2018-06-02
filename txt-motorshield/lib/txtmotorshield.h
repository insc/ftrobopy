#ifndef TXTMOTORSHIELD_H_
#define TXTMOTORSHIELD_H_

typedef struct txt_io_config {
	char cmd;
	char cycle;
	char master; // not used
	char ftX1_uni[4];
	char crc_1; // not used;
	char crc_2; // not used;
	char empty[6];
} txt_io_config;

/* public */
void open_init_motorshield();

void close_motorshield();

void config_trail_follower(int port);

void config_switch(int port);

void config_voltage(int port);

void config_color_sensor(int port);

void config_resistor(int port);

void config_ultra_sonic(int port);

txt_io_config* get_txt_io_config();
#endif /* TXTMOTORSHIELD_H_ */
