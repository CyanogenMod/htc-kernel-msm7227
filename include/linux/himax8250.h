#ifndef HIMAX8250_H
#define HIMAX8250_H
#include <linux/types.h>

#define HIMAX8250_NAME "Himax8250"
struct himax_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_width_min;
	int abs_width_max;
	int (*power)(int on);
	int gpio_irq;
	uint8_t command_76[3];
	uint8_t command_6e[2];
	uint8_t command_39[2];
	uint8_t command_37[5];
	uint8_t command_7d[6];
	uint8_t command_c2[5];
	uint8_t command_7f[15];
	uint8_t command_c0[4];
	uint8_t command_62[11];
	uint8_t command_63[11];
	uint8_t command_64[11];
	uint8_t command_65[11];
	uint8_t command_66[11];
	uint8_t command_67[11];
	uint8_t command_68[11];
	uint8_t command_69[11];
	uint8_t command_6a[11];
	uint8_t command_6b[11];
	uint8_t command_6c[11];
	uint8_t command_6d[11];
	uint8_t command_c9[36];
	uint8_t command_cb[11];
	uint8_t command_d4[4];
	uint8_t command_b2[49];
	uint8_t command_b3[49];
	uint8_t command_c5[8];
	uint8_t command_c6[4];
	uint8_t command_7a[4];
	uint8_t command_78[2];
	uint8_t command_3a[2];
	uint8_t command_e9[3];
	uint8_t command_ea[5];
	uint8_t command_eb[5];
	uint8_t command_ec[10];
	uint8_t command_ee[2];
	uint8_t command_ed[5];
	uint8_t command_ef[3];
	uint8_t command_f0[2];
	uint8_t command_f1[5];
	uint8_t command_f2[5];
	uint8_t command_f3[2];
	uint8_t command_f4[5];
	uint8_t command_f7[6];
	uint8_t command_e1[2];
	uint8_t cable_config[2];
};
#endif

