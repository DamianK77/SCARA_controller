#include "driver/i2c.h"
#define I2C_MASTER_TIMEOUT_MS       1000

uint16_t as_read_angle(int master_port, int device_addr);
uint16_t as_read_angle_raw(int master_port, int device_addr);
