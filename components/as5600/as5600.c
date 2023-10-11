#include <stdio.h>
#include "as5600.h"


//  CONFIGURATION REGISTERS
const uint8_t AS5600_ZMCO = 0x00;
const uint8_t AS5600_ZPOS = 0x01;  // + 0x02
const uint8_t AS5600_MPOS = 0x03;  // + 0x04
const uint8_t AS5600_MANG = 0x05;  // + 0x06
const uint8_t AS5600_CONF = 0x07;  // + 0x08

//  CONFIGURATION BIT MASKS - byte level
const uint8_t AS5600_CONF_POWER_MODE    = 0x03;
const uint8_t AS5600_CONF_HYSTERESIS    = 0x0C;
const uint8_t AS5600_CONF_OUTPUT_MODE   = 0x30;
const uint8_t AS5600_CONF_PWM_FREQUENCY = 0xC0;
const uint8_t AS5600_CONF_SLOW_FILTER   = 0x03;
const uint8_t AS5600_CONF_FAST_FILTER   = 0x1C;
const uint8_t AS5600_CONF_WATCH_DOG     = 0x20;


//  UNKNOWN REGISTERS 0x09-0x0A

//  OUTPUT REGISTERS
const uint8_t AS5600_RAW_ANGLE_MSB = 0x0C;  // + 0x0D
const uint8_t AS5600_RAW_ANGLE_LSB = 0x0D;
const uint8_t AS5600_ANGLE_MSB = 0x0E;  // + 0x0F
const uint8_t AS5600_ANGLE_LSB = 0x0F;

// I2C_ADDRESS REGISTERS (AS5600L)
const uint8_t AS5600_I2CADDR   = 0x20;
const uint8_t AS5600_I2CUPDT   = 0x21;

//  STATUS REGISTERS
const uint8_t AS5600_STATUS    = 0x0B;
const uint8_t AS5600_AGC       = 0x1A;
const uint8_t AS5600_MAGNITUDE = 0x1B;  // + 0x1C
const uint8_t AS5600_BURN      = 0xFF;

//  STATUS BITS
const uint8_t AS5600_MAGNET_HIGH   = 0x08;
const uint8_t AS5600_MAGNET_LOW    = 0x10;
const uint8_t AS5600_MAGNET_DETECT = 0x20;



uint16_t as_read_angle(int master_port, int device_addr)
{
    uint8_t read_buffer_lsb;
    uint8_t read_buffer_msb;
    i2c_master_write_read_device(master_port, device_addr, &AS5600_ANGLE_LSB, sizeof(AS5600_ANGLE_LSB), &read_buffer_lsb, sizeof(read_buffer_lsb), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_master_write_read_device(master_port, device_addr, &AS5600_ANGLE_MSB, sizeof(AS5600_ANGLE_MSB), &read_buffer_msb, sizeof(read_buffer_msb), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ((uint16_t)read_buffer_msb << 8) | read_buffer_lsb;
}

uint16_t as_read_angle_raw(int master_port, int device_addr)
{
    uint8_t read_buffer_lsb;
    uint8_t read_buffer_msb;
    i2c_master_write_read_device(master_port, device_addr, &AS5600_RAW_ANGLE_LSB, sizeof(AS5600_RAW_ANGLE_LSB), &read_buffer_lsb, sizeof(read_buffer_lsb), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_master_write_read_device(master_port, device_addr, &AS5600_RAW_ANGLE_MSB, sizeof(AS5600_RAW_ANGLE_MSB), &read_buffer_msb, sizeof(read_buffer_msb), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ((uint16_t)read_buffer_msb << 8) | read_buffer_lsb;
}
