#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <stdbool.h>
#include "bno055.h"

// Initialize BNO055 sensor
bool init_bno055(void);

// Scan I2C bus for devices
void i2c_scan(const struct i2c_dt_spec *dev_i2c);

// Read and update BNO055 data
void update_bno055_data(void);

// Read BNO055 chip ID
int read_chip_id(void);

// Set BNO055 operation mode
int set_operation_mode(uint8_t operation_mode);

// Print I2C error codes
void print_i2c_error(int error);

// External quaternion data
extern float q0, q1, q2, q3;
extern float q0_ref_bno, q1_ref_bno, q2_ref_bno, q3_ref_bno;
extern float q0_rel_bno, q1_rel_bno, q2_rel_bno, q3_rel_bno;

#endif // BNO055_DRIVER_H