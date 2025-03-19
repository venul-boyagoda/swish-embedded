#include "bno055_driver.h"
#include <stdio.h>
#include <zephyr/sys/byteorder.h>

#define BNO055_EXPECTED_CHIP_ID 0xA0    // from datasheet
#define I2C2_NODE DT_NODELABEL(bno055)

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C2_NODE);

// Quaternion variables
float q0, q1, q2, q3;
float q0_ref_bno, q1_ref_bno, q2_ref_bno, q3_ref_bno;
float q0_rel_bno, q1_rel_bno, q2_rel_bno, q3_rel_bno;

void print_i2c_error(int error)
{
    switch (error) {
        case -EIO:
            printk("Error: Input/Output error\n");
            break;
        case -EBUSY:
            printk("Error: Device or resource busy\n");
            break;
        case -ENOTSUP:
            printk("Error: Operation not supported\n");
            break;
        default:
            printk("Error: Unknown error code %d\n", error);
    }
}

int read_chip_id(void)
{
    uint8_t chip_id;
    int ret = i2c_write_read_dt(&dev_i2c, &(uint8_t){BNO055_CHIP_ID_ADDR}, 1, &chip_id, 1);
    if (ret != 0) {
        printk("Failed to read chip ID. Error code: %d\n", ret);
        print_i2c_error(ret);
        return ret;
    }
    return (chip_id == BNO055_EXPECTED_CHIP_ID) ? 0 : -1;
}

int set_operation_mode(uint8_t operation_mode)
{
    uint8_t config[2] = {BNO055_OPR_MODE_ADDR, operation_mode};
    int ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
    if (ret != 0) {
        printk("Failed to set operation mode. Error code: %d\n", ret);
        print_i2c_error(ret);
    } else {
        printk("Operation mode set successfully\n");
    }
    return ret;
}

void i2c_scan(const struct i2c_dt_spec *dev_i2c)
{
    if (!device_is_ready(dev_i2c->bus)) {
        printk("I2C: Device is not ready.\n");
        return;
    }

    printk("I2C: Scanning bus for devices...\n");

    for (uint8_t addr = 0; addr <= 0x7F; addr++) {
        struct i2c_msg msgs[1];
        uint8_t dst;

        /* Send the address to read from */
        msgs[0].buf = &dst;
        msgs[0].len = 0;
        msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

        if (i2c_transfer(dev_i2c->bus, &msgs[0], 1, addr) == 0) {
            printk("I2C: Device found at address 0x%2x\n", addr);
        }
    }

    printk("I2C: Scan complete\n");
}

void update_bno055_data(void)
{
    uint8_t quaternion_data[8];
    int16_t q0_raw, q1_raw, q2_raw, q3_raw;

    if (i2c_burst_read_dt(&dev_i2c, BNO055_QUATERNION_DATA_W_LSB_ADDR, 
                          quaternion_data, sizeof(quaternion_data)) < 0) {
        printk("Failed to read quaternion data\n");
    } else {
        q0_raw = (int16_t)((quaternion_data[1] << 8) | quaternion_data[0]);
        q1_raw = (int16_t)((quaternion_data[3] << 8) | quaternion_data[2]);
        q2_raw = (int16_t)((quaternion_data[5] << 8) | quaternion_data[4]);
        q3_raw = (int16_t)((quaternion_data[7] << 8) | quaternion_data[6]);
        
        // Convert raw values to floating-point values
        q0 = q0_raw / 16384.0f;
        q1 = q1_raw / 16384.0f;
        q2 = q2_raw / 16384.0f;
        q3 = q3_raw / 16384.0f;
    }
}

bool init_bno055(void)
{
    printk("Starting BNO055 I2C debug\n");

    i2c_scan(&dev_i2c);

    int ret = read_chip_id();
    if (ret != 0) {
        printk("Failed to verify BNO055 chip ID\n");
        return false;
    }

    ret = set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
    if (ret != 0) {
        printk("Failed to set BNO055 operation mode\n");
        return false;
    }

    printk("BNO055 initialization complete\n");
    return true;
}

bool fetch_gyro_values_rps(float *gyro_values) {
    struct bno055_gyro_float_t gyro_data;
    BNO055_RETURN_FUNCTION_TYPE result = bno055_convert_float_gyro_xyz_rps(&gyro_data);
    if (result != BNO055_SUCCESS) {
        return false; // Indicate failure
    }
    gyro_values[0] = gyro_data.x;
    gyro_values[1] = gyro_data.y;
    gyro_values[2] = gyro_data.z;
    return true; // Indicate success
}

bool fetch_linear_accel(float *linear_accel_values) {
    struct bno055_linear_accel_t linear_accel_raw;
    BNO055_RETURN_FUNCTION_TYPE result;

    // Fetch linear acceleration data (X, Y, Z)
    result = bno055_read_linear_accel_xyz(&linear_accel_raw);
    if (result != BNO055_SUCCESS) {
        return false; // Return false if reading fails
    }

    // Convert raw linear acceleration data to m/s²
    linear_accel_values[0] = (float)linear_accel_raw.x / BNO055_LINEAR_ACCEL_DIV_MSQ;
    linear_accel_values[1] = (float)linear_accel_raw.y / BNO055_LINEAR_ACCEL_DIV_MSQ;
    linear_accel_values[2] = (float)linear_accel_raw.z / BNO055_LINEAR_ACCEL_DIV_MSQ;

    return true; // Return true if successful
}