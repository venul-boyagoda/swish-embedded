#include "bno055_driver.h"
#include <stdio.h>
#include <zephyr/sys/byteorder.h>

#define BNO055_EXPECTED_CHIP_ID 0xA0    // from datasheet
#define I2C2_NODE DT_NODELABEL(bno055)
#define PI 3.14159265358979323846

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C2_NODE);

/* Adding Bosch i2c device struct and creating wrappers to be able to use bosch driver functions*/
static struct bno055_t bno055_device;

static s8 zephyr_bno055_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len) {
    return i2c_burst_write(dev_i2c.bus, dev_addr, reg_addr, reg_data, len) == 0 ? BNO055_SUCCESS : BNO055_ERROR;
}

static s8 zephyr_bno055_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 len) {
    return i2c_burst_read(dev_i2c.bus, dev_addr, reg_addr, reg_data, len) == 0 ? BNO055_SUCCESS : BNO055_ERROR;
}

static void zephyr_bno055_delay(u32 msec) {
    k_msleep(msec);
}


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

    /* Bosch wrapper code */
    bno055_device.dev_addr = dev_i2c.addr;
    bno055_device.bus_write = zephyr_bno055_bus_write;
    bno055_device.bus_read = zephyr_bno055_bus_read;
    bno055_device.delay_msec = zephyr_bno055_delay;
    bno055_init(&bno055_device);  // Now p_bno055 is ready


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
    struct bno055_gyro_t gyro_raw;
    BNO055_RETURN_FUNCTION_TYPE result = bno055_read_gyro_xyz(&gyro_raw);
    if (result != BNO055_SUCCESS) {
        return false;
    }

    const float dps_scale = 1.0f / 16.0f;
    const float deg_to_rad = (PI / 180.0f);

    gyro_values[0] = ((float)gyro_raw.x * dps_scale) * deg_to_rad;
    gyro_values[1] = ((float)gyro_raw.y * dps_scale) * deg_to_rad;
    gyro_values[2] = ((float)gyro_raw.z * dps_scale) * deg_to_rad;

    return true;
}

bool fetch_linear_accel(float *linear_accel_values) {
    struct bno055_linear_accel_t accel_raw;
    u8 buffer[6];
    BNO055_RETURN_FUNCTION_TYPE result = bno055_read_linear_accel_xyz(&accel_raw);

    if (result != BNO055_SUCCESS) {
        return false;
    }

    linear_accel_values[0] = (float)accel_raw.x / BNO055_LINEAR_ACCEL_DIV_MSQ;
    linear_accel_values[1] = (float)accel_raw.y / BNO055_LINEAR_ACCEL_DIV_MSQ;
    linear_accel_values[2] = (float)accel_raw.z / BNO055_LINEAR_ACCEL_DIV_MSQ;

    return true;
}
