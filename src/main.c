#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <stdio.h>
#include "bno055.h"
#include "mahony.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define SLEEP_TIME_MS 10
#define BNO055_EXPECTED_CHIP_ID 0xA0    // from datasheet
#define I2C2_NODE DT_NODELABEL(bno055)
#define BUTTON_NODE DT_NODELABEL(button0)

#define PI 3.14159265358979323846f
#define STATIONARY_THRESHOLD 0.03f // to consider the gyro stationary in rad/s
#define RECALIBRATION_PERIOD 20 // in seconds

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C2_NODE);

const struct device *const bmi = DEVICE_DT_GET(DT_NODELABEL(bmi270));
const struct device *const mag = DEVICE_DT_GET(DT_NODELABEL(bmm150));

static const struct device *button_dev = DEVICE_DT_GET(DT_GPIO_CTLR(BUTTON_NODE, gpios));
static const gpio_pin_t button_pin = DT_GPIO_PIN(BUTTON_NODE, gpios);
static const gpio_flags_t button_flags = DT_GPIO_FLAGS(BUTTON_NODE, gpios);

/* Callback function for button press */
static struct gpio_callback button_cb_data;

void i2c_work_handler(struct k_work *work);

/* Work queue definition */
K_WORK_DEFINE(i2c_work, i2c_work_handler);

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN     (sizeof(DEVICE_NAME) - 1)
#define BT_UUID_IMU_SERVICE_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x123456789ABC)
#define BT_UUID_IMU_CHAR_VAL BT_UUID_128_ENCODE(0x87654321, 0x4321, 0x6789, 0x4321, 0xCBA987654321)

static struct bt_uuid_128 imu_service_uuid = BT_UUID_INIT_128(BT_UUID_IMU_SERVICE_VAL);
static struct bt_uuid_128 imu_char_uuid = BT_UUID_INIT_128(BT_UUID_IMU_CHAR_VAL);

static uint8_t imu_data[36];  // Buffer to hold 3×3 rotation matrix (9 floats * 4 bytes)
static struct bt_conn *default_conn = NULL;

/* Bluetooth Advertisement Data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_IMU_SERVICE_VAL),
};

/* Callback for when a device connects */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed, err 0x%02x\n", err);
        return;
    }
    printk("Connected\n");
    default_conn = bt_conn_ref(conn);
}

/* Callback for when a device disconnects */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected, reason 0x%02x\n", reason);
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
}

ssize_t read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset)
{
    const char *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
};

/* Bluetooth GATT Service */
static struct bt_gatt_attr imu_service_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&imu_service_uuid),
    // BT_GATT_CHARACTERISTIC(&imu_char_uuid.uuid,
    //                        BT_GATT_CHRC_NOTIFY,
    //                        BT_GATT_PERM_READ,
    //                        NULL, NULL, imu_data),
    BT_GATT_CHARACTERISTIC(&imu_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        read_callback, NULL, imu_data),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service imu_service = BT_GATT_SERVICE(imu_service_attrs);

/* Notify IMU Data */
static void notify_imu_data()
{
    if (default_conn) {
        bt_gatt_notify(default_conn, &imu_service_attrs[1], imu_data, sizeof(imu_data));
    }
}

static struct bt_conn_cb conn_callbacks = {
	.connected		    = connected,
	.disconnected   	= disconnected,
};

float q0, q1, q2, q3;
float q0_ref_bno, q1_ref_bno, q2_ref_bno, q3_ref_bno;
float q0_rel_bno, q1_rel_bno, q2_rel_bno, q3_rel_bno;
float q0_ref_onboard, q1_ref_onboard, q2_ref_onboard, q3_ref_onboard;
float q0_rel_onboard, q1_rel_onboard, q2_rel_onboard, q3_rel_onboard;

float rot_mat[3][3];

// error codes below
static void print_i2c_error(int error)
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

static int read_chip_id(void)
{
    uint8_t chip_id;
    int ret = i2c_write_read_dt(&dev_i2c, &(uint8_t){BNO055_CHIP_ID_ADDR}, 1, &chip_id, 1);
    if (ret != 0) {
        printk("Failed to read chip ID. Error code: %d\n", ret);
        print_i2c_error(ret);
        return ret;
    }
        return ret;
    return (chip_id == BNO055_EXPECTED_CHIP_ID) ? 0 : -1;
}

static int set_operation_mode(uint8_t operation_mode)
{
    uint8_t config[2] = {BNO055_OPR_MODE_ADDR, operation_mode};
    int ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
    // int calib = i2c_read_dt(&dev_i2c, BNO055_CALIB_STAT_ADDR, calib);
    if (ret != 0) {
        printk("Failed to set operation mode. Error code: %d\n", ret);
        print_i2c_error(ret);
    } else {
        printk("Operation mode set successfully\n");
    }
    return ret;
}

static void i2c_scan(const struct i2c_dt_spec *dev_i2c)
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

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {

    k_work_submit(&i2c_work);
}

void setup_button(void)
{
    /* Configure button as input with pull-up and interrupt */

    int ret = gpio_pin_configure(button_dev, button_pin, GPIO_INPUT | button_flags);
    if (ret < 0) {
        printk("Error configuring button GPIO\n");
        return;
    }

    /* Set up interrupt */    
    ret = gpio_pin_interrupt_configure(button_dev, button_pin, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        printk("Error configuring button interrupt\n");
        return;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button_pin));
    gpio_add_callback(button_dev, &button_cb_data);
}

/* Function to handle I2C operation (runs in work queue) */
void i2c_work_handler(struct k_work *work)
{
    printk("Resetting Quaternions Reference");
    
    q0_ref_bno = q0;
    q1_ref_bno = q1;
    q2_ref_bno = q2;
    q3_ref_bno = q3;

    q0_ref_onboard = qq0;
    q1_ref_onboard = qq1;
    q2_ref_onboard = qq2;
    q3_ref_onboard = qq3;

}

void normalize_quat(float *w, float *x, float *y, float *z) {
    float norm = sqrt((*w) * (*w) + (*x) * (*x) + (*y) * (*y) + (*z) * (*z));

    if (norm > 0.0f) {
        *w /= norm;
        *x /= norm;
        *y /= norm;
        *z /= norm;
    }
}

void quaternion_to_rot_mat(float w, float x, float y, float z, float R[3][3]) {

    // Calculate the rotation matrix elements
    R[0][0] = 1 - 2 * (y * y + z * z);
    R[0][1] = 2 * (x * y - z * w);
    R[0][2] = 2 * (x * z + y * w);

    R[1][0] = 2 * (x * y + z * w);
    R[1][1] = 1 - 2 * (x * x + z * z);
    R[1][2] = 2 * (y * z - x * w);

    R[2][0] = 2 * (x * z - y * w);
    R[2][1] = 2 * (y * z + x * w);
    R[2][2] = 1 - 2 * (x * x + y * y);
}

// ******** ONBOARD IMU Functions ***********

// Define offset variables globally
float accel_data[3] = {0.0f, 0.0f, 0.0f};
float gyro_data[3] = {0.0f, 0.0f, 0.0f};
float mag_data[3] = {0.0f, 0.0f, 0.0f};

float accel_offset[3] = {0.0f, 0.0f, 0.0f};
float gyro_offset[3] = {0.0f, 0.0f, 0.0f};

/*
PURPOSE: Read a large number of samples and acquire an average
*/
void calibrate_sensor(const struct device *dev) {
    int samples = 100;
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    struct sensor_value accel[3], gyro[3];	
    
    printk("Calibrating sensor - keep device still...\n");
    
    // Collect samples while device is still
    for (int i = 0; i < samples; i++) {
        sensor_sample_fetch(dev);
        sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
        
        // Convert sensor_value to float and accumulate
        for (int j = 0; j < 3; j++) {
            float accel_val = sensor_value_to_float(&accel[j]);
            float gyro_val = sensor_value_to_float(&gyro[j]);
            
            accel_sum[j] += accel_val;
            gyro_sum[j] += gyro_val;
        }
        
        k_sleep(K_MSEC(10)); // Based on 100 Hz sampling rate
    }
    
    // Calculate average offset
    for (int i = 0; i < 3; i++) {
        accel_offset[i] = accel_sum[i] / samples;
        gyro_offset[i] = gyro_sum[i] / samples;
        
    }
    
    printk("Calibration complete.\n");
    printk("Accel offsets: X:%f Y:%f Z:%f\n", 
           accel_offset[0], accel_offset[1], accel_offset[2]);
    printk("Gyro offsets: X:%f Y:%f Z:%f\n", 
           gyro_offset[0], gyro_offset[1], gyro_offset[2]);
}
// Apply offsets to readings
void get_calibrated_readings(const struct device *dev, struct sensor_value *acc, struct sensor_value *gyr, float *accel, float *gyro) {
    
    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);
    
    // Apply offsets except for Z-axis
    for (int i = 0; i < 3; i++) {
        gyro[i] = (sensor_value_to_float(&gyr[i]) - gyro_offset[i]); // convert from dps to radians *PI/180.0f
    }
	accel[0] = (sensor_value_to_float(&acc[0]) - accel_offset[1]); // *9.80655f if in G
	accel[1] = (sensor_value_to_float(&acc[1]) - accel_offset[1]);
	accel[2] = sensor_value_to_float(&acc[2]);

}

void configure_sensors(const struct device *dev, enum sensor_channel chan, struct sensor_value *full_scale, struct sensor_value *sampling_freq, struct sensor_value *oversampling) {
	
	if(sampling_freq != NULL) {
		sensor_attr_set(dev, chan, SENSOR_ATTR_SAMPLING_FREQUENCY, sampling_freq);
	}
	if(full_scale != NULL) {
		sensor_attr_set(dev, chan, SENSOR_ATTR_FULL_SCALE, full_scale);
	}

	if(oversampling != NULL) {
		sensor_attr_set(dev, chan, SENSOR_ATTR_OVERSAMPLING, oversampling);
	}

}

void configure_thingy_sensors(const struct device *accgyro, const struct device *mag) {
    
	struct sensor_value full_scale, sampling_freq, oversampling;

    // Configure accelerometer
	/* Setting scale in G, due to loss of precision if the SI unit m/s^2
	 * is used
	 */
    full_scale.val1 = 2;            /* 2G of scale */
    full_scale.val2 = 0;
    sampling_freq.val1 = 100;       /* Hz. Performance mode */
    sampling_freq.val2 = 0;
    oversampling.val1 = 16;          /* Choose one of: 1, 2, 4, 8, 16, 32, 64, 128 unless in Performance mode (>=100 Hz) */
    oversampling.val2 = 0;

    configure_sensors(accgyro, SENSOR_CHAN_ACCEL_XYZ, &full_scale, &sampling_freq, &oversampling);

    // Configure gyroscope
    full_scale.val1 = 500;          /* dps */
    full_scale.val2 = 0;
    sampling_freq.val1 = 100;       /* Hz. Performance mode */
    sampling_freq.val2 = 0;
    oversampling.val1 = 4;          /* max oversampling for gyroscope */
    oversampling.val2 = 0;

    configure_sensors(accgyro, SENSOR_CHAN_GYRO_XYZ, &full_scale, &sampling_freq, &oversampling);

	// Configure magnetometer XY axis
	oversampling.val1 = 15;          // Repetition for X and Y axes, range 1-511
	oversampling.val2 = 0;

	configure_sensors(mag, SENSOR_CHAN_MAGN_X, NULL, NULL, &oversampling);
	
	oversampling.val1 = 27;          // Repetition for Z axis, range 1-256
	configure_sensors(mag, SENSOR_CHAN_MAGN_Z, NULL, NULL, &oversampling);

	sampling_freq.val1 = 30;       // Hz. from device code, supported frequencies are 2,6,8,10,15,20,25 and 30 Hz
	sampling_freq.val2 = 0;
	configure_sensors(mag, SENSOR_CHAN_MAGN_XYZ, NULL, &sampling_freq, NULL);
}

void check_device_ready(const struct device *bmi, const struct device *mag){
	if (!device_is_ready(bmi)) {
		printf("Device %s is not ready\n", bmi->name);
		return 0;
	}

	printf("Device %p name is %s\n", bmi, bmi->name);

	if (!device_is_ready(mag)) {
		printk("Device %s is not ready\n", mag->name);
		return 0;
	}

	printk("Device %p name is %s\n", mag, mag->name);

	configure_thingy_sensors(bmi, mag);
}

float lowPassFilter(float new_value, float *filtered_value, float alpha) {
    *filtered_value = alpha * new_value + (1 - alpha) * (*filtered_value);
    return *filtered_value;
}

/*
PURPOSE: Check if sensor is stationary to reset values and reduce drift
INPUT: dev - device pointer gyro values
*/
static bool is_gyro_stationary(const struct device *dev)
{
    struct sensor_value gyro[3];
    double gx, gy, gz;
    double magnitude;
    
    /* Fetch current gyro values */
    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    
    /* Convert to rad/s and compute magnitude */
    gx = sensor_value_to_double(&gyro[0]);
    gy = sensor_value_to_double(&gyro[1]);
    gz = sensor_value_to_double(&gyro[2]);
    
    magnitude = sqrt(gx*gx + gy*gy + gz*gz);
    
    return magnitude < STATIONARY_THRESHOLD;
}





int main(void)
{
    // ***** Initialization *****

    setup_button();

     // ***** BLE Initialization *****

    printk("Starting IMU BLE Peripheral\n");

    int err = bt_enable(NULL);
    printk("BT Enable Error: %d", err);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    printk("Bluetooth initialized\n");

    err = bt_conn_cb_register(&conn_callbacks);
    printk("BT Connection Callback Error: %d", err);
    if (err) {
        printk("Failed to register connection callback (err:%d)\n", err);
        return 0;
    }

    err = bt_gatt_service_register(&imu_service);
    printk("BT GATT Service Error: %d", err);
    if (err) {
        printk("Failed to register IMU service (err:%d)\n", err);
        return 0;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    printk("BT Advertise Error: %d", err);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return 0;
    }

    printk("Advertising started\n");

     // ***** BNO Initialization *****

    printk("Starting BNO055 I2C debug\n");

    i2c_scan(&dev_i2c);

    int ret = read_chip_id();
    if (ret != 0) {
        printk("Failed to verify BNO055 chip ID\n");
        return -1;
    }

    ret = set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
    if (ret != 0) {
        printk("Failed to set BNO055 operation mode\n");
        return -1;
    }

    printk("BNO055 initialization complete\n");

    // ***** Onboard IMU Initialization *****

    printk("Starting onboard IMU initialization\n");

    check_device_ready(bmi, mag);
	k_sleep(K_MSEC(10000)); // Wait for sensor to stabilize before calibrating
	calibrate_sensor(bmi);

    printk("Onboard IMU initialization complete\n");

    // ***** Variable Declarations *****

    // BNO Variables
    
    uint8_t quaternion_data[8];
    int16_t q0_raw, q1_raw, q2_raw, q3_raw;
    int16_t accel_x, accel_y, accel_z;
    float accel_x_f, accel_y_f, accel_z_f;

    // Onboard IMU Variables

    struct sensor_value acc[3], gyr[3], grav[3];
    
    float alpha = 0.2f; // Filter coefficient (0.01 to 0.2 is typical)
    float filtered_value_x = 0.0f;
    float filtered_value_y = 0.0f;

	int counter = 1;
	int counterb = 1;

    // ***** Main loop *****
    while (true) {

            // BNO Data Reading

            if (i2c_burst_read_dt(&dev_i2c, BNO055_QUATERNION_DATA_W_LSB_ADDR, quaternion_data, sizeof(quaternion_data)) < 0) {
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

                // Print raw quaternion values
                // printk("Raw Quaternion: Q0=%d, Q1=%d, Q2=%d, Q3=%d\n", q0_raw, q1_raw, q2_raw, q3_raw);

            }

            // Onboard IMU Data Reading

            get_calibrated_readings(bmi, acc, gyr, accel_data, gyro_data);

            // Apply the low-pass filter to accel_data[0] and accel_data[1]
            float new_accel_data_x = lowPassFilter(accel_data[0], &filtered_value_x, alpha) - 0.17;
            float new_accel_data_y = lowPassFilter(accel_data[1], &filtered_value_y, alpha);
            
            MahonyAHRSupdateIMU(gyro_data[0], gyro_data[1], gyro_data[2], new_accel_data_x, new_accel_data_y, accel_data[2]);

            if((counterb % 10000 == 0) && (is_gyro_stationary(bmi))){
                calibrate_sensor(bmi);
                printk("Recalibrating sensor\n");
                counterb = 0;
            }


            // Get relative quaternion
            if (q0_ref_bno) {
                float norm_sq = q0_ref_bno * q0_ref_bno + q1_ref_bno * q1_ref_bno + q2_ref_bno * q2_ref_bno + q3_ref_bno * q3_ref_bno;

                // Compute inverse of reference quaternion (conjugate divided by norm squared)
                float q0_ref_inv =  q0_ref_bno / norm_sq;
                float q1_ref_inv = -q1_ref_bno / norm_sq;
                float q2_ref_inv = -q2_ref_bno / norm_sq;
                float q3_ref_inv = -q3_ref_bno / norm_sq;

                // Compute relative quaternion (q * q_ref_inv)
                q0_rel_bno = q0 * q0_ref_inv - q1 * q1_ref_inv - q2 * q2_ref_inv - q3 * q3_ref_inv;
                q1_rel_bno = q0 * q1_ref_inv + q1 * q0_ref_inv + q2 * q3_ref_inv - q3 * q2_ref_inv;
                q2_rel_bno = q0 * q2_ref_inv - q1 * q3_ref_inv + q2 * q0_ref_inv + q3 * q1_ref_inv;
                q3_rel_bno = q0 * q3_ref_inv + q1 * q2_ref_inv - q2 * q1_ref_inv + q3 * q0_ref_inv;

                normalize_quat(&q0_rel_bno, &q1_rel_bno, &q2_rel_bno, &q3_rel_bno);
            }

            // Get relative quaternion
            if (q0_ref_onboard) {
                float norm_sq = q0_ref_onboard * q0_ref_onboard + q1_ref_onboard * q1_ref_onboard + q2_ref_onboard * q2_ref_onboard + q3_ref_onboard * q3_ref_onboard;

                // Compute inverse of reference quaternion (conjugate divided by norm squared)
                float q0_ref_inv =  q0_ref_onboard / norm_sq;
                float q1_ref_inv = -q1_ref_onboard / norm_sq;
                float q2_ref_inv = -q2_ref_onboard / norm_sq;
                float q3_ref_inv = -q3_ref_onboard / norm_sq;

                // Compute relative quaternion (q * q_ref_inv)
                q0_rel_onboard = qq0 * q0_ref_inv - qq1 * q1_ref_inv - qq2 * q2_ref_inv - qq3 * q3_ref_inv;
                q1_rel_onboard = qq0 * q1_ref_inv + qq1 * q0_ref_inv + qq2 * q3_ref_inv - qq3 * q2_ref_inv;
                q2_rel_onboard = qq0 * q2_ref_inv - qq1 * q3_ref_inv + qq2 * q0_ref_inv + qq3 * q1_ref_inv;
                q3_rel_onboard = qq0 * q3_ref_inv + qq1 * q2_ref_inv - qq2 * q1_ref_inv + qq3 * q0_ref_inv;

                normalize_quat(&q0_rel_onboard, &q1_rel_onboard, &q2_rel_onboard, &q3_rel_onboard);
            }
            
            // Print normalized quaternion values
            // printk("BNO Sensor Quaternion: Q0=%.4f, Q1=%.4f, Q2=%.4f, Q3=%.4f\n", q0, q1, q2, q3);
            // printk("BNO Reference Quaternion: Q0=%.4f, Q1=%.4f, Q2=%.4f, Q3=%.4f\n", q0_ref, q1_ref, q2_ref, q3_ref);
            // printk("BNO Relative Quaternion: Q0=%.4f, Q1=%.4f, Q2=%.4f, Q3=%.4f\n", q0_rel, q1_rel, q2_rel, q3_rel);

            quaternion_to_rot_mat(q0_rel_bno, q1_rel_bno, q2_rel_bno, q3_rel_bno, rot_mat);

            // printk("Rotation Matrix: ");
            // for (int i = 0; i < 3; i++) {
            //     for (int j = 0; j < 3; j++) {
            //         printk("%.3f ", rot_mat[i][j]);
            //     }
            //     printk("\n");
            // }

            for (int i = 0; i < 9; i++) {
                sys_put_le32(*(uint32_t *)&rot_mat[i / 3][i % 3], &imu_data[i * 4]);
            }
            notify_imu_data();

            // print data every 1 sec
            if (counter % 100 == 0){
                printk("BNO Relative Quaternion: Q0=%.4f, Q1=%.4f, Q2=%.4f, Q3=%.4f\n", q0_rel_bno, q1_rel_bno, q2_rel_bno, q3_rel_bno);
                printk("Onboard Sensor Quaternion: Q0=%.4f, Q1=%.4f, Q2=%.4f, Q3=%.4f\n", q0_rel_onboard, q1_rel_onboard, q2_rel_onboard, q3_rel_onboard);
                counter = 0;
            }
            counter++, counterb++;



            k_msleep(SLEEP_TIME_MS); 
    }
    
    return 0;
}