#include "onboard_imu.h"
#include <stdio.h>
#include <math.h>

#define STATIONARY_THRESHOLD 0.03f // to consider the gyro stationary in rad/s

const struct device *const bmi = DEVICE_DT_GET(DT_NODELABEL(bmi270));
const struct device *const mag = DEVICE_DT_GET(DT_NODELABEL(bmm150));

// Sensor data variables
float accel_data[3] = {0.0f, 0.0f, 0.0f};
float gyro_data[3] = {0.0f, 0.0f, 0.0f};
float mag_data[3] = {0.0f, 0.0f, 0.0f};

float accel_offset[3] = {0.0f, 0.0f, 0.0f};
float gyro_offset[3] = {0.0f, 0.0f, 0.0f};

// Filter variables
float alpha = 0.2f; // Filter coefficient (0.01 to 0.2 is typical)
float filtered_value_x = 0.0f;
float filtered_value_y = 0.0f;

// Quaternion variables
volatile float q0_ref_onboard, q1_ref_onboard, q2_ref_onboard, q3_ref_onboard;
volatile float q0_rel_onboard, q1_rel_onboard, q2_rel_onboard, q3_rel_onboard;

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

void check_device_ready(const struct device *bmi, const struct device *mag) {
    if (!device_is_ready(bmi)) {
        printf("Device %s is not ready\n", bmi->name);
        return;
    }

    printf("Device %p name is %s\n", bmi, bmi->name);

    if (!device_is_ready(mag)) {
        printk("Device %s is not ready\n", mag->name);
        return;
    }

    printk("Device %p name is %s\n", mag, mag->name);

    configure_thingy_sensors(bmi, mag);
}

float lowPassFilter(float new_value, float *filtered_value, float alpha) {
    *filtered_value = alpha * new_value + (1 - alpha) * (*filtered_value);
    return *filtered_value;
}

bool is_gyro_stationary(const struct device *dev) {
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

void update_onboard_imu_data(void) {
    struct sensor_value acc[3], gyr[3];
    
    get_calibrated_readings(bmi, acc, gyr, accel_data, gyro_data);

    // Apply the low-pass filter to accel_data[0] and accel_data[1]
    float new_accel_data_x = lowPassFilter(accel_data[0], &filtered_value_x, alpha) - 0.1; // 0.1 is the bias
    float new_accel_data_y = lowPassFilter(accel_data[1], &filtered_value_y, alpha);
    
    MahonyAHRSupdateIMU(gyro_data[0], gyro_data[1], gyro_data[2], 
                         new_accel_data_x, new_accel_data_y, accel_data[2]);
    
    // Mahony algorithm updates qq0, qq1, qq2, qq3 internally
}

bool init_onboard_imu(void) {
    printk("Starting onboard IMU initialization\n");

    check_device_ready(bmi, mag);
    k_sleep(K_MSEC(10000)); // Wait for sensor to stabilize before calibrating
    calibrate_sensor(bmi);

    printk("Onboard IMU initialization complete\n");
    return true;
}