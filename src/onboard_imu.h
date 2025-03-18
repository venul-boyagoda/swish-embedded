#ifndef ONBOARD_IMU_H
#define ONBOARD_IMU_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdbool.h>
#include "mahony.h"

// Initialize onboard IMU
bool init_onboard_imu(void);

// Calibrate onboard sensor
void calibrate_sensor(const struct device *dev);

// Get calibrated readings from onboard sensor
void get_calibrated_readings(const struct device *dev, struct sensor_value *acc, 
                             struct sensor_value *gyr, float *accel, float *gyro);

// Configure sensor parameters
void configure_sensors(const struct device *dev, enum sensor_channel chan, 
                       struct sensor_value *full_scale, struct sensor_value *sampling_freq, 
                       struct sensor_value *oversampling);

// Configure Thingy sensors
void configure_thingy_sensors(const struct device *accgyro, const struct device *mag);

// Check if devices are ready
void check_device_ready(const struct device *bmi, const struct device *mag);

// Apply low-pass filter to data
float lowPassFilter(float new_value, float *filtered_value, float alpha);

// Check if gyro is stationary
bool is_gyro_stationary(const struct device *dev);

// Update onboard IMU data
void update_onboard_imu_data(void);

// External quaternion data
// extern volatile float qq0, qq1, qq2, qq3;
extern volatile float q0_ref_onboard, q1_ref_onboard, q2_ref_onboard, q3_ref_onboard;
extern volatile float q0_rel_onboard, q1_rel_onboard, q2_rel_onboard, q3_rel_onboard;

#endif // ONBOARD_IMU_H