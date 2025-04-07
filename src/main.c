#include <zephyr/kernel.h>
#include <stdio.h>
#include "bno055_driver.h"
#include "onboard_imu.h"
#include "bluetooth.h"
#include "quaternion_utils.h"
#include "button.h"

#define SLEEP_TIME_MS 10 // sensor frequency is 100 Hz
#define RECALIBRATION_PERIOD 20 // in seconds


int main(void)
{
    // Initialize button
    setup_button();

    // Initialize BLE
    if (!init_bluetooth()) {
        printk("Bluetooth initialization failed\n");
        return -1;
    }

    // Initialize BNO055
    if (!init_bno055()) {
        printk("BNO055 initialization failed\n");
        return -1;
    }

    // Initialize onboard IMU
    if (!init_onboard_imu()) {
        printk("Onboard IMU initialization failed\n");
        return -1;
    }

    // Variables for printing data periodically
    int counter = 1;

    // Main loop
    while (true) {
        // Read and process BNO055 data
        update_bno055_data();
        
        // Read and process onboard IMU data
        update_onboard_imu_data();
        
        // Calculate relative quaternions
        calculate_relative_quaternions();
        
        // Convert quaternion to rotation matrix and send over BLE
        // update_rotation_matrix_and_notify();
        send_scaled_imu_data();
        
        // Print data every 1 sec (100 cycles at 10ms intervals)
        if (counter % 100 == 0) {
            printk("BNO Relative Quaternion: Q0=%.4f, Q1=%.4f, Q2=%.4f, Q3=%.4f\n", q0_rel_bno, q1_rel_bno, q2_rel_bno, q3_rel_bno);
            printk("Onboard Sensor Quaternion: Q0=%.4f, Q1=%.4f, Q2=%.4f, Q3=%.4f\n", q0_rel_onboard, q1_rel_onboard, q2_rel_onboard, q3_rel_onboard);
            counter = 0;
        }
        counter++;

        k_msleep(SLEEP_TIME_MS);
    }
    
    return 0;
}