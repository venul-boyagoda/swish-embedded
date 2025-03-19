/**
 * @file quaternion_utils.h
 * @brief Quaternion manipulation utilities for IMU data processing
 */

 #ifndef QUATERNION_UTILS_H
 #define QUATERNION_UTILS_H
 
 #include <zephyr/kernel.h>
 #include "onboard_imu.h"
 #include "bno055_driver.h"
 #include "mahony.h"
 #include <math.h>
 
 extern uint8_t imu_data_combined[108];

 
 /**
  * @brief Normalizes a quaternion to unit length
  * 
  * @param w Pointer to quaternion w component
  * @param x Pointer to quaternion x component
  * @param y Pointer to quaternion y component
  * @param z Pointer to quaternion z component
  */
 void normalize_quat(float *w, float *x, float *y, float *z);
 
 /**
  * @brief Converts a quaternion to a rotation matrix
  * 
  * @param w Quaternion w component
  * @param x Quaternion x component
  * @param y Quaternion y component
  * @param z Quaternion z component
  * @param R Output 3x3 rotation matrix
  */
 void quaternion_to_rot_mat(float w, float x, float y, float z, float R[3][3]);
 
 /**
  * @brief Calculates relative quaternions for both IMUs based on reference quaternions
  */
 void calculate_relative_quaternions(void);
 

 // Update rotation matrix and send it over BLE
 void update_rotation_matrix_and_notify(void); 
  /**
  * @brief Resets the reference quaternions to current orientation
  * Used when button is pressed
  */
 void reset_quaternion_reference(void);
 
 #endif /* QUATERNION_UTILS_H */