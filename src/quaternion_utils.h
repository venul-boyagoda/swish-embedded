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
 
 // Global quaternion variables
//  extern float q0, q1, q2, q3; // BNO055 quaternion
// //  extern float qq0, qq1, qq2, qq3; // Onboard IMU quaternion (from Mahony filter)
 
//  // Reference quaternions for calculating relative orientation
//  extern float q0_ref_bno, q1_ref_bno, q2_ref_bno, q3_ref_bno;
//  extern float q0_rel_bno, q1_rel_bno, q2_rel_bno, q3_rel_bno;
 
//  extern float q0_ref_onboard, q1_ref_onboard, q2_ref_onboard, q3_ref_onboard;
//  extern float q0_rel_onboard, q1_rel_onboard, q2_rel_onboard, q3_rel_onboard;
 
 // Rotation matrix 
 extern float rot_mat[3][3];
 
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