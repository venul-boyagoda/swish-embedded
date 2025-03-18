/**
 * @file quaternion_utils.c
 * @brief Implementation of quaternion manipulation utilities
 */

 #include "quaternion_utils.h"
 #include <zephyr/sys/byteorder.h>
 #include <zephyr/bluetooth/gatt.h>
 #include "bluetooth.h"
 
 // External declarations for BLE functionality
 extern uint8_t imu_data[36];
 extern void notify_imu_data(void);
 
 // Define quaternion variables
//  float q0, q1, q2, q3;
// //  float qq0, qq1, qq2, qq3;
 
//  float q0_ref_bno, q1_ref_bno, q2_ref_bno, q3_ref_bno;
//  float q0_rel_bno, q1_rel_bno, q2_rel_bno, q3_rel_bno;
 
//  float q0_ref_onboard, q1_ref_onboard, q2_ref_onboard, q3_ref_onboard;
//  float q0_rel_onboard, q1_rel_onboard, q2_rel_onboard, q3_rel_onboard;
 
 float rot_mat[3][3];
 
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
 
 void calculate_relative_quaternions(void) {
     // Calculate relative quaternion for BNO055
     if (q0_ref_bno) {
         float norm_sq = q0_ref_bno * q0_ref_bno + q1_ref_bno * q1_ref_bno + 
                         q2_ref_bno * q2_ref_bno + q3_ref_bno * q3_ref_bno;
 
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
 
     // Calculate relative quaternion for onboard IMU
     if (q0_ref_onboard) {
         float norm_sq = q0_ref_onboard * q0_ref_onboard + q1_ref_onboard * q1_ref_onboard + 
                         q2_ref_onboard * q2_ref_onboard + q3_ref_onboard * q3_ref_onboard;
 
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
 }
 
 void update_rotation_matrix_and_notify(void) {
     // Generate rotation matrix from BNO055 relative quaternion
     quaternion_to_rot_mat(q0_rel_bno, q1_rel_bno, q2_rel_bno, q3_rel_bno, rot_mat);
     
     // Convert rotation matrix to little-endian byte array for BLE
     for (int i = 0; i < 9; i++) {
         sys_put_le32(*(uint32_t *)&rot_mat[i / 3][i % 3], &imu_data[i * 4]);
     }
     
     // Notify BLE clients
     notify_imu_data();
 }

 void reset_quaternion_reference(void) {
     // Store current quaternions as reference quaternions
     q0_ref_bno = q0;
     q1_ref_bno = q1;
     q2_ref_bno = q2;
     q3_ref_bno = q3;
 
     q0_ref_onboard = qq0;
     q1_ref_onboard = qq1;
     q2_ref_onboard = qq2;
     q3_ref_onboard = qq3;
     
     printk("Quaternion reference reset complete\n");
 }