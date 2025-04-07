/**
 * @file quaternion_utils.c
 * @brief Implementation of quaternion manipulation utilities
 */

 #include "quaternion_utils.h"
 #include <zephyr/sys/byteorder.h>
 #include <zephyr/bluetooth/gatt.h>
 #include "bluetooth.h"
 
 // External declarations for BLE functionality
 uint8_t imu_data_combined[34];
 // extern void notify_imu_data(void);

 
 float rot_mat_BNO[3][3];
 float rot_mat_BMI[3][3];
 float bmi_gyro[3];
 float bno_gyro[3];
 float bno_accel[3];
 
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
     quaternion_to_rot_mat(q0_rel_bno, q1_rel_bno, q2_rel_bno, q3_rel_bno, rot_mat_BNO);

     // Generate rotation matrix from BMI270 relative quaternion
     quaternion_to_rot_mat(q0_rel_onboard, q1_rel_onboard, q2_rel_onboard, q3_rel_onboard, rot_mat_BMI);
     
     // Convert rotation matrix to little-endian byte array for BLE for BNO055 & BMI270
     for (int i = 0; i < 9; i++) {
        sys_put_le32(*(uint32_t *)&rot_mat_BNO[i / 3][i % 3], &imu_data_combined[i * 4]);
        sys_put_le32(*(uint32_t *)&rot_mat_BMI[i / 3][i % 3], &imu_data_combined[36 + i * 4]);
     }

     // extract and store Gyro data from BMI270 and BNO055
     get_gyro_data(bmi_gyro);
     fetch_gyro_values_rps(bno_gyro);
     
     // extract and store Linear Acceleration from BNO055
     fetch_linear_accel(bno_accel);

     // combine all extracted data into a single array and convert to uint32_t
     for (int i = 0; i < 3; i++) {
        sys_put_le32(*(uint32_t *)&bno_gyro[i], &imu_data_combined[72 + i * 4]);
        sys_put_le32(*(uint32_t *)&bno_accel[i], &imu_data_combined[84 + i * 4]);
        sys_put_le32(*(uint32_t *)&bmi_gyro[i], &imu_data_combined[96 + i * 4]);
    }

     // Notify BLE clients
     notify_imu_data();
 }

 void send_scaled_imu_data() {
    const float scale = 10000.0f;

    // Scale quaternions
    int16_t bno_q[4] = {
        (int16_t)(q0_rel_bno * scale),
        (int16_t)(q1_rel_bno * scale),
        (int16_t)(q2_rel_bno * scale),
        (int16_t)(q3_rel_bno * scale)
    };
    int16_t bmi_q[4] = {
        (int16_t)(q0_rel_onboard * scale),
        (int16_t)(q1_rel_onboard * scale),
        (int16_t)(q2_rel_onboard * scale),
        (int16_t)(q3_rel_onboard * scale)
    };

    // Scale gyro and accel values
    get_gyro_data(bmi_gyro);               // fills bmi_gyro[3]
    fetch_gyro_values_rps(bno_gyro);      // fills bno_gyro[3]
    fetch_linear_accel(bno_accel);        // fills bno_accel[3]

    int16_t bno_gyro_scaled[3], bmi_gyro_scaled[3], bno_accel_scaled[3];
    for (int i = 0; i < 3; i++) {
        bno_gyro_scaled[i]  = (int16_t)(bno_gyro[i]  * scale);
        bmi_gyro_scaled[i]  = (int16_t)(bmi_gyro[i]  * scale);
        bno_accel_scaled[i] = (int16_t)(bno_accel[i] * scale);
    }

    // Pack into imu_data_combined (little-endian)
    int offset = 0;
    for (int i = 0; i < 4; i++) sys_put_le16(bno_q[i],       &imu_data_combined[offset += 0]);
    for (int i = 0; i < 4; i++) sys_put_le16(bmi_q[i],       &imu_data_combined[offset += 2]);
    for (int i = 0; i < 3; i++) sys_put_le16(bno_gyro_scaled[i],  &imu_data_combined[offset += 2]);
    for (int i = 0; i < 3; i++) sys_put_le16(bmi_gyro_scaled[i],  &imu_data_combined[offset += 2]);
    for (int i = 0; i < 3; i++) sys_put_le16(bno_accel_scaled[i], &imu_data_combined[offset += 2]);

    notify_imu_data();  // Send all 34 bytes
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