#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <stdbool.h>

extern uint8_t imu_data[36];

// Initialize Bluetooth functionality
bool init_bluetooth(void);

// Notify IMU data over BLE
extern void notify_imu_data(void);

// // Update rotation matrix and send it over BLE
// void update_rotation_matrix_and_notify(void);

// BLE connection callbacks
void connected(struct bt_conn *conn, uint8_t err);
void disconnected(struct bt_conn *conn, uint8_t reason);
ssize_t read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                     void *buf, uint16_t len, uint16_t offset);

#endif // BLUETOOTH_H