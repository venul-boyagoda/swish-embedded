#include "bluetooth.h"
#include <stdio.h>
#include <zephyr/sys/byteorder.h>

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define BT_UUID_IMU_SERVICE_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x123456789ABC)
#define BT_UUID_IMU_CHAR_VAL BT_UUID_128_ENCODE(0x87654321, 0x4321, 0x6789, 0x4321, 0xCBA987654321)

static struct bt_uuid_128 imu_service_uuid = BT_UUID_INIT_128(BT_UUID_IMU_SERVICE_VAL);
static struct bt_uuid_128 imu_char_uuid = BT_UUID_INIT_128(BT_UUID_IMU_CHAR_VAL);

// uint8_t imu_data_combined[108];  // Buffer to hold 3×3 rotation matrix (9 floats * 4 bytes)
static struct bt_conn *default_conn = NULL;

/* Bluetooth Advertisement Data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_IMU_SERVICE_VAL),
};


static struct bt_conn_cb conn_callbacks = {
    .connected       = connected,
    .disconnected    = disconnected,
};


/* Bluetooth GATT Service */
static struct bt_gatt_attr imu_service_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&imu_service_uuid),
    BT_GATT_CHARACTERISTIC(&imu_char_uuid.uuid,
                         BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                         BT_GATT_PERM_READ,
                         read_callback, NULL, imu_data_combined),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service imu_service = BT_GATT_SERVICE(imu_service_attrs);


bool init_bluetooth(void) {
    int err;

    printk("Starting IMU BLE Peripheral\n");

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return false;
    }

    printk("Bluetooth initialized\n");

    bt_conn_cb_register(&conn_callbacks);
    bt_gatt_service_register(&imu_service);

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return false;
    }

    printk("Advertising started\n");
    return true;
}

/* Callback for when a device connects */
void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("Connection failed, err 0x%02x\n", err);
        return;
    }
    printk("Connected\n");
    default_conn = bt_conn_ref(conn);
}

/* Callback for when a device disconnects */
void disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("Disconnected, reason 0x%02x\n", reason);
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
}

ssize_t read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                     void *buf, uint16_t len, uint16_t offset) {
    const char *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

/* Notify IMU Data */
extern void notify_imu_data(void) {
    if (default_conn) {
        bt_gatt_notify(default_conn, &imu_service_attrs[1], imu_data_combined, sizeof(imu_data_combined));
    }
}

// void update_rotation_matrix_and_notify(void) {
//     quaternion_to_rot_mat(q0_rel_bno, q1_rel_bno, q2_rel_bno, q3_rel_bno, rot_mat);
    
//     // Convert rotation matrix to byte array for BLE transmission
//     for (int i = 0; i < 9; i++) {
//         sys_put_le32(*(uint32_t *)&rot_mat[i / 3][i % 3], &imu_data[i * 4]);
//     }
//     notify_imu_data();
// }