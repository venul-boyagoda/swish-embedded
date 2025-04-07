#include "bluetooth.h"
#include <stdio.h>
#include <stdint.h>              // For int64_t, ADDED
#include <zephyr/sys/byteorder.h>

#define IMU_OUT_INDEX 1
#define TIME_SYNC_INDEX 3

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* Define UUIDs: used to define services, characteristics, and/or descriptors in context of BLE
   UUIDs are used for the phone and BLE client to know what kind of data is being sent
   Although there are some standard features (UUIDs), you can define your own UUIDs for custom services
*/

#define BT_UUID_IMU_SERVICE_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x123456789ABC)
#define BT_UUID_IMU_CHAR_VAL BT_UUID_128_ENCODE(0x87654321, 0x4321, 0x6789, 0x4321, 0xCBA987654321)
#define BT_UUID_TIME_SYNC_CHAR_VAL BT_UUID_128_ENCODE(0x44444444, 0x4444, 0x4444, 0x4444, 0x444444444444)

static struct bt_uuid_128 imu_service_uuid = BT_UUID_INIT_128(BT_UUID_IMU_SERVICE_VAL);
static struct bt_uuid_128 imu_char_uuid = BT_UUID_INIT_128(BT_UUID_IMU_CHAR_VAL);
static struct bt_uuid_128 time_sync_char_uuid = BT_UUID_INIT_128(BT_UUID_TIME_SYNC_CHAR_VAL);

// uint8_t imu_data_combined[108];  // Buffer to hold 3×3 rotation matrix (9 floats * 4 bytes)
static struct bt_conn *default_conn = NULL;

static void notify_time_sync_t0();

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

// declaring prototype before struct call, and defining after for cleanliness
static void imu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

// Necessary for imu_service_attrs[]
static bool imu_notifications_enabled = false;

/* Bluetooth GATT Service */
static struct bt_gatt_attr imu_service_attrs[] = {
   // Primary IMU Service Declaration
   BT_GATT_PRIMARY_SERVICE(&imu_service_uuid),

   // IMU Characteristic with READ and NOTIFY --- INDEX 1
   BT_GATT_CHARACTERISTIC(&imu_char_uuid.uuid,
                        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Notify enabled
                        BT_GATT_PERM_READ,                        // Read permitted
                        read_callback, NULL, imu_data_combined),  // Handles read correctly

   // CCC for IMU Notifications
   BT_GATT_CCC(imu_ccc_cfg_changed,                                // IMU Notify subscription
               BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

   // Time Sync Characteristic with WRITE --- INDEX 3
   BT_GATT_CHARACTERISTIC(&time_sync_char_uuid.uuid,
                          BT_GATT_CHRC_WRITE,                      // Write only
                          BT_GATT_PERM_WRITE,
                          NULL, time_sync_write_cb, NULL),          // Time sync write callback

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

static void imu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    if (value == BT_GATT_CCC_NOTIFY) {
        imu_notifications_enabled = true;
        printk("✅ IMU notifications enabled\n");
    } else {
        imu_notifications_enabled = false;
        printk("❌ IMU notifications disabled\n");
    }
}


/* Callback for when a device connects */
void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("Connection failed, err 0x%02x\n", err);
        return;

        notify_time_sync_t0(); // get the time offset value
    }
    
    printk("Connected\n");
    
    // // Request 2M PHY after connection
    // struct bt_conn_le_phy_param phy_pref = {
    //     .options = 0, // no coded PHY options needed
    //     .pref_tx_phy = BT_GAP_LE_PHY_2M,
    //     .pref_rx_phy = BT_GAP_LE_PHY_2M,
    // };

    // int ret = bt_conn_le_phy_update(conn, &phy_pref);
    // if (ret) {
    //     printk("PHY update failed: %d\n", ret);
    // } else {
    //     printk("Requested 2M PHY\n");
    // }
    
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
                     void *buf, uint16_t len, uint16_t time_offset_estimate) {
    const char *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, time_offset_estimate, value, strlen(value));
}

/* Notify IMU Data */
extern void notify_imu_data(void) {
    if (default_conn) {
        bt_gatt_notify(default_conn, &imu_service_attrs[IMU_OUT_INDEX], imu_data_combined, sizeof(imu_data_combined));
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

/*
    * Time Sync Service that aligns IMU data with phone time through a reasonable estimate of the offset
    */
static ssize_t time_sync_write_cb(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf,
    uint16_t len,
    uint16_t time_offset_estimate,
    uint8_t flags) {
    if (len < sizeof(uint64_t) + sizeof(uint64_t)) {
        printk("Invalid time sync payload length\n");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = buf;

    uint64_t phone_time;
    uint64_t device_echo_time;

    // Copy 8 bytes from buf[0:8] into phone_time
    memcpy(&phone_time, &data[0], sizeof(uint64_t));
    
    // Copy 8 bytes from buf[8:16] into device_echo_time
    memcpy(&device_echo_time, &data[8], sizeof(uint64_t));

    uint64_t device_recv_time = k_uptime_get();  // get current uptime in ms

    time_offset_estimate = (int64_t)phone_time - ((int64_t)device_echo_time + (int64_t)device_recv_time) / 2;

    printk("  Time Sync:\n");
    printk("  Phone Time     : %llu\n", phone_time);
    printk("  Device Echo    : %llu\n", device_echo_time);
    printk("  Device Recv    : %llu\n", device_recv_time);
    printk("  Offset Estimate: %lld ms\n", (long long)time_offset_estimate);

    // Store offset globally if needed
    // time_offset_ms = offset;

    return len;
}

static void notify_time_sync_t0(void) {
    if (!default_conn) return;


    uint64_t t0 = k_uptime_get();

    printk("📤 Sending time sync t₀: %llu ms\n", t0);

    bt_gatt_notify(default_conn, &imu_service_attrs[TIME_SYNC_INDEX], &t0, sizeof(t0));
}