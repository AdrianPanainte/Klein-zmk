/*
 * Battery Service that reports the lower of the central's and the peripheral's
 * battery level, since most hosts only display one level per keyboard.
 *
 * Mirrors Zephyr's subsys/bluetooth/services/bas.c (same service name and
 * attribute layout, so GATT handles don't move); build with CONFIG_BT_BAS=n.
 *
 * The peripheral's level is read directly from its Battery Service once a minute.
 * ZMK's CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING is deliberately not used:
 * it subscribes during split discovery with the same discover params as the key and
 * sensor subscriptions, and the extra subscription can knock out the encoder's.
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define PERIPHERAL_POLL_FIRST K_SECONDS(15)
#define PERIPHERAL_POLL_PERIOD K_SECONDS(60)

// 0 = unknown
static uint8_t central_level;
static uint8_t peripheral_level;
static uint8_t battery_level = 100U;

static void blvl_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    ARG_UNUSED(value);
}

static ssize_t read_blvl(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                         uint16_t len, uint16_t offset) {
    uint8_t lvl8 = battery_level;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &lvl8, sizeof(lvl8));
}

static const struct bt_gatt_cpf level_cpf = {
    .format = 0x04,        /* uint8 */
    .exponent = 0x0,
    .unit = 0x27AD,        /* Percentage */
    .name_space = 0x01,    /* Bluetooth SIG */
    .description = 0x0106, /* "main" */
};

BT_GATT_SERVICE_DEFINE(bas, BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
                       BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ, read_blvl, NULL, &battery_level),
                       BT_GATT_CCC(blvl_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CPF(&level_cpf), );

// Runs on the system work queue, so the levels are only read/written from one thread
static void update_level(struct k_work *work) {
    uint8_t level = central_level;
    if (peripheral_level > 0 && (level == 0 || peripheral_level < level)) {
        level = peripheral_level;
    }
    if (level == 0 || level > 100U || level == battery_level) {
        return;
    }

    LOG_DBG("Reporting battery level %d (central %d, peripheral %d)", level, central_level,
            peripheral_level);
    battery_level = level;
    int rc = bt_gatt_notify(NULL, &bas.attrs[1], &level, sizeof(level));
    if (rc < 0 && rc != -ENOTCONN) {
        LOG_WRN("Failed to notify battery level: %d", rc);
    }
}

static K_WORK_DEFINE(update_work, update_level);

static int battery_lowest_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev != NULL) {
        central_level = ev->state_of_charge;
        k_work_submit(&update_work);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(klein_battery_lowest, battery_lowest_listener);
ZMK_SUBSCRIPTION(klein_battery_lowest, zmk_battery_state_changed);

/* Peripheral level: read its Battery Level characteristic over the split link */

// Static: the read keeps pointing at the UUID until it completes
static const struct bt_uuid_16 batt_lvl_uuid = BT_UUID_INIT_16(BT_UUID_BAS_BATTERY_LEVEL_VAL);
static struct bt_gatt_read_params read_params;
static atomic_t read_in_flight;
static uint8_t read_result;

static void apply_read_result(struct k_work *work) {
    peripheral_level = read_result;
    update_level(work);
}

static K_WORK_DEFINE(apply_read_work, apply_read_result);

static uint8_t peripheral_level_read_cb(struct bt_conn *conn, uint8_t err,
                                        struct bt_gatt_read_params *params, const void *data,
                                        uint16_t length) {
    if (atomic_cas(&read_in_flight, 1, 0)) {
        read_result = (err == 0 && data != NULL && length >= 1) ? ((const uint8_t *)data)[0] : 0;
        k_work_submit(&apply_read_work);
    }
    return BT_GATT_ITER_STOP;
}

// The split link is the only connection where this half is the BLE central
static void find_split_conn(struct bt_conn *conn, void *data) {
    struct bt_conn **found = data;
    struct bt_conn_info info;
    if (*found == NULL && bt_conn_get_info(conn, &info) == 0 &&
        info.role == BT_CONN_ROLE_CENTRAL) {
        *found = bt_conn_ref(conn);
    }
}

static void poll_peripheral(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(poll_work, poll_peripheral);

static void poll_peripheral(struct k_work *work) {
    k_work_schedule(&poll_work, PERIPHERAL_POLL_PERIOD);

    struct bt_conn *conn = NULL;
    bt_conn_foreach(BT_CONN_TYPE_LE, find_split_conn, &conn);
    if (conn == NULL) {
        read_result = 0; // disconnected: fall back to this half's level
        k_work_submit(&apply_read_work);
        return;
    }

    if (!atomic_cas(&read_in_flight, 0, 1)) {
        bt_conn_unref(conn);
        return; // previous read still pending
    }

    read_params = (struct bt_gatt_read_params){
        .func = peripheral_level_read_cb,
        .handle_count = 0,
        .by_uuid = {.start_handle = 0x0001,
                    .end_handle = 0xffff,
                    .uuid = &batt_lvl_uuid.uuid},
    };
    int err = bt_gatt_read(conn, &read_params);
    if (err < 0) {
        LOG_DBG("Peripheral battery read failed to start: %d", err);
        atomic_set(&read_in_flight, 0);
    }
    bt_conn_unref(conn);
}

static int battery_lowest_init(void) {
    k_work_schedule(&poll_work, PERIPHERAL_POLL_FIRST);
    return 0;
}

SYS_INIT(battery_lowest_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
