/*
 * Battery Service that reports the lower of the central's and the peripheral's
 * battery level, since most hosts only display one level per keyboard.
 *
 * Mirrors Zephyr's subsys/bluetooth/services/bas.c (same service name and
 * attribute layout, so GATT handles don't move); build with CONFIG_BT_BAS=n.
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// 0 = unknown. The central also reports 0 for a peripheral when it disconnects.
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

static void update_level(void) {
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

static int battery_lowest_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *central_ev = as_zmk_battery_state_changed(eh);
    if (central_ev != NULL) {
        central_level = central_ev->state_of_charge;
        update_level();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_peripheral_battery_state_changed *periph_ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (periph_ev != NULL) {
        peripheral_level = periph_ev->state_of_charge;
        update_level();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(klein_battery_lowest, battery_lowest_listener);
ZMK_SUBSCRIPTION(klein_battery_lowest, zmk_battery_state_changed);
ZMK_SUBSCRIPTION(klein_battery_lowest, zmk_peripheral_battery_state_changed);
