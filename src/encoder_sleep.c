/*
 * Disconnect the left encoder pins before deep sleep so their pull-ups
 * can't leak current through a closed encoder contact while powered off.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define ENCODER_NODE DT_NODELABEL(left_encoder)

// Only the left half enables the encoder; on the right this compiles to nothing.
#if DT_NODE_HAS_STATUS(ENCODER_NODE, okay)

static const struct gpio_dt_spec enc_a = GPIO_DT_SPEC_GET(ENCODER_NODE, a_gpios);
static const struct gpio_dt_spec enc_b = GPIO_DT_SPEC_GET(ENCODER_NODE, b_gpios);

static void disconnect_pin(const struct gpio_dt_spec *pin) {
    gpio_pin_interrupt_configure_dt(pin, GPIO_INT_DISABLE);
    gpio_pin_configure_dt(pin, GPIO_DISCONNECTED);
}

// ZMK raises the SLEEP state synchronously right before sys_poweroff(), and the
// nRF52 keeps pin configuration in System OFF, so the pins stay disconnected
// until a key press wakes (resets) the board.
static int encoder_sleep_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev == NULL || ev->state != ZMK_ACTIVITY_SLEEP) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    LOG_INF("Disconnecting encoder pins before deep sleep");
    disconnect_pin(&enc_a);
    disconnect_pin(&enc_b);

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(klein_encoder_sleep, encoder_sleep_listener);
ZMK_SUBSCRIPTION(klein_encoder_sleep, zmk_activity_state_changed);

#endif
