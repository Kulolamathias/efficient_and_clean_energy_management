/**
 * @file main.c
 * @brief Application entry point – commercial‑style occupancy lighting & fan control.
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * Application layer. Reads physical inputs (buttons, PIR) and drives relay
 * outputs. The logic is contained in a deterministic finite‑state machine and
 * manual‑override manager. All timing values are configurable at the top.
 * =============================================================================
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "relay"

/*===========================================================================
 * GPIO PIN ASSIGNMENTS
 *===========================================================================*/
#define BUTTON_FAN_GPIO     0       /**< Fan toggle button (active low, internal pull‑up) */
#define RELAY_FAN_GPIO      4       /**< Fan relay (active low) */
#define BUTTON_BULB_GPIO    32      /**< Bulb manual toggle button (active low, internal pull‑up) */
#define PIR_GPIO            23      /**< PIR motion sensor (active high) */
#define RELAY_BULB_GPIO     2       /**< Bulb relay (active low) */

/*===========================================================================
 * TIMING CONFIGURATION (all values in milliseconds)
 *===========================================================================*/
#define BUTTON_DEBOUNCE_MS          20      /**< Button debounce period */
#define PIR_DEBOUNCE_MS             50      /**< Raw PIR signal debounce */
#define PIR_CONFIRM_ON_MS           200     /**< Motion must persist this long before turning ON */
#define PIR_HOLD_ON_MS              4000    /**< Bulb stays ON at least this long after last motion */
#define PIR_CONFIRM_OFF_MS          200     /**< Absence must persist this long before turning OFF */
#define PIR_MANUAL_OFF_LOCKOUT_MS   5000    /**< After manual OFF, PIR is frozen for this long */
#define PIR_MANUAL_ON_TIMEOUT_MS    0       /**< 0 = manual ON stays forever; else auto‑return after this */

#define POLL_PERIOD_MS              10      /**< Main loop polling interval */

/*===========================================================================
 * PIR STATE MACHINE
 *===========================================================================*/
typedef enum {
    PIR_STATE_IDLE,            /**< No motion, bulb off */
    PIR_STATE_CONFIRMING_ON,   /**< Motion detected, waiting for confirmation */
    PIR_STATE_ON_TIMER,        /**< Bulb on, hold‑timer running */
    PIR_STATE_CONFIRMING_OFF   /**< Hold‑timer expired, checking if room is truly empty */
} pir_state_t;

typedef struct {
    pir_state_t state;         /**< Current state of the PIR state machine */
    TickType_t  state_entry;   /**< Tick when current state was entered */
    TickType_t  last_motion;   /**< Last time motion was confirmed (state ON_TIMER) */
} pir_fsm_t;

/*===========================================================================
 * PRIVATE HELPERS
 *===========================================================================*/
/**
 * @brief Configure all GPIOs (inputs with pull‑ups, relays as outputs).
 */
static void gpio_init(void)
{
    gpio_config_t io_conf = {0};

    /* Buttons (pressed = LOW) */
    io_conf.pin_bit_mask = (1ULL << BUTTON_FAN_GPIO) | (1ULL << BUTTON_BULB_GPIO);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    /* PIR input (active HIGH, weak internal pull‑down) */
    io_conf.pin_bit_mask = (1ULL << PIR_GPIO);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    /* Relay outputs (active low, initial OFF = HIGH) */
    io_conf.pin_bit_mask = (1ULL << RELAY_FAN_GPIO) | (1ULL << RELAY_BULB_GPIO);
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(RELAY_FAN_GPIO, 1);
    gpio_set_level(RELAY_BULB_GPIO, 1);
}

/**
 * @brief Simple debounced button state reader.
 *
 * @param raw           Current raw GPIO level.
 * @param last_reading  Pointer to last raw reading.
 * @param last_change   Pointer to tick of last level change.
 * @param last_steady   Pointer to the debounced stable level.
 * @param now           Current tick count.
 * @return true if the debounced level just changed (edge detected).
 */
static bool button_debounce(int raw, bool *last_reading, TickType_t *last_change,
                            bool *last_steady, TickType_t now)
{
    bool edge = false;
    if (raw != *last_reading) {
        *last_change = now;
    }
    if ((now - *last_change) >= pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
        if (raw != *last_steady) {
            *last_steady = raw;
            edge = true;    // debounced state changed
        }
    }
    *last_reading = raw;
    return edge;
}

/*===========================================================================
 * APPLICATION ENTRY
 *===========================================================================*/
void app_main(void)
{
    gpio_init();

    /* --- Fan button state --- */
    bool        fan_on            = false;
    TickType_t  fan_btn_change    = 0;
    bool        fan_btn_steady    = true;   // not pressed
    bool        fan_btn_reading   = true;

    /* --- Bulb button state (manual override) --- */
    TickType_t  bulb_btn_change    = 0;
    bool        bulb_btn_steady    = true;
    bool        bulb_btn_reading   = true;

    /* --- Manual control flags & timers --- */
    bool        bulb_manual_on    = false;    // true = button forced bulb ON
    TickType_t  lockout_until     = 0;        // PIR frozen until this tick (after manual off)
    TickType_t  manual_on_timeout = 0;        // if >0, auto‑return to auto at this tick

    /* --- PIR state machine instance --- */
    pir_fsm_t pir = {
        .state        = PIR_STATE_IDLE,
        .state_entry  = 0,
        .last_motion  = 0
    };
    bool pir_on_request = false;   // PIR FSM demand

    /* --- PIR raw signal debounce --- */
    TickType_t  pir_debounce_time = 0;
    bool        pir_stable        = false;
    bool        pir_reading       = false;

    ESP_LOGI(TAG, "System ready. Fan btn: GPIO%d, Bulb btn: GPIO%d, PIR: GPIO%d",
             BUTTON_FAN_GPIO, BUTTON_BULB_GPIO, PIR_GPIO);

    while (1) {
        TickType_t now = xTaskGetTickCount();

        /*------------------------------------------------------------------
         * FAN BUTTON – simple toggle
         *------------------------------------------------------------------*/
        int fan_btn_raw = gpio_get_level(BUTTON_FAN_GPIO);
        bool fan_edge = button_debounce(fan_btn_raw, &fan_btn_reading,
                                        &fan_btn_change, &fan_btn_steady, now);
        if (fan_edge && fan_btn_steady == 0) {   // falling edge (press)
            fan_on = !fan_on;
            gpio_set_level(RELAY_FAN_GPIO, fan_on ? 0 : 1);
            ESP_LOGI(TAG, "Fan %s", fan_on ? "ON" : "OFF");
        }

        /*------------------------------------------------------------------
         * BULB BUTTON – manual override toggle
         *------------------------------------------------------------------*/
        int bulb_btn_raw = gpio_get_level(BUTTON_BULB_GPIO);
        bool bulb_edge = button_debounce(bulb_btn_raw, &bulb_btn_reading,
                                         &bulb_btn_change, &bulb_btn_steady, now);
        if (bulb_edge && bulb_btn_steady == 0) {   // press
            if (!bulb_manual_on) {
                // Toggle ON
                bulb_manual_on = true;
                // If manual‑on timeout is enabled, schedule auto‑return
                if (PIR_MANUAL_ON_TIMEOUT_MS > 0) {
                    manual_on_timeout = now + pdMS_TO_TICKS(PIR_MANUAL_ON_TIMEOUT_MS);
                } else {
                    manual_on_timeout = 0;   // never auto‑return
                }
                ESP_LOGI(TAG, "Bulb ON (manual override)");
            } else {
                // Toggle OFF
                bulb_manual_on = false;
                // Lockout PIR for a while so you can leave without re‑trigger
                lockout_until = now + pdMS_TO_TICKS(PIR_MANUAL_OFF_LOCKOUT_MS);
                // Reset PIR state machine to idle (ignore any pending motion)
                pir.state = PIR_STATE_IDLE;
                pir_on_request = false;
                ESP_LOGI(TAG, "Bulb OFF (manual override, lockout %d ms)",
                         PIR_MANUAL_OFF_LOCKOUT_MS);
            }
        }

        /*------------------------------------------------------------------
         * MANUAL‑ON AUTO‑RETURN (if configured)
         *------------------------------------------------------------------*/
        if (bulb_manual_on && manual_on_timeout != 0 && now >= manual_on_timeout) {
            bulb_manual_on = false;
            // After auto‑return, lockout the PIR briefly (same as manual off)
            lockout_until = now + pdMS_TO_TICKS(PIR_MANUAL_OFF_LOCKOUT_MS);
            pir.state = PIR_STATE_IDLE;
            pir_on_request = false;
            ESP_LOGI(TAG, "Bulb OFF (manual on timeout expired)");
        }

        /*------------------------------------------------------------------
         * PIR RAW SIGNAL DEBOUNCE
         *------------------------------------------------------------------*/
        bool pir_raw = gpio_get_level(PIR_GPIO);
        if (pir_raw != pir_reading) {
            pir_debounce_time = now;
        }
        if ((now - pir_debounce_time) >= pdMS_TO_TICKS(PIR_DEBOUNCE_MS)) {
            pir_stable = pir_raw;   // stable reading
        }
        pir_reading = pir_raw;

        /*------------------------------------------------------------------
         * PIR STATE MACHINE – only active when NOT in lockout
         *------------------------------------------------------------------*/
        if (lockout_until == 0 || now >= lockout_until) {
            // Normal PIR operation
            switch (pir.state) {

            case PIR_STATE_IDLE:
                if (pir_stable) {
                    pir.state = PIR_STATE_CONFIRMING_ON;
                    pir.state_entry = now;
                    ESP_LOGD(TAG, "PIR: motion detected, confirming...");
                }
                break;

            case PIR_STATE_CONFIRMING_ON:
                if (!pir_stable) {
                    pir.state = PIR_STATE_IDLE;
                    ESP_LOGD(TAG, "PIR: false trigger, returning to idle");
                } else if ((now - pir.state_entry) >= pdMS_TO_TICKS(PIR_CONFIRM_ON_MS)) {
                    pir_on_request = true;   // demand light ON
                    pir.last_motion = now;
                    pir.state = PIR_STATE_ON_TIMER;
                    ESP_LOGI(TAG, "Bulb ON (motion confirmed)");
                }
                break;

            case PIR_STATE_ON_TIMER:
                if (pir_stable) {
                    pir.last_motion = now;   // refresh hold timer
                } else if ((now - pir.last_motion) >= pdMS_TO_TICKS(PIR_HOLD_ON_MS)) {
                    pir_on_request = false;  // demand light OFF
                    pir.state = PIR_STATE_CONFIRMING_OFF;
                    pir.state_entry = now;
                    ESP_LOGD(TAG, "PIR: hold expired, confirming absence...");
                }
                break;

            case PIR_STATE_CONFIRMING_OFF:
                if (pir_stable) {
                    // Motion returned -> go straight back to ON
                    pir_on_request = true;
                    pir.last_motion = now;
                    pir.state = PIR_STATE_ON_TIMER;
                    ESP_LOGI(TAG, "Bulb ON (motion returned)");
                } else if ((now - pir.state_entry) >= pdMS_TO_TICKS(PIR_CONFIRM_OFF_MS)) {
                    pir.state = PIR_STATE_IDLE;
                    ESP_LOGI(TAG, "Bulb OFF (absence confirmed)");
                }
                break;
            }
        } else {
            // Lockout active: keep PIR idle and demand off
            pir_on_request = false;
            if (pir.state != PIR_STATE_IDLE) {
                pir.state = PIR_STATE_IDLE;   // ensure it's idle
            }
        }

        /*------------------------------------------------------------------
         * FINAL RELAY CONTROL – combine manual and automatic demands
         *------------------------------------------------------------------*/
        bool bulb_on;
        if (bulb_manual_on) {
            bulb_on = true;                     // manual override
        } else if (lockout_until > now) {
            bulb_on = false;                    // forced off by lockout
        } else {
            bulb_on = pir_on_request;           // PIR demand
        }
        gpio_set_level(RELAY_BULB_GPIO, bulb_on ? 0 : 1);

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}