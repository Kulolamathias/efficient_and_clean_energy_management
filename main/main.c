/**
 * @file main.c
 * @brief Application entry point – relay control for fan and bulb.
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * This module lives at the application layer (main component). It directly
 * reads GPIOs and drives relays. In a full layered architecture the relay
 * actions would be commands issued by a core service, but for this
 * self‑contained demo the logic is kept local and deterministic.
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
#define BUTTON_GPIO         0       /**< BOOT button (active low, internal pull‑up) */
#define RELAY_FAN_GPIO      4       /**< Fan relay (active low) */
#define PIR_GPIO            23      /**< PIR motion sensor (active high) */
#define RELAY_BULB_GPIO     2       /**< Bulb relay (active low) */

/*===========================================================================
 * TIMING CONFIGURATION (all values in milliseconds)
 *===========================================================================*/
#define BUTTON_DEBOUNCE_MS      20      /**< Button debounce period */
#define PIR_DEBOUNCE_MS         50      /**< Raw PIR signal debounce */
#define PIR_CONFIRM_ON_MS       200     /**< Motion must persist this long before turning ON */
#define PIR_HOLD_ON_MS          4000   /**< Bulb stays ON at least this long after last motion */
#define PIR_CONFIRM_OFF_MS      200     /**< Absence must persist this long before turning OFF */

#define POLL_PERIOD_MS          10      /**< Main loop polling interval */

/*===========================================================================
 * PIR STATE MACHINE ENUM
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
 * PRIVATE HELPER: initialise GPIOs
 *===========================================================================*/
static void gpio_init(void)
{
    gpio_config_t io_conf = {0};

    /* Button input (pressed = LOW) */
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    /* PIR input (active HIGH, weak internal pull‑down for safety) */
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

/*===========================================================================
 * APPLICATION ENTRY
 *===========================================================================*/
void app_main(void)
{
    gpio_init();

    /* ----- Button (fan toggle) state variables ----- */
    bool        fan_on             = false;
    TickType_t  btn_last_change    = 0;
    bool        btn_last_steady    = true;   // button not pressed
    bool        btn_last_reading   = true;

    /* ----- PIR state machine instance ----- */
    pir_fsm_t pir = {
        .state        = PIR_STATE_IDLE,
        .state_entry  = 0,
        .last_motion  = 0
    };

    /* ----- PIR debounce variables (raw signal) ----- */
    TickType_t  pir_debounce_time = 0;
    bool        pir_stable        = false;   // debounced, stable level
    bool        pir_last_reading  = false;

    ESP_LOGI(TAG, "System ready. Button toggles fan (GPIO%d), PIR controls bulb (GPIO%d).",
             RELAY_FAN_GPIO, RELAY_BULB_GPIO);

    while (1) {
        TickType_t now = xTaskGetTickCount();

        /*---------------------------------------------------------------------
         * BUTTON HANDLING – simple debounced toggle
         *---------------------------------------------------------------------*/
        bool btn_raw = gpio_get_level(BUTTON_GPIO);
        if (btn_raw != btn_last_reading) {
            btn_last_change = now;
        }
        if ((now - btn_last_change) >= pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
            if (btn_raw != btn_last_steady) {
                btn_last_steady = btn_raw;
                if (btn_raw == 0) {      // falling edge = press
                    fan_on = !fan_on;
                    gpio_set_level(RELAY_FAN_GPIO, fan_on ? 0 : 1);
                    ESP_LOGI(TAG, "Fan %s", fan_on ? "ON" : "OFF");
                }
            }
        }
        btn_last_reading = btn_raw;

        /*---------------------------------------------------------------------
         * PIR RAW SIGNAL DEBOUNCE (independent of state machine)
         *---------------------------------------------------------------------*/
        bool pir_raw = gpio_get_level(PIR_GPIO);
        if (pir_raw != pir_last_reading) {
            pir_debounce_time = now;
        }
        if ((now - pir_debounce_time) >= pdMS_TO_TICKS(PIR_DEBOUNCE_MS)) {
            pir_stable = pir_raw;   // level has been stable long enough
        }
        pir_last_reading = pir_raw;

        /*---------------------------------------------------------------------
         * PIR STATE MACHINE
         *---------------------------------------------------------------------*/
        switch (pir.state) {

        case PIR_STATE_IDLE:
            if (pir_stable) {
                // First sign of motion -> start confirmation timer
                pir.state = PIR_STATE_CONFIRMING_ON;
                pir.state_entry = now;
                ESP_LOGD(TAG, "PIR: motion detected, confirming...");
            }
            break;

        case PIR_STATE_CONFIRMING_ON:
            if (!pir_stable) {
                // Motion disappeared before confirmation -> noise, go back
                pir.state = PIR_STATE_IDLE;
                ESP_LOGD(TAG, "PIR: false trigger, returning to idle");
            } else if ((now - pir.state_entry) >= pdMS_TO_TICKS(PIR_CONFIRM_ON_MS)) {
                // Motion confirmed -> turn bulb ON
                gpio_set_level(RELAY_BULB_GPIO, 0);
                pir.last_motion = now;
                pir.state = PIR_STATE_ON_TIMER;
                ESP_LOGI(TAG, "Bulb ON (motion confirmed)");
            }
            break;

        case PIR_STATE_ON_TIMER:
            if (pir_stable) {
                // Motion still present -> reset hold timer
                pir.last_motion = now;
            } else if ((now - pir.last_motion) >= pdMS_TO_TICKS(PIR_HOLD_ON_MS)) {
                // No motion for hold time -> start off‑confirmation
                gpio_set_level(RELAY_BULB_GPIO, 1);    // turn OFF tentatively
                pir.state = PIR_STATE_CONFIRMING_OFF;
                pir.state_entry = now;
                ESP_LOGD(TAG, "PIR: hold expired, confirming absence...");
            }
            break;

        case PIR_STATE_CONFIRMING_OFF:
            if (pir_stable) {
                // Motion returned during off‑confirmation -> cancel, go back ON
                gpio_set_level(RELAY_BULB_GPIO, 0);
                pir.last_motion = now;
                pir.state = PIR_STATE_ON_TIMER;
                ESP_LOGI(TAG, "Bulb ON (motion returned)");
            } else if ((now - pir.state_entry) >= pdMS_TO_TICKS(PIR_CONFIRM_OFF_MS)) {
                // Confirmed absence -> final OFF
                pir.state = PIR_STATE_IDLE;
                ESP_LOGI(TAG, "Bulb OFF (absence confirmed)");
            }
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}