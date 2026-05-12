/**
 * @file main.c
 * @brief Occupancy controller with LCD2004 – robust relay state tracking.
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * Application layer. Deterministic occupancy state machines for bulb & fan,
 * with an LCD displaying the *logical* relay state (immune to GPIO loading).
 * =============================================================================
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "lcd_i2c.h"
#include "ina219.h"

#define TAG "main"

/*===========================================================================
 * GPIO MAPPING
 *===========================================================================*/
#define PIR_GPIO                23
#define BUTTON_BULB_GPIO        32
#define RELAY_BULB_GPIO         2
#define BUTTON_FAN_GPIO         0
#define RELAY_FAN_GPIO          4

/*===========================================================================
 * TIMING CONSTANTS (ms)
 *===========================================================================*/
#define BUTTON_DEBOUNCE_MS              20
#define PIR_DEBOUNCE_MS                 50
#define PIR_CONFIRM_ON_MS               200
#define PIR_HOLD_ON_MS                  4000
#define PIR_CONFIRM_OFF_MS              200
#define PIR_MANUAL_OFF_LOCKOUT_MS       5000
#define PIR_MANUAL_ON_TIMEOUT_MS        0
#define POLL_PERIOD_MS                  10
#define LCD_UPDATE_MS                   500

/*===========================================================================
 * OCCUPANCY STATE MACHINE
 *===========================================================================*/
typedef enum {
    OCC_STATE_IDLE,
    OCC_STATE_CONFIRMING_ON,
    OCC_STATE_ON_TIMER,
    OCC_STATE_CONFIRMING_OFF
} occ_state_t;

typedef struct {
    occ_state_t state;
    TickType_t  state_entry;
    TickType_t  last_motion;
    bool        manual_on;
    TickType_t  lockout_until;
    TickType_t  manual_timeout;
    int         relay_pin;
    int         button_pin;
    TickType_t  btn_change;
    bool        btn_steady;
    bool        btn_reading;
    bool        demand;            /**< true = automation wants relay ON */
    bool        relay_on;          /**< actual relay state (active‑low: true = LOW) */
    const char *name;
} occ_controller_t;

/*===========================================================================
 * PRIVATE HELPERS
 *===========================================================================*/
static void gpio_init(void) {
    gpio_config_t io_conf = {0};
    io_conf.pin_bit_mask = (1ULL << PIR_GPIO);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << RELAY_BULB_GPIO) | (1ULL << RELAY_FAN_GPIO);
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(RELAY_BULB_GPIO, 1);
    gpio_set_level(RELAY_FAN_GPIO, 1);
}

static void button_input_init(int gpio) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

static bool button_pressed(int raw, bool *last_reading, TickType_t *last_change,
                           bool *last_steady, TickType_t now) {
    bool press = false;
    if (raw != *last_reading) *last_change = now;
    if ((now - *last_change) >= pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
        if (raw != *last_steady) {
            *last_steady = raw;
            if (raw == 0) press = true;
        }
    }
    *last_reading = raw;
    return press;
}

static void occ_reset(occ_controller_t *occ, TickType_t now, TickType_t lockout_ms) {
    occ->state = OCC_STATE_IDLE;
    occ->demand = false;
    occ->lockout_until = now + pdMS_TO_TICKS(lockout_ms);
}

static void occ_handle_button(occ_controller_t *occ, TickType_t now) {
    int raw = gpio_get_level(occ->button_pin);
    bool press = button_pressed(raw, &occ->btn_reading, &occ->btn_change,
                                &occ->btn_steady, now);
    if (!press) return;

    if (!occ->manual_on) {
        occ->manual_on = true;
        occ->manual_timeout = (PIR_MANUAL_ON_TIMEOUT_MS > 0)
                ? now + pdMS_TO_TICKS(PIR_MANUAL_ON_TIMEOUT_MS) : 0;
        ESP_LOGI(TAG, "%s ON (manual)", occ->name);
    } else {
        occ->manual_on = false;
        occ_reset(occ, now, PIR_MANUAL_OFF_LOCKOUT_MS);
        ESP_LOGI(TAG, "%s OFF (manual, lockout %dms)", occ->name, PIR_MANUAL_OFF_LOCKOUT_MS);
    }
}

static void occ_check_manual_timeout(occ_controller_t *occ, TickType_t now) {
    if (occ->manual_on && occ->manual_timeout != 0 && now >= occ->manual_timeout) {
        occ->manual_on = false;
        occ_reset(occ, now, PIR_MANUAL_OFF_LOCKOUT_MS);
        ESP_LOGI(TAG, "%s OFF (manual timeout)", occ->name);
    }
}

static void occ_state_machine(occ_controller_t *occ, bool pir_stable, TickType_t now) {
    if (occ->lockout_until != 0 && now < occ->lockout_until) {
        occ->state = OCC_STATE_IDLE;
        occ->demand = false;
        return;
    }

    switch (occ->state) {
    case OCC_STATE_IDLE:
        if (pir_stable) {
            occ->state = OCC_STATE_CONFIRMING_ON;
            occ->state_entry = now;
        }
        break;
    case OCC_STATE_CONFIRMING_ON:
        if (!pir_stable) {
            occ->state = OCC_STATE_IDLE;
        } else if ((now - occ->state_entry) >= pdMS_TO_TICKS(PIR_CONFIRM_ON_MS)) {
            occ->demand = true;
            occ->last_motion = now;
            occ->state = OCC_STATE_ON_TIMER;
        }
        break;
    case OCC_STATE_ON_TIMER:
        if (pir_stable) {
            occ->last_motion = now;
        } else if ((now - occ->last_motion) >= pdMS_TO_TICKS(PIR_HOLD_ON_MS)) {
            occ->demand = false;
            occ->state = OCC_STATE_CONFIRMING_OFF;
            occ->state_entry = now;
        }
        break;
    case OCC_STATE_CONFIRMING_OFF:
        if (pir_stable) {
            occ->demand = true;
            occ->last_motion = now;
            occ->state = OCC_STATE_ON_TIMER;
        } else if ((now - occ->state_entry) >= pdMS_TO_TICKS(PIR_CONFIRM_OFF_MS)) {
            occ->demand = false;
            occ->state = OCC_STATE_IDLE;
        }
        break;
    }
}

/**
 * @brief Update the physical relay and store its logical state.
 */
static void occ_update_relay(occ_controller_t *occ) {
    bool on;
    if (occ->manual_on) {
        on = true;
    } else if (occ->lockout_until != 0 && xTaskGetTickCount() < occ->lockout_until) {
        on = false;
    } else {
        on = occ->demand;
    }
    // Write to relay (active low: 0 = ON)
    gpio_set_level(occ->relay_pin, on ? 0 : 1);
    // Store the logical state (what the relay is supposed to be)
    occ->relay_on = on;
}

/*===========================================================================
 * LCD2004 WRAPPER (your proven driver)
 *===========================================================================*/
static lcd_handle_t *lcd = NULL;

static void lcd_setup(void) {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    lcd_config_t lcd_cfg = {
        .i2c_port = I2C_NUM_0,
        .i2c_addr = 0x27,
        .rows = 4,
        .cols = 20,
        .backlight_enable = true,
        .i2c_timeout_ms = 1000,
        .cmd_delay_us = 50
    };
    lcd = lcd_i2c_init(&lcd_cfg);
    if (lcd) {
        lcd_clear(lcd);
        lcd_print_str(lcd, "System ready.");
    } else {
        ESP_LOGE(TAG, "LCD init failed");
    }
}

/**
 * @brief Update LCD with LOGICAL relay states (reliable).
 */
static void lcd_status_update(occ_controller_t *bulb, occ_controller_t *fan,
                              bool motion_now, const ina219_data_t *ina) {
    if (!lcd) return;

    // Line 0: Bulb
    lcd_set_cursor(lcd, 0, 0);
    lcd_printf(lcd, "BULB: %s  %s",
               bulb->relay_on ? "ON " : "OFF",
               bulb->manual_on ? "(man)" : "(auto)");

    // Line 1: Fan
    lcd_set_cursor(lcd, 1, 0);
    lcd_printf(lcd, "FAN:  %s  %s",
               fan->relay_on ? "ON " : "OFF",
               fan->manual_on ? "(man)" : "(auto)");

    // Line 2: PIR
    lcd_set_cursor(lcd, 2, 0);
    lcd_printf(lcd, "PIR: %s", motion_now ? "MOTION" : "NO MOTION");

    // Line 3: INA219 readings
    if (ina) {
        lcd_set_cursor(lcd, 3, 0);
        lcd_printf(lcd, "V:%05.1fV I:%05.2fA", ina->bus_voltage, ina->current);
    } else {
        lcd_set_cursor(lcd, 3, 0);
        lcd_print_str(lcd, "INA219 offline");
    }
}

/**
 * @brief Quick I2C scanner for legacy I2C driver.
 *        Prints every device address that responds.
 */
static void i2c_scan(void) {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Device found at 0x%02X", addr);
        }
    }
}

/*===========================================================================
 * APP MAIN
 *===========================================================================*/
void app_main(void) {
    gpio_init();
    button_input_init(BUTTON_BULB_GPIO);
    button_input_init(BUTTON_FAN_GPIO);
    lcd_setup();

    /* --- INA219 initialisation --- */
    ina219_config_t ina_cfg = {
        .i2c_addr = 0x44,
        .shunt_resistance = 0.1f,   // typical for INA219 breakout
        .max_current = 3.2f
    };
    
    // i2c_scan(); // Uncomment to scan for I2C devices on the bus
    if (ina219_init(&ina_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "INA219 init failed");
    } else {
        ESP_LOGI(TAG, "INA219 ready");
    }

    occ_controller_t bulb = {
        .state = OCC_STATE_IDLE, .state_entry = 0, .last_motion = 0,
        .manual_on = false, .lockout_until = 0, .manual_timeout = 0,
        .relay_pin = RELAY_BULB_GPIO, .button_pin = BUTTON_BULB_GPIO,
        .btn_change = 0, .btn_steady = true, .btn_reading = true,
        .demand = false, .relay_on = false, .name = "Bulb"
    };
    occ_controller_t fan = {
        .state = OCC_STATE_IDLE, .state_entry = 0, .last_motion = 0,
        .manual_on = false, .lockout_until = 0, .manual_timeout = 0,
        .relay_pin = RELAY_FAN_GPIO, .button_pin = BUTTON_FAN_GPIO,
        .btn_change = 0, .btn_steady = true, .btn_reading = true,
        .demand = false, .relay_on = false, .name = "Fan"
    };

    TickType_t pir_debounce_time = 0;
    bool pir_stable = false, pir_last_reading = false;
    TickType_t last_lcd = 0;

    ESP_LOGI(TAG, "System ready");

    while (1) {
        TickType_t now = xTaskGetTickCount();

        /* PIR raw debounce */
        bool pir_raw = gpio_get_level(PIR_GPIO);
        if (pir_raw != pir_last_reading) pir_debounce_time = now;
        if ((now - pir_debounce_time) >= pdMS_TO_TICKS(PIR_DEBOUNCE_MS))
            pir_stable = pir_raw;
        pir_last_reading = pir_raw;

        /* Bulb controller */
        occ_handle_button(&bulb, now);
        occ_check_manual_timeout(&bulb, now);
        occ_state_machine(&bulb, pir_stable, now);
        occ_update_relay(&bulb);

        /* Fan controller */
        occ_handle_button(&fan, now);
        occ_check_manual_timeout(&fan, now);
        occ_state_machine(&fan, pir_stable, now);
        occ_update_relay(&fan);

        static ina219_data_t ina_data;
        static bool ina_ok = false;

        /* LCD refresh using logical states */
        #if 0

            // here the sensor not connected, so just show "offline" message
            if (lcd && (now - last_lcd) >= pdMS_TO_TICKS(LCD_UPDATE_MS)) {
                lcd_status_update(&bulb, &fan, pir_stable, NULL); /**< TODO: Pass INA219 data */
                last_lcd = now;
            }

        #else

            // In a real system, reading INA219 here and passing the data to lcd_status_update.
            if (lcd && (now - last_lcd) >= pdMS_TO_TICKS(LCD_UPDATE_MS)) {
                ina_ok = (ina219_read(&ina_data) == ESP_OK);
                lcd_status_update(&bulb, &fan, pir_stable, ina_ok ? &ina_data : NULL);
                last_lcd = now;
            }
        #endif

        ina_ok = false;
        if ((now - last_lcd) >= pdMS_TO_TICKS(LCD_UPDATE_MS)) {
            // Attempt to read INA219 (will fail gracefully if not initialised)
            if (ina219_read(&ina_data) == ESP_OK) {
                ina_ok = true;
            } else {
                ina_ok = false;
            }
            if (lcd) lcd_status_update(&bulb, &fan, pir_stable, ina_ok ? &ina_data : NULL);
            last_lcd = now;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}