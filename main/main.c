/**
 * @file main.c
 * @brief Occupancy controller + INA219 + GSM alerts.
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * Application layer. Combines deterministic occupancy state machines,
 * an INA219 current monitor, and a GSM module for event notifications.
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
#include "gsm_sim800.h"

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
#define GSM_CHECK_MS                    5000

/*===========================================================================
 * GSM CONFIGURATION
 *===========================================================================*/
#define GSM_TX_PIN              17
#define GSM_RX_PIN              16
#define GSM_UART_NUM            UART_NUM_2
#define GSM_BAUD_RATE           9600

#define NOTIFY_PHONE            "+255688173415"   // change to your number
#define SMS_PASSWORD            "SECRET123"

/*===========================================================================
 * OCCUPANCY STATE MACHINE (identical to working version)
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
    bool        demand;
    bool        relay_on;
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
        // Notify via SMS
        char msg[64];
        snprintf(msg, sizeof(msg), "%s manually turned ON", occ->name);
        gsm_send_sms_async(NOTIFY_PHONE, msg);
    } else {
        occ->manual_on = false;
        occ_reset(occ, now, PIR_MANUAL_OFF_LOCKOUT_MS);
        ESP_LOGI(TAG, "%s OFF (manual, lockout %dms)", occ->name, PIR_MANUAL_OFF_LOCKOUT_MS);
        char msg[64];
        snprintf(msg, sizeof(msg), "%s manually turned OFF", occ->name);
        gsm_send_sms_async(NOTIFY_PHONE, msg);
    }
}

static void occ_check_manual_timeout(occ_controller_t *occ, TickType_t now) {
    if (occ->manual_on && occ->manual_timeout != 0 && now >= occ->manual_timeout) {
        occ->manual_on = false;
        occ_reset(occ, now, PIR_MANUAL_OFF_LOCKOUT_MS);
        ESP_LOGI(TAG, "%s OFF (manual timeout)", occ->name);
        char msg[64];
        snprintf(msg, sizeof(msg), "%s auto OFF (manual timeout)", occ->name);
        gsm_send_sms_async(NOTIFY_PHONE, msg);
    }
}

static void occ_state_machine(occ_controller_t *occ, bool pir_stable, TickType_t now) {
    if (occ->lockout_until != 0 && now < occ->lockout_until) {
        occ->state = OCC_STATE_IDLE;
        occ->demand = false;
        return;
    }

    bool old_demand = occ->demand;
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

    /* Send SMS on automatic state change */
    if (old_demand != occ->demand && !occ->manual_on) {
        char msg[64];
        snprintf(msg, sizeof(msg), "%s %s (auto)", occ->name, occ->demand ? "ON" : "OFF");
        gsm_send_sms_async(NOTIFY_PHONE, msg);
    }
}

static void occ_update_relay(occ_controller_t *occ) {
    bool on;
    if (occ->manual_on) {
        on = true;
    } else if (occ->lockout_until != 0 && xTaskGetTickCount() < occ->lockout_until) {
        on = false;
    } else {
        on = occ->demand;
    }
    gpio_set_level(occ->relay_pin, on ? 0 : 1);
    occ->relay_on = on;
}

/*===========================================================================
 * LCD2004 WRAPPER
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

static void lcd_status_update(occ_controller_t *bulb, occ_controller_t *fan,
                              bool motion_now, const ina219_data_t *ina) {
    if (!lcd) return;

    lcd_set_cursor(lcd, 0, 0);
    lcd_printf(lcd, "BULB: %s  %s", bulb->relay_on ? "ON " : "OFF",
               bulb->manual_on ? "(man)" : "(auto)");
    lcd_set_cursor(lcd, 1, 0);
    lcd_printf(lcd, "FAN:  %s  %s", fan->relay_on ? "ON " : "OFF",
               fan->manual_on ? "(man)" : "(auto)");
    lcd_set_cursor(lcd, 2, 0);
    lcd_printf(lcd, "PIR: %s", motion_now ? "MOTION" : "NO MOTION");
    lcd_set_cursor(lcd, 3, 0);
    if (ina) {
        lcd_printf(lcd, "V:%05.1fV I:%05.2fA", ina->bus_voltage, ina->current);
    } else {
        lcd_printf(lcd, "INA219 offline");
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

    /* INA219 (addr 0x44 as scanned) */
    ina219_config_t ina_cfg = {
        .i2c_addr = 0x44,
        .shunt_resistance = 0.1f,
        .max_current = 3.2f
    };
    bool ina_ok = (ina219_init(&ina_cfg) == ESP_OK);
    if (!ina_ok) ESP_LOGE(TAG, "INA219 init failed");

    /* GSM SIM800 */
    gsm_config_t gsm_cfg = {
        .uart_port   = GSM_UART_NUM,
        .tx_pin      = GSM_TX_PIN,
        .rx_pin      = GSM_RX_PIN,
        .baud_rate   = GSM_BAUD_RATE,
        .buf_size    = 2048,
        .timeout_ms  = 30000,
        .retry_count = 2,
    };
    if (gsm_init(&gsm_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "GSM init failed – SMS alerts disabled");
    } else {
        ESP_LOGI(TAG, "GSM ready, sending startup message");
        vTaskDelay(pdMS_TO_TICKS(2000));
        gsm_send_sms(NOTIFY_PHONE, "System started. Bulb & Fan automation active.", 35000);
    }

    /* Occupancy controllers */
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
    TickType_t last_lcd = 0, last_gsm_check = 0;

    while (1) {
        TickType_t now = xTaskGetTickCount();

        bool pir_raw = gpio_get_level(PIR_GPIO);
        if (pir_raw != pir_last_reading) pir_debounce_time = now;
        if ((now - pir_debounce_time) >= pdMS_TO_TICKS(PIR_DEBOUNCE_MS))
            pir_stable = pir_raw;
        pir_last_reading = pir_raw;

        occ_handle_button(&bulb, now);
        occ_check_manual_timeout(&bulb, now);
        occ_state_machine(&bulb, pir_stable, now);
        occ_update_relay(&bulb);

        occ_handle_button(&fan, now);
        occ_check_manual_timeout(&fan, now);
        occ_state_machine(&fan, pir_stable, now);
        occ_update_relay(&fan);

        if (lcd && (now - last_lcd) >= pdMS_TO_TICKS(LCD_UPDATE_MS)) {
            ina219_data_t ina_data;
            bool ina_read_ok = (ina_ok && ina219_read(&ina_data) == ESP_OK);
            lcd_status_update(&bulb, &fan, pir_stable,
                             ina_read_ok ? &ina_data : NULL);
            last_lcd = now;
        }

        /* GSM SMS check & processing */
        if (gsm_is_ready() && (now - last_gsm_check) >= pdMS_TO_TICKS(GSM_CHECK_MS)) {
            gsm_check_sms(true);
            gsm_process_received_sms();
            last_gsm_check = now;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}