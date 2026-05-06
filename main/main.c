#if 1


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "relay"

/* --- Pin assignments --- */
#define BUTTON_GPIO         0       /**< BOOT button, active low */
#define RELAY2_FERN         4       /**< Relay 1 (manual toggle), active low */
#define PIR_GPIO            23      /**< PIR motion sensor, active high */
#define RELAY2_BULB         2       /**< Relay 2 (PIR controlled), active low */

#define DEBOUNCE_MS         50      /**< Debounce time in ms */
#define POLL_PERIOD_MS      10      /**< Loop poll interval */

void app_main(void)
{
    /* === GPIO configuration === */
    gpio_config_t io_conf = {0};

    // Button input with internal pull-up (pressed = LOW)
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // PIR input (external pull-down usually on module, but we add a weak internal pull-down for safety)
    io_conf.pin_bit_mask = (1ULL << PIR_GPIO);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Relay outputs (active low, initial OFF = HIGH)
    io_conf.pin_bit_mask = (1ULL << RELAY2_FERN) | (1ULL << RELAY2_BULB);
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(RELAY2_FERN, 1);   // OFF
    gpio_set_level(RELAY2_BULB, 1);   // OFF

    /* === State variables for button debounce === */
    bool relay1_on  = false;
    TickType_t btn_last_change = 0;
    bool btn_last_steady = 1;          // button not pressed = HIGH
    bool btn_last_reading = 1;

    /* === State variables for PIR debounce === */
    bool relay2_on  = false;
    TickType_t pir_last_change = 0;
    bool pir_last_steady = 0;          // assume no motion
    bool pir_last_reading = 0;

    ESP_LOGI(TAG, "System ready. Button toggles Relay1 (GPIO%d), PIR controls Relay2 (GPIO%d).", RELAY2_FERN, RELAY2_BULB);

    while (1) {
        TickType_t now = xTaskGetTickCount();

        /* --- Button handling (toggle) --- */
        bool btn_raw = gpio_get_level(BUTTON_GPIO);
        if (btn_raw != btn_last_reading) {
            btn_last_change = now;
        }
        if ((now - btn_last_change) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
            if (btn_raw != btn_last_steady) {
                btn_last_steady = btn_raw;
                if (btn_raw == 0) {   // press (falling edge)
                    relay1_on = !relay1_on;
                    gpio_set_level(RELAY2_FERN, relay1_on ? 0 : 1);
                    ESP_LOGI(TAG, "Relay1 %s", relay1_on ? "ON" : "OFF");
                }
            }
        }
        btn_last_reading = btn_raw;

        /* --- PIR handling (automatic) --- */
        bool pir_raw = gpio_get_level(PIR_GPIO);
        if (pir_raw != pir_last_reading) {
            pir_last_change = now;
        }
        if ((now - pir_last_change) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
            if (pir_raw != pir_last_steady) {
                pir_last_steady = pir_raw;
                if (pir_raw) {          // motion detected
                    relay2_on = true;
                    gpio_set_level(RELAY2_BULB, 0);
                    ESP_LOGI(TAG, "Relay2 ON (motion)");
                } else {                // no motion
                    relay2_on = false;
                    gpio_set_level(RELAY2_BULB, 1);
                    ESP_LOGI(TAG, "Relay2 OFF (no motion)");
                }
            }
        }
        pir_last_reading = pir_raw;

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}



#else

#include "pzem004t_v1.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"

#include "pzem004t_v1.h"

#define PZEM_UART_NUM   UART_NUM_1
#define PZEM_TX_PIN     17
#define PZEM_RX_PIN     16

void app_main(void)
{
    esp_err_t err = pzem_init(PZEM_UART_NUM, PZEM_TX_PIN, PZEM_RX_PIN);
    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "PZEM init failed");
        return;
    }

    pzem_data_t data;
    while (1) {
        if (pzem_read_all(PZEM_UART_NUM, &data) == ESP_OK) {
            // Use data here (post event, update context, etc.)
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}





#endif