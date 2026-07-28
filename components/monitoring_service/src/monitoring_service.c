/**
 * @file monitoring_service.c
 * @brief Deterministic scheduling for energy acquisition and LCD rendering.
 * @author Matthithyahu
 *
 * One task owns all application-level I2C calls. This avoids cross-device bus
 * contention while keeping any I2C timeout isolated from relay/button control.
 */

#include "monitoring_service.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_types.h"
#include "control_service.h"
#include "display_service.h"
#include "driver/i2c.h"
#include "energy_service.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "monitor";

/** @brief Monitoring singleton state. */
typedef struct {
  TaskHandle_t task;
  bool display_available;
  bool initialized;
} monitoring_context_t;

static monitoring_context_t s_monitor = {0};

/** @brief Return monotonic milliseconds since ESP32 startup. */
static uint64_t monotonic_ms(void) {
  return (uint64_t)(esp_timer_get_time() / 1000LL);
}

/** @brief Configure and install the proven ESP-IDF legacy I2C driver. */
static esp_err_t install_i2c_bus(void) {
  const i2c_config_t configuration = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = APP_I2C_SDA_GPIO,
      .scl_io_num = APP_I2C_SCL_GPIO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = APP_I2C_FREQUENCY_HZ,
      .clk_flags = 0,
  };

  ESP_RETURN_ON_ERROR(i2c_param_config(APP_I2C_PORT, &configuration), TAG,
                      "I2C parameter configuration failed");
  ESP_RETURN_ON_ERROR(
      i2c_driver_install(APP_I2C_PORT, I2C_MODE_MASTER, 0U, 0U, 0), TAG,
      "I2C driver installation failed");
  return ESP_OK;
}

/** @brief Periodically acquire electrical data and refresh the LCD. */
static void monitoring_task(void *argument) {
  (void)argument;
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t loop_period = pdMS_TO_TICKS(APP_MONITOR_LOOP_INTERVAL_MS);
  uint64_t last_energy_sample_ms = 0U;
  uint64_t last_lcd_refresh_ms = 0U;
  uint64_t next_lcd_retry_ms = 0U;

  while (true) {
    const uint64_t now_ms = monotonic_ms();

    if ((now_ms - last_energy_sample_ms) >= APP_ENERGY_SAMPLE_INTERVAL_MS) {
      app_control_snapshot_t control = {0};
      bool relay_states[APP_LOAD_COUNT] = {false};
      if (control_service_get_snapshot(&control) == ESP_OK) {
        for (size_t index = 0U; index < APP_LOAD_COUNT; ++index) {
          relay_states[index] = control.loads[index].relay_on;
        }
      }
      (void)energy_service_sample(now_ms, relay_states);
      last_energy_sample_ms = now_ms;
    }

    if ((now_ms - last_lcd_refresh_ms) >= APP_LCD_REFRESH_INTERVAL_MS) {
      if (!s_monitor.display_available && (now_ms >= next_lcd_retry_ms)) {
        s_monitor.display_available = display_service_init() == ESP_OK;
        next_lcd_retry_ms = now_ms + APP_SENSOR_RETRY_INTERVAL_MS;
      }

      if (s_monitor.display_available) {
        const esp_err_t display_result = display_service_render(now_ms);
        if (display_result != ESP_OK) {
          ESP_LOGW(TAG, "LCD refresh failed; retrying after a short backoff");
          s_monitor.display_available = false;
          next_lcd_retry_ms = now_ms + APP_SENSOR_RETRY_INTERVAL_MS;
        }
      }
      last_lcd_refresh_ms = now_ms;
    }

    vTaskDelayUntil(&last_wake, loop_period);
  }
}

esp_err_t monitoring_service_init(void) {
  if (s_monitor.initialized) {
    return ESP_OK;
  }

  ESP_RETURN_ON_ERROR(install_i2c_bus(), TAG, "Shared I2C bus failed");
  ESP_RETURN_ON_ERROR(energy_service_init(), TAG, "Energy service failed");
  s_monitor.display_available = display_service_init() == ESP_OK;
  s_monitor.initialized = true;
  ESP_LOGI(TAG, "Monitoring service initialized");
  return ESP_OK;
}

esp_err_t monitoring_service_start(void) {
  if (!s_monitor.initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (s_monitor.task != NULL) {
    return ESP_OK;
  }

  const BaseType_t created = xTaskCreate(
      monitoring_task, "monitoring_service", APP_MONITOR_TASK_STACK_SIZE, NULL,
      APP_MONITOR_TASK_PRIORITY, &s_monitor.task);
  return created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
