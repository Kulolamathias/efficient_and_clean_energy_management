/**
 * @file main/main.c
 * @brief Application composition root for the energy-management system.
 * @author Matthithyahu
 *
 * Business logic lives in focused components. Startup order deliberately makes
 * button/relay control operational before slower I2C and SIM800 initialization.
 */

#include "app_config.h"
#include "control_service.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "monitoring_service.h"
#include "notification_service.h"
#include "nvs_flash.h"

static const char *TAG = "app_main";

/** @brief Initialize NVS, recovering from incompatible or exhausted pages. */
static esp_err_t initialize_nvs(void) {
  esp_err_t result = nvs_flash_init();
  if ((result == ESP_ERR_NVS_NO_FREE_PAGES) ||
      (result == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
    ESP_LOGW(TAG, "NVS requires reinitialization");
    ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
    result = nvs_flash_init();
  }
  return result;
}

/**
 * @brief ESP-IDF application entry point.
 *
 * @return This function does not return; it deletes its startup task after all
 *         long-lived service tasks have been created.
 */
void app_main(void) {
  ESP_LOGI(TAG, "%s by %s", APP_PROJECT_NAME, APP_PROJECT_AUTHOR);

  /* GPIO safe states and the non-blocking button task come first. */
  ESP_ERROR_CHECK(control_service_init());
  ESP_ERROR_CHECK(control_service_start());

  ESP_ERROR_CHECK(initialize_nvs());

  /* A separate task owns all potentially slow I2C operations. */
  ESP_ERROR_CHECK(monitoring_service_init());
  ESP_ERROR_CHECK(monitoring_service_start());

  /* Electrical snapshots and modem delivery use separate queues/tasks. */
  ESP_ERROR_CHECK(notification_service_init());
  ESP_ERROR_CHECK(notification_service_start());

  ESP_LOGI(TAG, "All services started");
  vTaskDelete(NULL);
}
