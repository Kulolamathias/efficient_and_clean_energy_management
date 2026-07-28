/**
 * @file monitoring_service.h
 * @brief Single-owner coordinator for the shared LCD/INA219 I2C bus.
 * @author Matthithyahu
 */

#ifndef MONITORING_SERVICE_H
#define MONITORING_SERVICE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Install the shared I2C bus and initialize energy/display services.
 *
 * @return ESP_OK when monitoring can start.
 */
esp_err_t monitoring_service_init(void);

/**
 * @brief Start the task that serializes all INA219 and LCD bus access.
 *
 * @return ESP_OK on success or ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t monitoring_service_start(void);

#ifdef __cplusplus
}
#endif

#endif /* MONITORING_SERVICE_H */
