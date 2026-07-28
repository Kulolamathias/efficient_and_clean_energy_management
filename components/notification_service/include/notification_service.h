/**
 * @file notification_service.h
 * @brief Event-time measurement preparation and reliable GSM serialization.
 * @author Matthithyahu
 */

#ifndef NOTIFICATION_SERVICE_H
#define NOTIFICATION_SERVICE_H

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Allocate notification queues and connect to relay events.
 *
 * @return ESP_OK on success.
 */
esp_err_t notification_service_init(void);

/**
 * @brief Start independent snapshot-preparation and SIM800 sender tasks.
 *
 * @return ESP_OK on success or ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t notification_service_start(void);

/**
 * @brief Report whether the SIM800 sender has completed initialization.
 *
 * @return true when outgoing SMS transmission is ready.
 */
bool notification_service_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* NOTIFICATION_SERVICE_H */
