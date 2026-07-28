/**
 * @file control_service.h
 * @brief Deterministic PIR, button, and relay-control service.
 * @author Matthithyahu
 */

#ifndef CONTROL_SERVICE_H
#define CONTROL_SERVICE_H

#include "app_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure inputs, force both relays OFF, and allocate the event queue.
 *
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
esp_err_t control_service_init(void);

/**
 * @brief Start the dedicated high-priority control task.
 *
 * @return ESP_OK on success or ESP_ERR_INVALID_STATE when not initialized.
 */
esp_err_t control_service_start(void);

/**
 * @brief Get the relay-event queue consumed by the notification service.
 *
 * @return Queue handle, or NULL before initialization.
 */
QueueHandle_t control_service_get_event_queue(void);

/**
 * @brief Copy the latest atomic control snapshot.
 *
 * @param[out] snapshot Destination for the copied state.
 * @return ESP_OK on success or ESP_ERR_INVALID_ARG for a NULL destination.
 */
esp_err_t control_service_get_snapshot(app_control_snapshot_t *snapshot);

/**
 * @brief Return a stable display name for a load.
 *
 * @param load_id Load identifier.
 * @return Constant string such as "Bulb" or "Fan".
 */
const char *control_service_load_name(app_load_id_t load_id);

/**
 * @brief Return the compact text for a relay transition reason.
 *
 * @param reason Relay transition reason.
 * @return Constant lowercase reason string.
 */
const char *control_service_reason_text(app_relay_reason_t reason);

/**
 * @brief Return a compact display label for a control mode.
 *
 * @param mode Control mode.
 * @return Constant mode label.
 */
const char *control_service_mode_text(app_control_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_SERVICE_H */
