/**
 * @file energy_service.h
 * @brief Sampling, time-based integration, history, and persistence service.
 * @author Matthithyahu
 */

#ifndef ENERGY_SERVICE_H
#define ENERGY_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize both INA219 devices and restore saved energy totals.
 *
 * Missing sensors are non-fatal and are retried later by the sampling loop.
 *
 * @return ESP_OK when the service is ready, even if a sensor is offline.
 */
esp_err_t energy_service_init(void);

/**
 * @brief Sample all available sensors and integrate energy using real elapsed
 * time.
 *
 * This function must be called by only one monitoring task.
 *
 * @param now_ms Monotonic sample time in milliseconds.
 * @param relay_states Relay state paired with each load measurement.
 * @return ESP_OK when the service was called with valid arguments.
 */
esp_err_t energy_service_sample(uint64_t now_ms,
                                const bool relay_states[APP_LOAD_COUNT]);

/**
 * @brief Copy the latest electrical state for one load.
 *
 * @param load_id Requested load.
 * @param[out] snapshot Destination for the copied measurement.
 * @return ESP_OK on success or ESP_ERR_INVALID_ARG for bad arguments.
 */
esp_err_t energy_service_get_snapshot(app_load_id_t load_id,
                                      app_energy_snapshot_t *snapshot);

/**
 * @brief Average event-matching samples from the in-memory history.
 *
 * Only samples captured while the relay matched @p relay_on are selected. This
 * prevents an OFF measurement being reported for a rapid ON/OFF sequence.
 *
 * @param load_id Requested load.
 * @param relay_on Relay state required in every selected sample.
 * @param earliest_time_ms Earliest acceptable sample timestamp.
 * @param latest_time_ms Latest acceptable sample timestamp.
 * @param sample_count Number of samples to average.
 * @param[out] snapshot Destination for the averaged result.
 * @return ESP_OK when enough samples exist, otherwise ESP_ERR_NOT_FOUND.
 */
esp_err_t energy_service_get_averaged_snapshot(app_load_id_t load_id,
                                               bool relay_on,
                                               uint64_t earliest_time_ms,
                                               uint64_t latest_time_ms,
                                               size_t sample_count,
                                               app_energy_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* ENERGY_SERVICE_H */
