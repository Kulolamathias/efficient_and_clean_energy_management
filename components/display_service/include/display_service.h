/**
 * @file display_service.h
 * @brief LCD2004 rendering service with changed-line updates.
 * @author Matthithyahu
 */

#ifndef DISPLAY_SERVICE_H
#define DISPLAY_SERVICE_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the LCD on an already-installed I2C bus.
 *
 * @return ESP_OK on success; ESP_ERR_NOT_FOUND if the LCD is unavailable.
 */
esp_err_t display_service_init(void);

/**
 * @brief Render the current status or electrical screen.
 *
 * Only changed 20-character rows are transmitted, avoiding full-screen clear
 * flicker and reducing I2C traffic.
 *
 * @param now_ms Current monotonic time in milliseconds.
 * @return ESP_OK on success or an error if the LCD transaction failed.
 */
esp_err_t display_service_render(uint64_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_SERVICE_H */
