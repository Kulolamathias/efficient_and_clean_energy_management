/**
 * @file ina219.h
 * @brief INA219 DC current/power monitor driver (legacy I2C – to be modernised).
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * Driver layer. Abstracts the INA219 sensor over I2C.
 * Provides calibrated current, voltage, and power readings.
 * =============================================================================
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Configuration for the INA219 device. */
typedef struct {
    uint8_t i2c_addr;       /**< I2C address (default 0x40) */
    float shunt_resistance; /**< Shunt resistor value in ohms (default 0.1) */
    float max_current;      /**< Maximum expected current in A (used for calibration) */
} ina219_config_t;

/** @brief Measurement results. */
typedef struct {
    float bus_voltage;      /**< Bus voltage in V */
    float shunt_voltage;    /**< Shunt voltage in mV */
    float current;          /**< Current in A */
    float power;            /**< Power in W */
} ina219_data_t;

/**
 * @brief Initialise the INA219 and configure calibration.
 *
 * @param config Pointer to device configuration.
 * @return ESP_OK on success.
 */
esp_err_t ina219_init(const ina219_config_t *config);

/**
 * @brief Trigger a single measurement and read all values.
 *
 * @param data Output structure filled with readings.
 * @return ESP_OK on success.
 */
esp_err_t ina219_read(ina219_data_t *data);

#ifdef __cplusplus
}
#endif