/**
 * @file pzem004t_v1.h
 * @brief Driver for PZEM-004T V1.0 (Modbus-RTU over UART).
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * This module belongs to the DRIVERS layer.
 * Responsibilities:
 *  - Hardware abstraction for the PZEM-004T V1 sensor.
 *  - Send Modbus-RTU read requests and parse responses.
 *  - Provide a simple, non-blocking read API returning all measurements.
 * The module does NOT:
 *  - Make any system-level decisions.
 *  - Post events or directly interact with services/core.
 *  - Contain business logic.
 * =============================================================================
 *
 * @author SoY. Mathithyahu
 * @data 2026/04/29
 * @version 1.0.0
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief All electrical parameters measured by PZEM-004T V1. */
typedef struct {
    float voltage;      /**< Voltage in V (resolution 0.1 V) */
    float current;      /**< Current in A (resolution 0.001 A) */
    float power;        /**< Active power in W (resolution 0.1 W) */
    float energy;       /**< Energy in Wh (resolution 1 Wh) */
    float frequency;    /**< Frequency in Hz (resolution 0.1 Hz) */
    float pf;           /**< Power factor (0.00 .. 1.00, resolution 0.01) */
} pzem_data_t;

/**
 * @brief Initialise the PZEM-004T V1 UART interface.
 *
 * @param uart_num      UART port number (e.g., UART_NUM_1).
 * @param tx_pin        TX GPIO number.
 * @param rx_pin        RX GPIO number.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t pzem_init(uint8_t uart_num, int tx_pin, int rx_pin);

/**
 * @brief Read all measurements from the PZEM-004T V1 in one request.
 *
 * Blocks for at most ~200 ms (including a timeout margin).
 *
 * @param uart_num  UART port number used during initialisation.
 * @param data      Pointer to a pzem_data_t structure to fill.
 * @return ESP_OK on success, ESP_FAIL or other error code on failure.
 */
esp_err_t pzem_read_all(uint8_t uart_num, pzem_data_t *data);

#ifdef __cplusplus
}
#endif