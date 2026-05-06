/**
 * @file gsm.h
 * @brief GSM driver for SIM800/SIM900 – reliable SMS sending.
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * This module belongs to the DRIVERS layer.
 * Responsibilities:
 *   - Initialise the GSM module (power on, UART, AT setup).
 *   - Send SMS messages to any international number.
 *   - Handle unsolicited result codes transparently.
 * The module does NOT:
 *   - Make decisions about when to send SMS.
 *   - Post events or interact with services/core directly.
 * =============================================================================
 *
 * @author SoY. Mathithyahu
 * @date 2026/05/06
 * @version 1.0.0
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GSM configuration parameters.
 */
typedef struct {
    uint8_t         uart_num;       /**< UART port (e.g., UART_NUM_2) */
    int             tx_pin;         /**< TX GPIO (ESP32 → GSM RX) */
    int             rx_pin;         /**< RX GPIO (ESP32 ← GSM TX) */
    int             pwrkey_pin;     /**< PWRKEY GPIO (active low, high‑Z otherwise) */
    int             baud_rate;      /**< Typically 9600 or 115200 */
} gsm_config_t;

/**
 * @brief Initialise the GSM module.
 *
 * Powers on the module, configures UART, and synchronises the AT interface.
 * Blocks for up to ~10 seconds while the module registers on the network.
 *
 * @param cfg   Pointer to configuration structure.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t gsm_init(const gsm_config_t *cfg);

/**
 * @brief Send an SMS in text mode.
 *
 * The function blocks until the message is sent or a timeout occurs.
 * Multiple threads may call this function – access is serialised internally.
 *
 * @param phone_number  International format, e.g., "+255688173415".
 * @param message       ASCII text (max ~160 chars for single SMS).
 * @return ESP_OK on success, ESP_FAIL if sending fails after retries.
 */
esp_err_t gsm_send_sms(const char *phone_number, const char *message);

/**
 * @brief Deinitialise the GSM module (power it down cleanly).
 *
 * @return ESP_OK.
 */
esp_err_t gsm_deinit(void);

#ifdef __cplusplus
}
#endif