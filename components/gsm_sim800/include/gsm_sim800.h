#ifndef GSM_SIM800_H
#define GSM_SIM800_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GSM SIM800 configuration structure
 */
typedef struct {
    uart_port_t uart_port;          ///< UART port number
    int tx_pin;                     ///< TX pin number
    int rx_pin;                     ///< RX pin number
    int baud_rate;                  ///< Baud rate (default: 9600)
    int buf_size;                   ///< UART buffer size
    uint32_t timeout_ms;            ///< Command timeout in milliseconds
    uint8_t retry_count;            ///< Command retry count
} gsm_config_t;

/**
 * @brief GSM SIM800 status enumeration
 */
typedef enum {
    GSM_STATUS_IDLE,                ///< Module is idle
    GSM_STATUS_INITIALIZING,        ///< Module is initializing
    GSM_STATUS_READY,               ///< Module is ready
    GSM_STATUS_SENDING_SMS,         ///< Sending SMS
    GSM_STATUS_RECEIVING_SMS,       ///< Receiving SMS
    GSM_STATUS_ERROR,               ///< Error state
} gsm_status_t;

/**
 * @brief SMS message structure for sending
 */
typedef struct {
    char phone_number[16];          ///< Phone number (max 15 digits + null)
    char message[161];              ///< Message text (max 160 chars + null)
    uint8_t retry_count;            ///< Current retry count
} sms_message_t;

/**
 * @brief Received SMS structure
 */
typedef struct {
    char sender[16];                ///< Sender phone number
    char timestamp[32];             ///< SMS timestamp
    char message[161];              ///< Message content
    uint8_t index;                  ///< SMS index in SIM
    bool authenticated;             ///< Whether SMS was authenticated
} received_sms_t;

/**
 * @brief SMS reception callback type
 */
typedef void (*sms_received_callback_t)(const received_sms_t *sms);

/**
 * @brief SMS authentication callback type
 * @return true if authenticated, false otherwise
 */
typedef bool (*sms_auth_callback_t)(const char *sender, const char *message);

/**
 * @brief Initialize GSM SIM800 module
 */
esp_err_t gsm_init(const gsm_config_t *config);

/**
 * @brief Deinitialize GSM SIM800 module
 */
esp_err_t gsm_deinit(void);

/**
 * @brief Send SMS message (blocking)
 */
esp_err_t gsm_send_sms(const char *phone_number, const char *message, uint32_t timeout_ms);

/**
 * @brief Send SMS message (non-blocking)
 */
esp_err_t gsm_send_sms_async(const char *phone_number, const char *message);

/**
 * @brief Check for new SMS messages
 */
esp_err_t gsm_check_sms(bool delete_after_read);

/**
 * @brief Process any pending received SMS
 * @return Number of SMS processed
 */
int gsm_process_received_sms(void);

/**
 * @brief Set password for SMS authentication
 */
void gsm_set_password(const char *password);

/**
 * @brief Set authorized phone numbers
 */
void gsm_set_authorized_numbers(const char **phone_numbers, uint8_t count);

/**
 * @brief Set callback for received SMS
 */
void gsm_set_received_callback(sms_received_callback_t callback);

/**
 * @brief Set callback for SMS authentication
 */
void gsm_set_auth_callback(sms_auth_callback_t callback);

/**
 * @brief Get current GSM module status
 */
gsm_status_t gsm_get_status(void);

/**
 * @brief Check if GSM module is ready
 */
bool gsm_is_ready(void);

/**
 * @brief Reset GSM module
 */
esp_err_t gsm_reset(void);

/**
 * @brief Set callback for SMS send events
 */
void gsm_set_sms_callback(void (*callback)(esp_err_t status, const char *phone_number));

#ifdef __cplusplus
}
#endif

#endif // GSM_SIM800_H