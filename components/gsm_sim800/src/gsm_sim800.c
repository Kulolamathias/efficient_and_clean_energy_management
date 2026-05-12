#include "gsm_sim800.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "string.h"
#include "inttypes.h"
#include <ctype.h>

static const char *TAG = "GSM_SIM800";

typedef struct {
    gsm_config_t config;
    gsm_status_t status;
    QueueHandle_t sms_queue;
    QueueHandle_t received_queue;
    TaskHandle_t async_task;
    SemaphoreHandle_t mutex;
    bool initialized;
    bool module_ready;
    char password[33];
    const char **authorized_numbers;
    uint8_t authorized_count;
    sms_received_callback_t received_callback;
    sms_auth_callback_t auth_callback;
    void (*sms_callback)(esp_err_t status, const char *phone_number);
} gsm_context_t;

static gsm_context_t g_gsm_ctx = {0};

#define RESPONSE_BUFFER_SIZE 2048  // Increased for SMS reading
static char g_response_buffer[RESPONSE_BUFFER_SIZE];

static const gsm_config_t DEFAULT_CONFIG = {
    .uart_port = UART_NUM_1,
    .tx_pin = GPIO_NUM_17,
    .rx_pin = GPIO_NUM_16,
    .baud_rate = 9600,
    .buf_size = 2048,
    .timeout_ms = 30000,
    .retry_count = 2,
};

// Private function declarations
static esp_err_t send_at_command(const char *cmd, const char *expected, uint32_t timeout, bool log_response);
static esp_err_t wait_for_response(const char *expected, uint32_t timeout);
static esp_err_t clear_uart_buffer(void);
static void async_sms_task(void *arg);
static esp_err_t internal_send_sms(const char *phone_number, const char *message, uint32_t timeout);
static bool is_authorized_number(const char *phone_number);
static void cleanup_phone_number(char *phone_number);
static esp_err_t read_and_process_sms(uint8_t index, bool delete_after);
static esp_err_t parse_sms_message(const char *response, received_sms_t *sms);

/**
 * @brief Parse SMS message from AT+CMGR response
 */
static esp_err_t parse_sms_message(const char *response, received_sms_t *sms) {
    if (!response || !sms) return ESP_FAIL;
    
    memset(sms, 0, sizeof(received_sms_t));
    
    // Find +CMGR: line
    char *cmgr_start = strstr(response, "+CMGR:");
    if (!cmgr_start) {
        ESP_LOGE(TAG, "No +CMGR in response");
        return ESP_FAIL;
    }
    
    // Parse format: +CMGR: "REC READ","+255688173415","","23/11/12,15:30:25+12"
    char *ptr = cmgr_start;
    
    // Find sender number (between quotes after first comma)
    char *quote1 = strchr(ptr, '"');
    if (!quote1) return ESP_FAIL;
    
    // Skip status field
    char *quote2 = strchr(quote1 + 1, '"');
    if (!quote2) return ESP_FAIL;
    
    // Find next field (sender)
    ptr = quote2 + 1;
    quote1 = strchr(ptr, '"');
    if (!quote1) return ESP_FAIL;
    
    quote2 = strchr(quote1 + 1, '"');
    if (!quote2) return ESP_FAIL;
    
    // Extract sender
    size_t sender_len = quote2 - quote1 - 1;
    if (sender_len > 0 && sender_len < sizeof(sms->sender)) {
        strncpy(sms->sender, quote1 + 1, sender_len);
        sms->sender[sender_len] = '\0';
    }
    
    // Find message content (after the timestamp field)
    // Look for the actual message after the header
    char *message_start = strstr(quote2, "\r\n");
    if (message_start) {
        message_start += 2;  // Skip \r\n
        
        // Find end of message (before OK or next command)
        char *message_end = strstr(message_start, "\r\nOK");
        if (!message_end) {
            message_end = strstr(message_start, "\r\n\r\n");
        }
        if (!message_end) {
            message_end = message_start + strlen(message_start);
        }
        
        size_t message_len = message_end - message_start;
        if (message_len > 0 && message_len < sizeof(sms->message)) {
            // Clean up the message (remove any trailing \r\n)
            char *temp = message_start;
            char *dest = sms->message;
            int i = 0;
            while (i < message_len && *temp && dest - sms->message < sizeof(sms->message) - 1) {
                if (*temp != '\r' && *temp != '\n') {
                    *dest++ = *temp;
                }
                temp++;
                i++;
            }
            *dest = '\0';
        }
    }
    
    // Clean phone number
    cleanup_phone_number(sms->sender);
    
    ESP_LOGI(TAG, "Parsed SMS - Sender: %s, Message: %s", sms->sender, sms->message);
    return ESP_OK;
}

/**
 * @brief Read and process a specific SMS by index
 */
static esp_err_t read_and_process_sms(uint8_t index, bool delete_after) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CMGR=%d", index);
    
    ESP_LOGI(TAG, "Reading SMS index %d", index);
    
    // Send command to read SMS
    esp_err_t ret = send_at_command(cmd, "+CMGR:", 5000, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SMS index %d", index);
        return ESP_FAIL;
    }
    
    // Parse the SMS
    received_sms_t sms;
    ret = parse_sms_message(g_response_buffer, &sms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse SMS index %d", index);
        return ESP_FAIL;
    }
    
    sms.index = index;
    
    // Add to received queue
    if (g_gsm_ctx.received_queue) {
        if (xQueueSend(g_gsm_ctx.received_queue, &sms, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Received queue full");
        }
    }
    
    // Delete SMS if requested
    if (delete_after) {
        snprintf(cmd, sizeof(cmd), "AT+CMGD=%d", index);
        send_at_command(cmd, "OK", 3000, false);
        ESP_LOGI(TAG, "Deleted SMS index %d", index);
    }
    
    return ESP_OK;
}

/**
 * @brief Clear UART receive buffer
 */
static esp_err_t clear_uart_buffer(void) {
    uint8_t temp[128];
    int len = 0;
    
    do {
        len = uart_read_bytes(g_gsm_ctx.config.uart_port, temp, sizeof(temp), 50 / portTICK_PERIOD_MS);
    } while (len > 0);
    
    return ESP_OK;
}

/**
 * @brief Wait for specific response
 */
static esp_err_t wait_for_response(const char *expected, uint32_t timeout) {
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    int pos = 0;
    
    memset(g_response_buffer, 0, RESPONSE_BUFFER_SIZE);
    
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < timeout) {
        int len = uart_read_bytes(g_gsm_ctx.config.uart_port, 
                                 (uint8_t *)g_response_buffer + pos, 
                                 RESPONSE_BUFFER_SIZE - pos - 1, 
                                 100 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            pos += len;
            g_response_buffer[pos] = '\0';
            
            // Check for expected response
            if (strstr(g_response_buffer, expected) != NULL) {
                return ESP_OK;
            }
            
            // Check for ERROR
            if (strstr(g_response_buffer, "ERROR") != NULL) {
                ESP_LOGE(TAG, "Module returned ERROR");
                return ESP_FAIL;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (pos > 0) {
        ESP_LOGE(TAG, "Timeout waiting for: %s", expected);
        ESP_LOGD(TAG, "Buffer content: %s", g_response_buffer);
    }
    
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Send AT command and wait for response
 */
static esp_err_t send_at_command(const char *cmd, const char *expected, uint32_t timeout, bool log_response) {
    if (!g_gsm_ctx.initialized) {
        return ESP_FAIL;
    }

    char at_cmd[128];
    snprintf(at_cmd, sizeof(at_cmd), "%s\r", cmd);
    
    if (log_response) {
        ESP_LOGI(TAG, "TX: %s", cmd);
    }
    
    // Clear buffer before sending
    clear_uart_buffer();
    
    // Send command
    int bytes_written = uart_write_bytes(g_gsm_ctx.config.uart_port, at_cmd, strlen(at_cmd));
    if (bytes_written != strlen(at_cmd)) {
        ESP_LOGE(TAG, "Failed to write command");
        return ESP_FAIL;
    }
    
    // Wait for response
    esp_err_t ret = wait_for_response(expected, timeout);
    if (ret == ESP_OK && log_response) {
        ESP_LOGI(TAG, "RX: OK for %s", cmd);
    } else if (ret != ESP_OK && log_response) {
        ESP_LOGE(TAG, "Failed: %s", cmd);
    }
    
    return ret;
}

/**
 * @brief Initialize GSM SIM800 module
 */
esp_err_t gsm_init(const gsm_config_t *config) {
    if (g_gsm_ctx.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    // Use default config if none provided
    if (config == NULL) {
        g_gsm_ctx.config = DEFAULT_CONFIG;
    } else {
        g_gsm_ctx.config = *config;
    }
    
    // Initialize mutex
    g_gsm_ctx.mutex = xSemaphoreCreateMutex();
    if (g_gsm_ctx.mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    // Create SMS queues
    g_gsm_ctx.sms_queue = xQueueCreate(5, sizeof(sms_message_t));
    g_gsm_ctx.received_queue = xQueueCreate(10, sizeof(received_sms_t));
    
    // Initialize other fields
    memset(g_gsm_ctx.password, 0, sizeof(g_gsm_ctx.password));
    g_gsm_ctx.authorized_numbers = NULL;
    g_gsm_ctx.authorized_count = 0;
    g_gsm_ctx.received_callback = NULL;
    g_gsm_ctx.auth_callback = NULL;
    g_gsm_ctx.sms_callback = NULL;
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = g_gsm_ctx.config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(g_gsm_ctx.config.uart_port, 
                                       g_gsm_ctx.config.buf_size, 
                                       g_gsm_ctx.config.buf_size, 
                                       0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(g_gsm_ctx.config.uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(g_gsm_ctx.config.uart_port, 
                                g_gsm_ctx.config.tx_pin, 
                                g_gsm_ctx.config.rx_pin, 
                                UART_PIN_NO_CHANGE, 
                                UART_PIN_NO_CHANGE));
    
    // Set initial status
    g_gsm_ctx.status = GSM_STATUS_INITIALIZING;
    g_gsm_ctx.initialized = true;
    g_gsm_ctx.module_ready = false;
    
    // Give module time to power up
    ESP_LOGI(TAG, "Waiting for module...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Clear buffer
    clear_uart_buffer();
    
    // Test communication
    esp_err_t ret = send_at_command("AT", "OK", 5000, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Module not responding");
        return ESP_FAIL;
    }
    
    // Configure module
    send_at_command("ATE0", "OK", 3000, true);      // Echo off
    send_at_command("AT+CMGF=1", "OK", 3000, true); // Text mode
    
    // Configure SMS reception
    // CNMI=2,1 - Show new SMS directly with content
    send_at_command("AT+CNMI=2,1,0,0,0", "OK", 3000, true);
    
    // Delete all old SMS
    send_at_command("AT+CMGDA=\"DEL ALL\"", "OK", 5000, true);
    
    ESP_LOGI(TAG, "Module ready");
    
    g_gsm_ctx.status = GSM_STATUS_READY;
    g_gsm_ctx.module_ready = true;
    
    // Create async task for sending
    xTaskCreate(async_sms_task, "gsm_async", 4096, NULL, 5, &g_gsm_ctx.async_task);
    
    return ESP_OK;
}

/**
 * @brief Internal SMS sending function
 */
static esp_err_t internal_send_sms(const char *phone_number, const char *message, uint32_t timeout) {
    if (!g_gsm_ctx.initialized) {
        return ESP_FAIL;
    }
    
    // Take mutex to ensure exclusive access
    if (xSemaphoreTake(g_gsm_ctx.mutex, pdMS_TO_TICKS(timeout)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to get mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGI(TAG, "SEND START to %s", phone_number);
    
    // Clear any pending data
    clear_uart_buffer();
    
    // Step 1: Set text mode
    esp_err_t ret = send_at_command("AT+CMGF=1", "OK", 5000, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed text mode");
        xSemaphoreGive(g_gsm_ctx.mutex);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Step 2: Start SMS
    char sms_cmd[64];
    snprintf(sms_cmd, sizeof(sms_cmd), "AT+CMGS=\"%s\"", phone_number);
    ret = send_at_command(sms_cmd, ">", 5000, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed SMS start");
        xSemaphoreGive(g_gsm_ctx.mutex);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Step 3: Send message with Ctrl+Z
    char full_message[162];
    snprintf(full_message, sizeof(full_message), "%s%c", message, 0x1A);
    
    int bytes_written = uart_write_bytes(g_gsm_ctx.config.uart_port, full_message, strlen(full_message));
    if (bytes_written != strlen(full_message)) {
        ESP_LOGE(TAG, "Failed to write message");
        xSemaphoreGive(g_gsm_ctx.mutex);
        return ESP_FAIL;
    }
    
    // Step 4: Wait for confirmation
    ESP_LOGI(TAG, "Waiting for confirmation...");
    ret = wait_for_response("+CMGS:", 30000);
    if (ret != ESP_OK) {
        // Try alternative response
        ret = wait_for_response("OK", 10000);
    }
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ SMS SENT to %s", phone_number);
    } else {
        ESP_LOGE(TAG, "❌ SMS FAILED to %s", phone_number);
    }
    
    xSemaphoreGive(g_gsm_ctx.mutex);
    return ret;
}

/**
 * @brief Send SMS message (blocking)
 */
esp_err_t gsm_send_sms(const char *phone_number, const char *message, uint32_t timeout) {
    return internal_send_sms(phone_number, message, timeout);
}

/**
 * @brief Send SMS message (non-blocking)
 */
esp_err_t gsm_send_sms_async(const char *phone_number, const char *message) {
    if (!g_gsm_ctx.initialized || g_gsm_ctx.sms_queue == NULL) {
        return ESP_FAIL;
    }
    
    sms_message_t sms = {0};
    strncpy(sms.phone_number, phone_number, sizeof(sms.phone_number) - 1);
    strncpy(sms.message, message, sizeof(sms.message) - 1);
    sms.retry_count = 0;
    
    if (xQueueSend(g_gsm_ctx.sms_queue, &sms, 0) != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Queued SMS for %s", phone_number);
    return ESP_OK;
}

/**
 * @brief Async SMS task
 */
static void async_sms_task(void *arg) {
    sms_message_t sms;
    
    while (1) {
        if (xQueueReceive(g_gsm_ctx.sms_queue, &sms, portMAX_DELAY) == pdTRUE) {
            esp_err_t result = internal_send_sms(sms.phone_number, sms.message, g_gsm_ctx.config.timeout_ms);
            
            if (g_gsm_ctx.sms_callback != NULL) {
                g_gsm_ctx.sms_callback(result, sms.phone_number);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Check for new SMS messages - FIXED VERSION
 */
esp_err_t gsm_check_sms(bool delete_after_read) {
    if (!g_gsm_ctx.initialized) {
        return ESP_FAIL;
    }
    
    // Take mutex
    if (xSemaphoreTake(g_gsm_ctx.mutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGI(TAG, "Checking for SMS...");
    
    // List ALL SMS (read and unread)
    esp_err_t ret = send_at_command("AT+CMGL=\"ALL\"", "OK", 10000, false);
    
    int sms_count = 0;
    
    if (ret == ESP_OK) {
        // Parse response to find SMS indexes
        char *ptr = g_response_buffer;
        
        // Look for +CMGL: lines which indicate SMS presence
        while ((ptr = strstr(ptr, "+CMGL:")) != NULL) {
            // Parse index number (comes right after +CMGL:)
            char *index_start = ptr + 6; // Skip "+CMGL:"
            uint8_t index = atoi(index_start);
            
            ESP_LOGI(TAG, "Found SMS at index %d, reading...", index);
            
            // Read and process this SMS
            if (read_and_process_sms(index, delete_after_read) == ESP_OK) {
                sms_count++;
            }
            
            // Move to next position
            ptr++;
        }
    }
    
    if (sms_count > 0) {
        ESP_LOGI(TAG, "Processed %d SMS messages", sms_count);
    } else {
        ESP_LOGD(TAG, "No SMS found");
    }
    
    xSemaphoreGive(g_gsm_ctx.mutex);
    return (sms_count > 0) ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Process received SMS from queue
 */
int gsm_process_received_sms(void) {
    if (!g_gsm_ctx.initialized || !g_gsm_ctx.received_queue) {
        return 0;
    }
    
    int processed = 0;
    received_sms_t sms;
    
    while (xQueueReceive(g_gsm_ctx.received_queue, &sms, 0) == pdTRUE) {
        processed++;
        
        if (g_gsm_ctx.received_callback != NULL) {
            g_gsm_ctx.received_callback(&sms);
        }
    }
    
    return processed;
}

/**
 * @brief Clean up phone number
 */
static void cleanup_phone_number(char *phone_number) {
    if (!phone_number) return;
    
    char *src = phone_number;
    char *dst = phone_number;
    
    while (*src) {
        if (isdigit((unsigned char)*src) || *src == '+') {
            *dst++ = *src;
        }
        src++;
    }
    *dst = '\0';
}

/**
 * @brief Check if phone number is authorized
 */
static bool is_authorized_number(const char *phone_number) {
    if (g_gsm_ctx.authorized_count == 0) {
        return true; // No list means all numbers are authorized
    }
    
    if (!g_gsm_ctx.authorized_numbers || !phone_number) {
        return false;
    }
    
    char cleaned_phone[16];
    strncpy(cleaned_phone, phone_number, sizeof(cleaned_phone) - 1);
    cleanup_phone_number(cleaned_phone);
    
    for (uint8_t i = 0; i < g_gsm_ctx.authorized_count; i++) {
        if (g_gsm_ctx.authorized_numbers[i]) {
            char cleaned_auth[16];
            strncpy(cleaned_auth, g_gsm_ctx.authorized_numbers[i], sizeof(cleaned_auth) - 1);
            cleanup_phone_number(cleaned_auth);
            
            if (strcmp(cleaned_phone, cleaned_auth) == 0) {
                return true;
            }
        }
    }
    
    return false;
}

/**
 * @brief Set password for SMS authentication
 */
void gsm_set_password(const char *password) {
    if (password) {
        strncpy(g_gsm_ctx.password, password, sizeof(g_gsm_ctx.password) - 1);
        ESP_LOGI(TAG, "Password: %s", g_gsm_ctx.password);
    }
}

/**
 * @brief Set authorized phone numbers
 */
void gsm_set_authorized_numbers(const char **phone_numbers, uint8_t count) {
    g_gsm_ctx.authorized_numbers = phone_numbers;
    g_gsm_ctx.authorized_count = count;
    
    ESP_LOGI(TAG, "Authorized %d numbers", count);
    for (uint8_t i = 0; i < count; i++) {
        if (phone_numbers[i]) {
            ESP_LOGI(TAG, "  %d: %s", i + 1, phone_numbers[i]);
        }
    }
}

/**
 * @brief Set callback for received SMS
 */
void gsm_set_received_callback(sms_received_callback_t callback) {
    g_gsm_ctx.received_callback = callback;
}

/**
 * @brief Set callback for SMS authentication
 */
void gsm_set_auth_callback(sms_auth_callback_t callback) {
    g_gsm_ctx.auth_callback = callback;
}

/**
 * @brief Get current status
 */
gsm_status_t gsm_get_status(void) {
    return g_gsm_ctx.status;
}

/**
 * @brief Check if GSM is ready
 */
bool gsm_is_ready(void) {
    return g_gsm_ctx.initialized && g_gsm_ctx.module_ready;
}

/**
 * @brief Reset GSM module
 */
esp_err_t gsm_reset(void) {
    if (!g_gsm_ctx.initialized) {
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Resetting...");
    
    if (xSemaphoreTake(g_gsm_ctx.mutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Send reset command
    esp_err_t ret = send_at_command("AT", "OK", 5000, true);
    
    if (ret == ESP_OK) {
        g_gsm_ctx.status = GSM_STATUS_READY;
        ESP_LOGI(TAG, "Reset OK");
    } else {
        g_gsm_ctx.status = GSM_STATUS_ERROR;
        ESP_LOGE(TAG, "Reset failed");
    }
    
    xSemaphoreGive(g_gsm_ctx.mutex);
    return ret;
}

/**
 * @brief Set SMS callback
 */
void gsm_set_sms_callback(void (*callback)(esp_err_t status, const char *phone_number)) {
    g_gsm_ctx.sms_callback = callback;
}