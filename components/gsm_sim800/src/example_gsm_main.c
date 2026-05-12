#include <stdio.h>
#include <string.h>
#include "gsm_sim800.h"
#include "esp_log.h"
#include "ctype.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define PHONE_NUMBER "+255688173415"
#define SMS_PASSWORD "SECRET123"

// GPIO to control
#define CONTROL_GPIO GPIO_NUM_2

// Authorized phone numbers
static const char *authorized_numbers[] = {
    "+255688173415",
};

// Simple command structure
typedef struct {
    char command[32];
    void (*handler)(const char *sender);
    const char *description;
} command_t;

// Global state
static bool system_enabled = true;
static uint32_t command_count = 0;

// Command handlers
static void handle_status(const char *sender) {
    char response[100];
    snprintf(response, sizeof(response), 
             "System Status:\n"
             "Enabled: %s\n"
             "Commands: %lu\n"
             "Uptime: %lu sec",
             system_enabled ? "YES" : "NO",
             command_count,
             xTaskGetTickCount() * portTICK_PERIOD_MS / 1000);
    ESP_LOGI("CMD", "Sending status response");
    gsm_send_sms_async(sender, response);
}

static void handle_enable(const char *sender) {
    system_enabled = true;
    ESP_LOGI("CMD", "System enabled");
    gsm_send_sms_async(sender, "✅ System ENABLED");
}

static void handle_disable(const char *sender) {
    system_enabled = false;
    ESP_LOGI("CMD", "System disabled");
    gsm_send_sms_async(sender, "❌ System DISABLED");
}

static void handle_led_on(const char *sender) {
    gpio_set_level(CONTROL_GPIO, 1);
    ESP_LOGI("CMD", "LED turned ON");
    gsm_send_sms_async(sender, "💡 LED ON");
}

static void handle_led_off(const char *sender) {
    gpio_set_level(CONTROL_GPIO, 0);
    ESP_LOGI("CMD", "LED turned OFF");
    gsm_send_sms_async(sender, "🔴 LED OFF");
}

static void handle_help(const char *sender) {
    const char *help = 
        "Available commands:\n"
        "SECRET123 STATUS\n"
        "SECRET123 ENABLE\n"
        "SECRET123 DISABLE\n"
        "SECRET123 LED ON\n"
        "SECRET123 LED OFF\n"
        "SECRET123 HELP";
    ESP_LOGI("CMD", "Sending help");
    gsm_send_sms_async(sender, help);
}

// Command definitions
static const command_t commands[] = {
    {"SECRET123 STATUS", handle_status, "Get system status"},
    {"SECRET123 ENABLE", handle_enable, "Enable system"},
    {"SECRET123 DISABLE", handle_disable, "Disable system"},
    {"SECRET123 LED ON", handle_led_on, "Turn LED ON"},
    {"SECRET123 LED OFF", handle_led_off, "Turn LED OFF"},
    {"SECRET123 HELP", handle_help, "Show help"},
};

// Custom authentication function
bool custom_auth_callback(const char *sender, const char *message) {
    ESP_LOGI("AUTH", "Custom auth check for %s", sender);
    
    // Example: Allow emergency commands without password
    if (strstr(message, "EMERGENCY") != NULL) {
        ESP_LOGI("AUTH", "Emergency command allowed");
        return true;
    }
    
    return false;
}

// Process received SMS with INTERACTIVE FEEDBACK
void handle_received_sms(const received_sms_t *sms) {
    ESP_LOGI("SMS", "========================================");
    ESP_LOGI("SMS", "📨 RECEIVED SMS:");
    ESP_LOGI("SMS", "  From: %s", sms->sender);
    ESP_LOGI("SMS", "  Message: %s", sms->message);
    ESP_LOGI("SMS", "========================================");
    
    // Check if sender is authorized
    bool authorized = false;
    for (int i = 0; i < sizeof(authorized_numbers) / sizeof(authorized_numbers[0]); i++) {
        char sender_clean[16], auth_clean[16];
        strcpy(sender_clean, sms->sender);
        strcpy(auth_clean, authorized_numbers[i]);
        
        // Remove any non-digit characters except +
        for (char *p = sender_clean; *p; p++) {
            if (!isdigit((unsigned char)*p) && *p != '+') {
                *p = ' ';
            }
        }
        for (char *p = auth_clean; *p; p++) {
            if (!isdigit((unsigned char)*p) && *p != '+') {
                *p = ' ';
            }
        }
        
        if (strstr(sender_clean, auth_clean) != NULL || strstr(auth_clean, sender_clean) != NULL) {
            authorized = true;
            break;
        }
    }
    
    if (!authorized) {
        ESP_LOGW("SMS", "❌ Unauthorized sender: %s", sms->sender);
        gsm_send_sms_async(sms->sender, "❌ ACCESS DENIED: Unauthorized number");
        return;
    }
    
    // Check if message contains password
    if (strstr(sms->message, SMS_PASSWORD) == NULL) {
        // Try custom auth
        if (!custom_auth_callback(sms->sender, sms->message)) {
            ESP_LOGW("SMS", "❌ Invalid password in: %s", sms->message);
            gsm_send_sms_async(sms->sender, "❌ ACCESS DENIED: Invalid password");
            return;
        }
    }
    
    // Send ACKNOWLEDGEMENT
    ESP_LOGI("SMS", "✅ Authorized, sending acknowledgement...");
    gsm_send_sms_async(sms->sender, "✅ Command received. Processing...");
    
    // Process command
    bool command_found = false;
    for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        if (strstr(sms->message, commands[i].command) != NULL) {
            ESP_LOGI("SMS", "Executing: %s", commands[i].description);
            
            // Execute command handler
            commands[i].handler(sms->sender);
            
            command_count++;
            command_found = true;
            break;
        }
    }
    
    if (!command_found) {
        ESP_LOGW("SMS", "❓ Unknown command: %s", sms->message);
        gsm_send_sms_async(sms->sender, "❓ Unknown command. Send 'SECRET123 HELP' for list.");
    }
}

// SMS send callback
void sms_send_callback(esp_err_t status, const char *phone_number) {
    if (status == ESP_OK) {
        ESP_LOGI("SEND", "✅ Sent to %s", phone_number);
    } else {
        ESP_LOGE("SEND", "❌ Failed to %s", phone_number);
    }
}

// Main application
void app_main(void)
{
    // Initialize GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CONTROL_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(CONTROL_GPIO, 0);
    
    ESP_LOGI("MAIN", "Starting GSM Control System");
    
    // Configure GSM
    gsm_config_t config = {
        .uart_port = UART_NUM_2,
        .tx_pin = GPIO_NUM_17,
        .rx_pin = GPIO_NUM_16,
        .baud_rate = 9600,
        .buf_size = 2048,
        .timeout_ms = 45000,
        .retry_count = 2,
    };
    
    // Initialize GSM
    if (gsm_init(&config) != ESP_OK) {
        ESP_LOGE("MAIN", "GSM init failed");
        return;
    }
    
    ESP_LOGI("MAIN", "GSM initialized");
    
    // Set callbacks
    gsm_set_received_callback(handle_received_sms);
    gsm_set_sms_callback(sms_send_callback);
    gsm_set_auth_callback(custom_auth_callback);
    gsm_set_password(SMS_PASSWORD);
    gsm_set_authorized_numbers(authorized_numbers, 
                              sizeof(authorized_numbers) / sizeof(authorized_numbers[0]));
    
    // Send welcome message
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI("MAIN", "Sending welcome message...");
    gsm_send_sms(PHONE_NUMBER, "System Online. Send 'SECRET123 HELP' for commands.", 60000);
    
    ESP_LOGI("MAIN", "====================================");
    ESP_LOGI("MAIN", "SYSTEM READY");
    ESP_LOGI("MAIN", "Password: %s", SMS_PASSWORD);
    ESP_LOGI("MAIN", "Auth Number: %s", PHONE_NUMBER);
    ESP_LOGI("MAIN", "====================================");
    
    // Main loop
    uint32_t loop_counter = 0;
    
    while (1) {
        loop_counter++;
        
        // Check for SMS every 5 seconds (more frequent)
        if (loop_counter % 50 == 0) {  // 50 * 100ms = 5 seconds
            ESP_LOGI("MAIN", "--- Checking SMS ---");
            esp_err_t sms_result = gsm_check_sms(true);  // Check and delete
            
            if (sms_result == ESP_OK) {
                ESP_LOGI("MAIN", "SMS found, processing...");
                // Process any received SMS
                int processed = gsm_process_received_sms();
                if (processed > 0) {
                    ESP_LOGI("MAIN", "Processed %d SMS", processed);
                }
            }
        }
        
        // Send heartbeat every 2 minutes
        if (loop_counter % 1200 == 0) {  // 1200 * 100ms = 2 minutes
            ESP_LOGI("MAIN", "💓 System heartbeat");
            // Optional: Send status update
            // char heartbeat_msg[60];
            // snprintf(heartbeat_msg, sizeof(heartbeat_msg), "Heartbeat: %lu commands", command_count);
            // gsm_send_sms_async(PHONE_NUMBER, heartbeat_msg);
        }
        
        // DETERMINISTIC delay
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Prevent overflow
        if (loop_counter >= 1000000) {
            loop_counter = 0;
        }
    }
}