/**
 * @file app_config.h
 * @brief Human-readable configuration for the complete application.
 * @author Matthithyahu
 *
 * Change installation-specific pins, polarities, addresses, and timings here.
 * Service implementations should not contain unexplained hardware numbers.
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"

/* --------------------------------------------------------------------------
 * Product identity
 * -------------------------------------------------------------------------- */
#define APP_PROJECT_NAME "Efficient Clean Energy Manager"
#define APP_PROJECT_AUTHOR "Matthithyahu"

/* --------------------------------------------------------------------------
 * Hardware map
 *
 * These values preserve the pins and relay polarity used by the actual source
 * project. They intentionally take precedence over the older written summary.
 * -------------------------------------------------------------------------- */
#define APP_PIR_GPIO GPIO_NUM_23
#define APP_BUTTON_BULB_GPIO GPIO_NUM_32
#define APP_BUTTON_FAN_GPIO GPIO_NUM_33
#define APP_RELAY_BULB_GPIO GPIO_NUM_18
#define APP_RELAY_FAN_GPIO GPIO_NUM_19

#define APP_BUTTON_PRESSED_LEVEL 0
#define APP_RELAY_ON_LEVEL 1
#define APP_RELAY_OFF_LEVEL 0

/* Verified on the installed HW-740: idle is LOW and motion is HIGH. */
#define APP_PIR_ACTIVE_LEVEL 1
#define APP_PIR_INTERNAL_PULLUP_ENABLED 0
#define APP_PIR_INTERNAL_PULLDOWN_ENABLED 0

/* --------------------------------------------------------------------------
 * I2C bus, LCD2004, and INA219 devices
 * -------------------------------------------------------------------------- */
#define APP_I2C_PORT I2C_NUM_0
#define APP_I2C_SDA_GPIO GPIO_NUM_21
#define APP_I2C_SCL_GPIO GPIO_NUM_22
#define APP_I2C_FREQUENCY_HZ 100000U
#define APP_I2C_TRANSACTION_TIMEOUT_MS 100U

#define APP_LCD_I2C_ADDRESS 0x27U
#define APP_LCD_ROWS 4U
#define APP_LCD_COLUMNS 20U
#define APP_LCD_COMMAND_DELAY_US 50U
#define APP_LCD_TRANSACTION_TIMEOUT_MS 25U

#define APP_INA219_BULB_ADDRESS 0x44U
#define APP_INA219_FAN_ADDRESS 0x40U
#define APP_INA219_SHUNT_RESISTANCE_OHM 0.1f
#define APP_INA219_MAX_EXPECTED_CURRENT_A 3.2f
#define APP_INA219_ADC_AVERAGE_COUNT 4U

/* --------------------------------------------------------------------------
 * Deterministic control timings
 * -------------------------------------------------------------------------- */
#define APP_CONTROL_TASK_STACK_SIZE 4096U
#define APP_CONTROL_TASK_PRIORITY 10U
#define APP_CONTROL_POLL_INTERVAL_MS 10U
#define APP_BUTTON_DEBOUNCE_MS 30U

#define APP_PIR_STARTUP_IGNORE_MS 5000U /**< Startup ignore time in milliseconds */
/*
 * A short debounce rejects electrical spikes while accepting real HW-740
 * pulses quickly. There is no additional confirm-on delay: once this debounce
 * succeeds, the relay is written in the same control cycle.
 */
#define APP_PIR_DEBOUNCE_MS 20U
/*
 * Once qualified motion disappears, keep automatically controlled loads ON
 * for this duration. Change this named value to tune the absence grace period.
 */
#define APP_PIR_ABSENCE_HOLD_MS 6000U

/* A manual ON command owns the relay for at least one complete minute. */
#define APP_MANUAL_ON_HOLD_MS 60000U
#define APP_MANUAL_OFF_LOCKOUT_MS 45000U
#define APP_RELAY_EVENT_QUEUE_DEPTH 32U

/* --------------------------------------------------------------------------
 * Monitoring, display, and energy persistence
 * -------------------------------------------------------------------------- */
#define APP_MONITOR_TASK_STACK_SIZE 6144U
#define APP_MONITOR_TASK_PRIORITY 6U
#define APP_MONITOR_LOOP_INTERVAL_MS 20U
#define APP_ENERGY_SAMPLE_INTERVAL_MS 100U
#define APP_ENERGY_HISTORY_LENGTH 64U
#define APP_LCD_REFRESH_INTERVAL_MS 500U
#define APP_LCD_SCREEN_SWITCH_INTERVAL_MS 3000U
#define APP_SENSOR_RETRY_INTERVAL_MS 10000U
#define APP_SENSOR_FAILURES_BEFORE_REINIT 3U

#define APP_ENERGY_PERSISTENCE_ENABLED 1
#define APP_ENERGY_PERSIST_INTERVAL_MS (15U * 60U * 1000U)
#define APP_ENERGY_PERSIST_MINIMUM_CHANGE_WH 0.01

/* --------------------------------------------------------------------------
 * Event-time electrical snapshots
 *
 * Relay switching remains immediate. These delays apply only to the sensor
 * snapshot used in the notification message.
 * -------------------------------------------------------------------------- */
#define APP_NOTIFICATION_ON_SETTLE_MS 750U
#define APP_NOTIFICATION_OFF_SETTLE_MS 300U
#define APP_NOTIFICATION_SAMPLE_COUNT 3U
#define APP_NOTIFICATION_SAMPLE_TIMEOUT_MS 3500U
#define APP_NOTIFICATION_SAMPLE_POLL_MS 25U
#define APP_PREPARED_SMS_QUEUE_DEPTH 32U
#define APP_SMS_MESSAGE_MAX_LENGTH 160U

/* --------------------------------------------------------------------------
 * SIM800 configuration
 *
 * The baud rate and long send timeouts preserve the proven modem behavior.
 * Incoming polling remains disabled because command handling is not yet an
 * application requirement and polling must never delay outgoing alerts.
 * -------------------------------------------------------------------------- */
#define APP_GSM_UART_PORT UART_NUM_2
#define APP_GSM_TX_GPIO GPIO_NUM_17
#define APP_GSM_RX_GPIO GPIO_NUM_16
#define APP_GSM_BAUD_RATE 9600
#define APP_GSM_UART_BUFFER_SIZE 2048
#define APP_GSM_COMMAND_TIMEOUT_MS 30000U
#define APP_GSM_DRIVER_RETRY_COUNT 2U
#define APP_GSM_SEND_RETRY_COUNT 2U
#define APP_GSM_SEND_RETRY_DELAY_MS 2000U
#define APP_GSM_RECOVERY_INTERVAL_MS 10000U
#define APP_GSM_DEFERRED_RETRY_DELAY_MS 30000U
/* 0 keeps retrying deferred event messages until delivery or reboot. */
#define APP_GSM_DEFERRED_RETRY_LIMIT 0U
#define APP_GSM_SENDER_TASK_STACK_SIZE 6144U
#define APP_GSM_SENDER_TASK_PRIORITY 5U
#define APP_NOTIFICATION_TASK_STACK_SIZE 5120U
#define APP_NOTIFICATION_TASK_PRIORITY 7U
#define APP_GSM_INBOX_POLL_ENABLED 0

// #define APP_NOTIFY_PHONE_NUMBER "+255688173415"
#define APP_NOTIFY_PHONE_NUMBER "+255656853836"
#define APP_STARTUP_SMS_TEXT "System started. Bulb & Fan automation active."

/* --------------------------------------------------------------------------
 * Configuration guards
 * -------------------------------------------------------------------------- */
#if APP_MANUAL_ON_HOLD_MS < 60000U
#error "APP_MANUAL_ON_HOLD_MS must be at least 60000 ms."
#endif

#if APP_NOTIFICATION_SAMPLE_COUNT == 0U
#error "APP_NOTIFICATION_SAMPLE_COUNT must be greater than zero."
#endif

#if APP_ENERGY_HISTORY_LENGTH < APP_NOTIFICATION_SAMPLE_COUNT
#error "Energy history must hold at least the notification sample count."
#endif

#if APP_CONTROL_POLL_INTERVAL_MS == 0U
#error "Control polling interval must be greater than zero."
#endif

#if APP_PIR_DEBOUNCE_MS < APP_CONTROL_POLL_INTERVAL_MS
#error "APP_PIR_DEBOUNCE_MS must be at least one control polling interval."
#endif

#endif /* APP_CONFIG_H */
