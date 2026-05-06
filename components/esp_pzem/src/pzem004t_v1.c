/**
 * @file pzem004t_v1.c
 * @brief Implementation of the PZEM-004T V1 Modbus-RTU driver.
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * See header. This file implements the low‑level Modbus communication.
 * All helper functions are static and invisible outside the driver.
 * =============================================================================
 */

#include "pzem004t_v1.h"

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "pzem_v1";

/* Modbus RTU constants for PZEM-004T V1 */
#define PZEM_SLAVE_ADDR         0x01    /**< Fixed slave address */
#define PZEM_BAUD_RATE          9600    /**< UART baud rate */
#define PZEM_UART_DATA_BITS     UART_DATA_8_BITS
#define PZEM_UART_PARITY        UART_PARITY_DISABLE
#define PZEM_UART_STOP_BITS     UART_STOP_BITS_1
#define PZEM_UART_FLOW_CTRL     UART_HW_FLOWCTRL_DISABLE

#define PZEM_READ_TIMEOUT_MS    100     /**< Timeout for Modbus response */
#define PZEM_BUF_SIZE           128     /**< Enough for a 9‑register reply */

/* Register addresses (V1.0 map) */
#define PZEM_REG_VOLTAGE        0x0000  /**< Voltage, uint16, 0.1 V/unit */
#define PZEM_REG_CURRENT_L      0x0001  /**< Current low word (32‑bit combined) */
#define PZEM_REG_POWER_L        0x0003  /**< Power low word */
#define PZEM_REG_ENERGY_L       0x0005  /**< Energy low word */
#define PZEM_REG_FREQUENCY      0x0007  /**< Frequency, uint16, 0.1 Hz/unit */
#define PZEM_REG_PF             0x0008  /**< Power factor, uint16, 0.01/unit */

#define PZEM_REG_COUNT          9       /**< Number of contiguous registers to read */

/* ---------- Modbus CRC ---------- */
/**
 * @brief Compute CRC-16 (Modbus) over a buffer.
 *
 * @param buf   Pointer to data.
 * @param len   Number of bytes.
 * @return      CRC-16 value (little‑endian ready).
 */
static uint16_t modbus_crc(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *buf++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ---------- Private helpers ---------- */
/**
 * @brief Send a Modbus read‑holding‑registers request.
 *
 * @param uart_num      UART port.
 * @param reg_addr      Starting register address.
 * @param reg_count     Number of registers to read.
 * @return ESP_OK on successful send.
 */
static esp_err_t pzem_send_request(uint8_t uart_num, uint16_t reg_addr, uint16_t reg_count)
{
    uint8_t tx_buf[8];
    tx_buf[0] = PZEM_SLAVE_ADDR;
    tx_buf[1] = 0x03;                // function code: read holding registers
    tx_buf[2] = (reg_addr >> 8) & 0xFF;
    tx_buf[3] = reg_addr & 0xFF;
    tx_buf[4] = (reg_count >> 8) & 0xFF;
    tx_buf[5] = reg_count & 0xFF;

    uint16_t crc = modbus_crc(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;

    const int written = uart_write_bytes(uart_num, tx_buf, sizeof(tx_buf));
    if (written != sizeof(tx_buf)) {
        ESP_LOGE(TAG, "UART write failed: %d/%u bytes", written, sizeof(tx_buf));
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Wait for and validate a Modbus response.
 *
 * Reads bytes until the expected length is received or timeout occurs.
 * Verifies CRC and slave address.
 *
 * @param uart_num   UART port.
 * @param rx_buf     Buffer to hold the response (must be at least 3 + 2*reg_count + 2 bytes).
 * @param reg_count  Number of registers requested.
 * @return ESP_OK if a valid response was received, error code otherwise.
 */
static esp_err_t pzem_receive_response(uint8_t uart_num, uint8_t *rx_buf, uint16_t reg_count)
{
    size_t exp_len = 3 + 2 * reg_count + 2;  // addr + func + bytecount + data + crc
    TickType_t start = xTaskGetTickCount();
    size_t received = 0;

    while (received < exp_len) {
        int len = uart_read_bytes(uart_num, rx_buf + received, exp_len - received,
                                  pdMS_TO_TICKS(50));
        if (len > 0) {
            received += len;
        }
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(PZEM_READ_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "Response timeout");
            return ESP_ERR_TIMEOUT;
        }
    }

    if (received != exp_len) {
        ESP_LOGE(TAG, "Invalid response length: %u (expected %u)", received, exp_len);
        return ESP_FAIL;
    }

    if (rx_buf[0] != PZEM_SLAVE_ADDR) {
        ESP_LOGE(TAG, "Wrong slave address: 0x%02X", rx_buf[0]);
        return ESP_FAIL;
    }
    if (rx_buf[1] != 0x03) {
        ESP_LOGE(TAG, "Unexpected function code: 0x%02X", rx_buf[1]);
        return ESP_FAIL;
    }

    uint16_t crc_rcv = (uint16_t)rx_buf[exp_len - 2] | ((uint16_t)rx_buf[exp_len - 1] << 8);
    if (modbus_crc(rx_buf, exp_len - 2) != crc_rcv) {
        ESP_LOGE(TAG, "CRC mismatch");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---------- Public API ---------- */
esp_err_t pzem_init(uint8_t uart_num, int tx_pin, int rx_pin)
{
    uart_config_t uart_config = {
        .baud_rate = PZEM_BAUD_RATE,
        .data_bits = PZEM_UART_DATA_BITS,
        .parity    = PZEM_UART_PARITY,
        .stop_bits = PZEM_UART_STOP_BITS,
        .flow_ctrl = PZEM_UART_FLOW_CTRL,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_driver_install(uart_num, PZEM_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Initialised on UART%d (TX:%d, RX:%d)", uart_num, tx_pin, rx_pin);
    return ESP_OK;
}

esp_err_t pzem_read_all(uint8_t uart_num, pzem_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Flush input in case of stale bytes */
    uart_flush_input(uart_num);

    esp_err_t err = pzem_send_request(uart_num, PZEM_REG_VOLTAGE, PZEM_REG_COUNT);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t rx_buf[PZEM_BUF_SIZE];
    err = pzem_receive_response(uart_num, rx_buf, PZEM_REG_COUNT);
    if (err != ESP_OK) {
        return err;
    }

    /* Data starts at byte 3 (byte count is at index 2) */
    const uint8_t *d = &rx_buf[3];

    /* Extract each register (all big‑endian) */
    uint16_t v_raw   = (d[0] << 8) | d[1];
    uint32_t c_raw   = ((uint32_t)(d[2] << 8 | d[3]))        // low word of current
                     | ((uint32_t)(d[4] << 8 | d[5]) << 16); // high word
    uint32_t p_raw   = ((uint32_t)(d[6] << 8 | d[7]))
                     | ((uint32_t)(d[8] << 8 | d[9]) << 16);
    uint32_t e_raw   = ((uint32_t)(d[10] << 8 | d[11]))
                     | ((uint32_t)(d[12] << 8 | d[13]) << 16);
    uint16_t f_raw   = (d[14] << 8) | d[15];
    uint16_t pf_raw  = (d[16] << 8) | d[17];

    /* Convert to engineering units */
    data->voltage   = v_raw / 10.0f;
    data->current   = c_raw / 1000.0f;   // mA -> A
    data->power     = p_raw / 10.0f;     // 0.1 W -> W
    data->energy    = (float)e_raw;      // Wh
    data->frequency = f_raw / 10.0f;     // 0.1 Hz -> Hz
    data->pf        = pf_raw / 100.0f;   // 0.01 -> 0..1

    ESP_LOGI(TAG, "Read: U=%.1fV I=%.3fA P=%.1fW E=%.0fWh F=%.1fHz PF=%.2f",
             data->voltage, data->current, data->power,
             data->energy, data->frequency, data->pf);

    return ESP_OK;
}