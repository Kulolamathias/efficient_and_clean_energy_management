/**
 * @file ina219.c
 * @brief INA219 driver – uses configured I2C address (no hard‑coded 0x40).
 */

#include "ina219.h"
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "ina219"
#define INA_I2C_PORT        I2C_NUM_0

/* Register addresses */
#define REG_CONFIG          0x00
#define REG_SHUNT_VOLTAGE   0x01
#define REG_BUS_VOLTAGE     0x02
#define REG_POWER           0x03
#define REG_CURRENT         0x04
#define REG_CALIBRATION     0x05

/* Configuration bits */
#define CONFIG_RST          0x8000
#define CONFIG_BRNG_32V     0x0000
#define CONFIG_GAIN_1       0x0000
#define CONFIG_BADC_12BIT   0x0180
#define CONFIG_SADC_12BIT   0x0078
#define CONFIG_MODE_SANDBVOLT_CONTINUOUS 0x0007

static float shunt_resistance;
static float current_lsb;
static uint8_t i2c_addr;            /**< Device address actually used */
static bool initialized = false;

/* ================================================================
 * Private helpers – use the stored i2c_addr, not a hard‑coded 0x40
 * ================================================================ */
static esp_err_t write_reg(uint8_t reg, uint16_t value) {
    uint8_t buf[3] = {reg, (value >> 8) & 0xFF, value & 0xFF};
    return i2c_master_write_to_device(INA_I2C_PORT, i2c_addr, buf, sizeof(buf),
                                      pdMS_TO_TICKS(100));
}

static esp_err_t read_reg(uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    uint8_t cmd = reg;
    esp_err_t err = i2c_master_write_read_device(INA_I2C_PORT, i2c_addr,
                                                 &cmd, 1, data, 2,
                                                 pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        *value = (data[0] << 8) | data[1];
    }
    return err;
}

/* ================================================================
 * Public API
 * ================================================================ */
esp_err_t ina219_init(const ina219_config_t *config) {
    if (!config || config->shunt_resistance <= 0.0f || config->max_current <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_addr = config->i2c_addr;        // Store device address
    shunt_resistance = config->shunt_resistance;
    float max_shunt_mv = 40.0f;          // ±40 mV with gain=1 (for info only, not used)
    current_lsb = config->max_current / 32767.0f;
    uint16_t cal = (uint16_t)(0.04096f / (current_lsb * shunt_resistance));

    // Reset
    write_reg(REG_CONFIG, CONFIG_RST);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Calibration
    write_reg(REG_CALIBRATION, cal);

    // Configuration: 32V range, gain=1, 12‑bit, continuous shunt+voltage
    uint16_t conf = CONFIG_BRNG_32V | CONFIG_GAIN_1 |
                    CONFIG_BADC_12BIT | CONFIG_SADC_12BIT |
                    CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    esp_err_t err = write_reg(REG_CONFIG, conf);
    if (err == ESP_OK) {
        initialized = true;
        ESP_LOGI(TAG, "Initialised at address 0x%02X", i2c_addr);
    } else {
        ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ina219_read(ina219_data_t *data) {
    if (!initialized || !data) return ESP_ERR_INVALID_STATE;

    uint16_t raw;

    if (read_reg(REG_BUS_VOLTAGE, &raw) != ESP_OK) return ESP_FAIL;
    data->bus_voltage = (raw >> 3) * 0.004f;

    if (read_reg(REG_SHUNT_VOLTAGE, &raw) != ESP_OK) return ESP_FAIL;
    int16_t shunt_raw = (int16_t)raw;
    data->shunt_voltage = shunt_raw * 0.01f;

    if (read_reg(REG_CURRENT, &raw) != ESP_OK) return ESP_FAIL;
    data->current = (int16_t)raw * current_lsb;

    if (read_reg(REG_POWER, &raw) != ESP_OK) return ESP_FAIL;
    data->power = raw * current_lsb * 20.0f;

    return ESP_OK;
}