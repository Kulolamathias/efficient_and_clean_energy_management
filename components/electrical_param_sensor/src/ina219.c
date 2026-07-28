/**
 * @file ina219.c
 * @brief Register-level INA219 implementation with checked configuration.
 * @author Matthithyahu
 *
 * The previous implementation labelled the 16 V and 128-sample settings as
 * 32 V and 12-bit single-sample settings. This implementation uses explicit
 * masks from the INA219 data sheet and checks every initialization write.
 */

#include "ina219.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ina219";

/* INA219 register addresses. */
#define INA219_REG_CONFIG 0x00U
#define INA219_REG_SHUNT_VOLTAGE 0x01U
#define INA219_REG_BUS_VOLTAGE 0x02U
#define INA219_REG_POWER 0x03U
#define INA219_REG_CURRENT 0x04U
#define INA219_REG_CALIBRATION 0x05U

/* Configuration register masks. */
#define INA219_CONFIG_RESET 0x8000U
#define INA219_CONFIG_BUS_RANGE_16V 0x0000U
#define INA219_CONFIG_BUS_RANGE_32V 0x2000U
#define INA219_CONFIG_GAIN_40MV 0x0000U
#define INA219_CONFIG_GAIN_80MV 0x0800U
#define INA219_CONFIG_GAIN_160MV 0x1000U
#define INA219_CONFIG_GAIN_320MV 0x1800U
#define INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS 0x0007U

/* ADC resolution and averaging encodings for bus bits [10:7]. */
#define INA219_BADC_12BIT_1_SAMPLE 0x0180U
#define INA219_BADC_12BIT_2_SAMPLES 0x0480U
#define INA219_BADC_12BIT_4_SAMPLES 0x0500U
#define INA219_BADC_12BIT_8_SAMPLES 0x0580U
#define INA219_BADC_12BIT_16_SAMPLES 0x0600U
#define INA219_BADC_12BIT_32_SAMPLES 0x0680U
#define INA219_BADC_12BIT_64_SAMPLES 0x0700U
#define INA219_BADC_12BIT_128_SAMPLES 0x0780U

/* ADC resolution and averaging encodings for shunt bits [6:3]. */
#define INA219_SADC_12BIT_1_SAMPLE 0x0018U
#define INA219_SADC_12BIT_2_SAMPLES 0x0048U
#define INA219_SADC_12BIT_4_SAMPLES 0x0050U
#define INA219_SADC_12BIT_8_SAMPLES 0x0058U
#define INA219_SADC_12BIT_16_SAMPLES 0x0060U
#define INA219_SADC_12BIT_32_SAMPLES 0x0068U
#define INA219_SADC_12BIT_64_SAMPLES 0x0070U
#define INA219_SADC_12BIT_128_SAMPLES 0x0078U

/* Conversion and calibration constants from the data sheet. */
#define INA219_CALIBRATION_NUMERATOR 0.04096f
#define INA219_BUS_VOLTAGE_LSB_V 0.004f
#define INA219_SHUNT_VOLTAGE_LSB_MV 0.01f
#define INA219_POWER_LSB_MULTIPLIER 20.0f
#define INA219_BUS_STATUS_CONVERSION_READY 0x0002U
#define INA219_BUS_STATUS_MATH_OVERFLOW 0x0001U
#define INA219_RESET_DELAY_MS 50U
#define INA219_REGISTER_MAX 65535.0f
#define INA219_SIGNED_REGISTER_COUNTS 32768.0f

/** @brief Runtime device data hidden behind the public handle. */
struct ina219_device {
  i2c_port_t i2c_port;
  uint8_t i2c_addr;
  uint32_t i2c_timeout_ms;
  float current_lsb_a;
};

/** @brief Write a big-endian 16-bit register value. */
static esp_err_t write_register(const struct ina219_device *device,
                                uint8_t register_address, uint16_t value) {
  const uint8_t bytes[] = {
      register_address,
      (uint8_t)(value >> 8U),
      (uint8_t)(value & 0xFFU),
  };
  return i2c_master_write_to_device(device->i2c_port, device->i2c_addr, bytes,
                                    sizeof(bytes),
                                    pdMS_TO_TICKS(device->i2c_timeout_ms));
}

/** @brief Read a big-endian 16-bit register value. */
static esp_err_t read_register(const struct ina219_device *device,
                               uint8_t register_address, uint16_t *value) {
  uint8_t bytes[2] = {0U};
  ESP_RETURN_ON_ERROR(i2c_master_write_read_device(
                          device->i2c_port, device->i2c_addr, &register_address,
                          sizeof(register_address), bytes, sizeof(bytes),
                          pdMS_TO_TICKS(device->i2c_timeout_ms)),
                      TAG, "I2C read failed at address 0x%02X",
                      device->i2c_addr);
  *value = (uint16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);
  return ESP_OK;
}

/** @brief Convert a public bus range to its register encoding. */
static esp_err_t bus_range_bits(ina219_bus_range_t range, uint16_t *bits) {
  switch (range) {
    case INA219_BUS_RANGE_16V:
      *bits = INA219_CONFIG_BUS_RANGE_16V;
      return ESP_OK;
    case INA219_BUS_RANGE_32V:
      *bits = INA219_CONFIG_BUS_RANGE_32V;
      return ESP_OK;
    default:
      return ESP_ERR_INVALID_ARG;
  }
}

/** @brief Convert a public PGA setting to its register encoding. */
static esp_err_t gain_bits(ina219_gain_t gain, uint16_t *bits) {
  switch (gain) {
    case INA219_GAIN_40MV:
      *bits = INA219_CONFIG_GAIN_40MV;
      return ESP_OK;
    case INA219_GAIN_80MV:
      *bits = INA219_CONFIG_GAIN_80MV;
      return ESP_OK;
    case INA219_GAIN_160MV:
      *bits = INA219_CONFIG_GAIN_160MV;
      return ESP_OK;
    case INA219_GAIN_320MV:
      *bits = INA219_CONFIG_GAIN_320MV;
      return ESP_OK;
    default:
      return ESP_ERR_INVALID_ARG;
  }
}

/** @brief Convert an averaging count to bus and shunt register encodings. */
static esp_err_t adc_bits(ina219_adc_average_t average, uint16_t *bus_bits,
                          uint16_t *shunt_bits) {
  switch (average) {
    case INA219_ADC_AVERAGE_1:
      *bus_bits = INA219_BADC_12BIT_1_SAMPLE;
      *shunt_bits = INA219_SADC_12BIT_1_SAMPLE;
      return ESP_OK;
    case INA219_ADC_AVERAGE_2:
      *bus_bits = INA219_BADC_12BIT_2_SAMPLES;
      *shunt_bits = INA219_SADC_12BIT_2_SAMPLES;
      return ESP_OK;
    case INA219_ADC_AVERAGE_4:
      *bus_bits = INA219_BADC_12BIT_4_SAMPLES;
      *shunt_bits = INA219_SADC_12BIT_4_SAMPLES;
      return ESP_OK;
    case INA219_ADC_AVERAGE_8:
      *bus_bits = INA219_BADC_12BIT_8_SAMPLES;
      *shunt_bits = INA219_SADC_12BIT_8_SAMPLES;
      return ESP_OK;
    case INA219_ADC_AVERAGE_16:
      *bus_bits = INA219_BADC_12BIT_16_SAMPLES;
      *shunt_bits = INA219_SADC_12BIT_16_SAMPLES;
      return ESP_OK;
    case INA219_ADC_AVERAGE_32:
      *bus_bits = INA219_BADC_12BIT_32_SAMPLES;
      *shunt_bits = INA219_SADC_12BIT_32_SAMPLES;
      return ESP_OK;
    case INA219_ADC_AVERAGE_64:
      *bus_bits = INA219_BADC_12BIT_64_SAMPLES;
      *shunt_bits = INA219_SADC_12BIT_64_SAMPLES;
      return ESP_OK;
    case INA219_ADC_AVERAGE_128:
      *bus_bits = INA219_BADC_12BIT_128_SAMPLES;
      *shunt_bits = INA219_SADC_12BIT_128_SAMPLES;
      return ESP_OK;
    default:
      return ESP_ERR_INVALID_ARG;
  }
}

esp_err_t ina219_init(const ina219_config_t *config, ina219_handle_t *handle) {
  if ((config == NULL) || (handle == NULL) ||
      (config->shunt_resistance_ohm <= 0.0f) ||
      (config->max_expected_current_a <= 0.0f) ||
      (config->i2c_timeout_ms == 0U)) {
    return ESP_ERR_INVALID_ARG;
  }
  *handle = NULL;

  uint16_t range = 0U;
  uint16_t gain = 0U;
  uint16_t bus_adc = 0U;
  uint16_t shunt_adc = 0U;
  ESP_RETURN_ON_ERROR(bus_range_bits(config->bus_range, &range), TAG,
                      "Bad bus range");
  ESP_RETURN_ON_ERROR(gain_bits(config->gain, &gain), TAG, "Bad gain");
  ESP_RETURN_ON_ERROR(adc_bits(config->adc_average, &bus_adc, &shunt_adc), TAG,
                      "Bad ADC average");

  struct ina219_device *device = calloc(1U, sizeof(*device));
  if (device == NULL) {
    return ESP_ERR_NO_MEM;
  }
  device->i2c_port = config->i2c_port;
  device->i2c_addr = config->i2c_addr;
  device->i2c_timeout_ms = config->i2c_timeout_ms;
  device->current_lsb_a =
      config->max_expected_current_a / INA219_SIGNED_REGISTER_COUNTS;

  const float calibration_float =
      INA219_CALIBRATION_NUMERATOR /
      (device->current_lsb_a * config->shunt_resistance_ohm);
  if (!isfinite(calibration_float) || (calibration_float < 1.0f) ||
      (calibration_float > INA219_REGISTER_MAX)) {
    free(device);
    return ESP_ERR_INVALID_ARG;
  }
  const uint16_t calibration = (uint16_t)calibration_float;

  esp_err_t result =
      write_register(device, INA219_REG_CONFIG, INA219_CONFIG_RESET);
  if (result == ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(INA219_RESET_DELAY_MS));
    result = write_register(device, INA219_REG_CALIBRATION, calibration);
  }
  if (result == ESP_OK) {
    const uint16_t configuration = range | gain | bus_adc | shunt_adc |
                                   INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS;
    result = write_register(device, INA219_REG_CONFIG, configuration);
  }

  if (result != ESP_OK) {
    ESP_LOGE(TAG, "Initialization failed at 0x%02X: %s", config->i2c_addr,
             esp_err_to_name(result));
    free(device);
    return result;
  }

  *handle = device;
  ESP_LOGI(TAG, "INA219 0x%02X ready: 32V/320mV, %u-sample averaging",
           device->i2c_addr, (unsigned)config->adc_average);
  return ESP_OK;
}

esp_err_t ina219_read(ina219_handle_t handle, ina219_data_t *data) {
  if ((handle == NULL) || (data == NULL)) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(data, 0, sizeof(*data));
  uint16_t raw_bus = 0U;
  uint16_t raw_shunt = 0U;
  uint16_t raw_current = 0U;
  uint16_t raw_power = 0U;

  ESP_RETURN_ON_ERROR(read_register(handle, INA219_REG_BUS_VOLTAGE, &raw_bus),
                      TAG, "Bus read failed");
  ESP_RETURN_ON_ERROR(
      read_register(handle, INA219_REG_SHUNT_VOLTAGE, &raw_shunt), TAG,
      "Shunt read failed");
  ESP_RETURN_ON_ERROR(read_register(handle, INA219_REG_CURRENT, &raw_current),
                      TAG, "Current read failed");
  ESP_RETURN_ON_ERROR(read_register(handle, INA219_REG_POWER, &raw_power), TAG,
                      "Power read failed");

  data->conversion_ready = (raw_bus & INA219_BUS_STATUS_CONVERSION_READY) != 0U;
  data->math_overflow = (raw_bus & INA219_BUS_STATUS_MATH_OVERFLOW) != 0U;
  data->bus_voltage_v = (float)(raw_bus >> 3U) * INA219_BUS_VOLTAGE_LSB_V;
  data->shunt_voltage_mv =
      (float)(int16_t)raw_shunt * INA219_SHUNT_VOLTAGE_LSB_MV;
  data->current_a = (float)(int16_t)raw_current * handle->current_lsb_a;
  data->power_w =
      (float)raw_power * handle->current_lsb_a * INA219_POWER_LSB_MULTIPLIER;

  return data->math_overflow ? ESP_ERR_INVALID_STATE : ESP_OK;
}

void ina219_deinit(ina219_handle_t handle) { free(handle); }
