/**
 * @file ina219.h
 * @brief Configurable instance-based INA219 current and power monitor driver.
 * @author Matthithyahu
 */

#ifndef INA219_H
#define INA219_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque INA219 device handle. */
typedef struct ina219_device *ina219_handle_t;

/** @brief Supported bus-voltage full-scale ranges. */
typedef enum {
  INA219_BUS_RANGE_16V = 0,
  INA219_BUS_RANGE_32V
} ina219_bus_range_t;

/** @brief Supported shunt PGA full-scale ranges. */
typedef enum {
  INA219_GAIN_40MV = 0,
  INA219_GAIN_80MV,
  INA219_GAIN_160MV,
  INA219_GAIN_320MV
} ina219_gain_t;

/** @brief Supported ADC hardware averaging counts. */
typedef enum {
  INA219_ADC_AVERAGE_1 = 1,
  INA219_ADC_AVERAGE_2 = 2,
  INA219_ADC_AVERAGE_4 = 4,
  INA219_ADC_AVERAGE_8 = 8,
  INA219_ADC_AVERAGE_16 = 16,
  INA219_ADC_AVERAGE_32 = 32,
  INA219_ADC_AVERAGE_64 = 64,
  INA219_ADC_AVERAGE_128 = 128
} ina219_adc_average_t;

/** @brief Per-device INA219 configuration. */
typedef struct {
  i2c_port_t i2c_port;              /**< Installed ESP-IDF I2C port. */
  uint8_t i2c_addr;                 /**< Seven-bit I2C address. */
  float shunt_resistance_ohm;       /**< External shunt resistance in ohms. */
  float max_expected_current_a;     /**< Maximum expected current in amperes. */
  ina219_bus_range_t bus_range;     /**< Bus voltage full-scale range. */
  ina219_gain_t gain;               /**< Shunt PGA full-scale range. */
  ina219_adc_average_t adc_average; /**< Bus and shunt sample averaging. */
  uint32_t i2c_timeout_ms;          /**< I2C transaction timeout. */
} ina219_config_t;

/** @brief One coherent set of INA219 register values. */
typedef struct {
  float bus_voltage_v;    /**< Bus voltage in volts. */
  float shunt_voltage_mv; /**< Shunt voltage in millivolts. */
  float current_a;        /**< Calculated current in amperes. */
  float power_w;          /**< Calculated power in watts. */
  bool conversion_ready;  /**< Latest conversion-ready flag. */
  bool math_overflow;     /**< INA219 math-overflow flag. */
} ina219_data_t;

/**
 * @brief Initialize and calibrate one INA219 sensor.
 *
 * @param[in] config Complete device configuration.
 * @param[out] handle Receives an allocated device handle on success.
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
esp_err_t ina219_init(const ina219_config_t *config, ina219_handle_t *handle);

/**
 * @brief Read bus voltage, shunt voltage, current, and power registers.
 *
 * @param[in] handle Initialized device handle.
 * @param[out] data Destination for converted measurements.
 * @return ESP_OK on success; ESP_ERR_INVALID_STATE on math overflow.
 */
esp_err_t ina219_read(ina219_handle_t handle, ina219_data_t *data);

/**
 * @brief Release a previously initialized sensor handle.
 *
 * @param[in] handle Device handle, or NULL.
 */
void ina219_deinit(ina219_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* INA219_H */
