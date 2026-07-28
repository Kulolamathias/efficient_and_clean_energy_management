/**
 * @file energy_service.c
 * @brief INA219 acquisition, accurate energy integration, and sample history.
 * @author Matthithyahu
 */

#include "energy_service.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "app_config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ina219.h"
#include "nvs.h"

static const char *TAG = "energy";

#define ENERGY_NVS_NAMESPACE "energy"
#define ENERGY_NVS_BULB_KEY "bulb_uwh"
#define ENERGY_NVS_FAN_KEY "fan_uwh"
#define ENERGY_MICROWH_PER_WH 1000000.0
#define ENERGY_MILLISECONDS_PER_HOUR 3600000.0

/** @brief Per-load sensor state owned by the monitoring task. */
typedef struct {
  ina219_handle_t sensor;
  app_energy_snapshot_t latest;
  app_energy_snapshot_t history[APP_ENERGY_HISTORY_LENGTH];
  size_t history_head;
  size_t history_count;
  double energy_wh;
  double last_persisted_wh;
  uint64_t last_sample_ms;
  uint64_t next_init_attempt_ms;
  uint32_t read_failures;
  uint32_t sample_sequence;
} energy_channel_t;

/** @brief Energy-service singleton context. */
typedef struct {
  energy_channel_t channels[APP_LOAD_COUNT];
  SemaphoreHandle_t data_lock;
  nvs_handle_t nvs_handle;
  bool nvs_open;
  uint64_t last_persist_ms;
  bool initialized;
} energy_context_t;

static energy_context_t s_energy = {0};

/** @brief Return the configured I2C address for a load. */
static uint8_t sensor_address(app_load_id_t load_id) {
  return load_id == APP_LOAD_BULB ? APP_INA219_BULB_ADDRESS
                                  : APP_INA219_FAN_ADDRESS;
}

/** @brief Return the NVS key for a load's accumulated total. */
static const char *energy_nvs_key(app_load_id_t load_id) {
  return load_id == APP_LOAD_BULB ? ENERGY_NVS_BULB_KEY : ENERGY_NVS_FAN_KEY;
}

/** @brief Initialize one sensor using the centralized application settings. */
static esp_err_t initialize_sensor(app_load_id_t load_id, uint64_t now_ms) {
  energy_channel_t *channel = &s_energy.channels[load_id];
  if (channel->sensor != NULL) {
    return ESP_OK;
  }

  const ina219_config_t configuration = {
      .i2c_port = APP_I2C_PORT,
      .i2c_addr = sensor_address(load_id),
      .shunt_resistance_ohm = APP_INA219_SHUNT_RESISTANCE_OHM,
      .max_expected_current_a = APP_INA219_MAX_EXPECTED_CURRENT_A,
      .bus_range = INA219_BUS_RANGE_32V,
      .gain = INA219_GAIN_320MV,
      .adc_average = (ina219_adc_average_t)APP_INA219_ADC_AVERAGE_COUNT,
      .i2c_timeout_ms = APP_I2C_TRANSACTION_TIMEOUT_MS,
  };

  const esp_err_t result = ina219_init(&configuration, &channel->sensor);
  if (result != ESP_OK) {
    channel->sensor = NULL;
    channel->next_init_attempt_ms = now_ms + APP_SENSOR_RETRY_INTERVAL_MS;
    ESP_LOGW(TAG, "%s INA219 at 0x%02X is offline; retry scheduled",
             load_id == APP_LOAD_BULB ? "Bulb" : "Fan", configuration.i2c_addr);
    return result;
  }

  channel->read_failures = 0U;
  channel->last_sample_ms = 0U;
  channel->next_init_attempt_ms = 0U;
  return ESP_OK;
}

/** @brief Push a successful sample into the chronological ring buffer. */
static void append_history(energy_channel_t *channel,
                           const app_energy_snapshot_t *sample) {
  channel->history[channel->history_head] = *sample;
  channel->history_head =
      (channel->history_head + 1U) % APP_ENERGY_HISTORY_LENGTH;
  if (channel->history_count < APP_ENERGY_HISTORY_LENGTH) {
    ++channel->history_count;
  }
}

/** @brief Mark a channel offline without destroying its accumulated energy. */
static void mark_read_failure(app_load_id_t load_id, uint64_t now_ms) {
  energy_channel_t *channel = &s_energy.channels[load_id];
  ++channel->read_failures;
  channel->last_sample_ms = 0U;

  xSemaphoreTake(s_energy.data_lock, portMAX_DELAY);
  channel->latest.sensor_online = false;
  xSemaphoreGive(s_energy.data_lock);

  if (channel->read_failures >= APP_SENSOR_FAILURES_BEFORE_REINIT) {
    ESP_LOGW(TAG,
             "%s INA219 failed %" PRIu32
             " consecutive reads; reinitializing later",
             load_id == APP_LOAD_BULB ? "Bulb" : "Fan", channel->read_failures);
    ina219_deinit(channel->sensor);
    channel->sensor = NULL;
    channel->next_init_attempt_ms = now_ms + APP_SENSOR_RETRY_INTERVAL_MS;
    channel->read_failures = 0U;
  }
}

/** @brief Convert a persisted micro-watt-hour counter to working units. */
static void restore_energy_totals(void) {
  if (!s_energy.nvs_open) {
    return;
  }

  for (size_t index = 0U; index < APP_LOAD_COUNT; ++index) {
    uint64_t stored_uwh = 0U;
    const esp_err_t result = nvs_get_u64(
        s_energy.nvs_handle, energy_nvs_key((app_load_id_t)index), &stored_uwh);
    if (result == ESP_OK) {
      const double restored_wh = (double)stored_uwh / ENERGY_MICROWH_PER_WH;
      s_energy.channels[index].energy_wh = restored_wh;
      s_energy.channels[index].last_persisted_wh = restored_wh;
      s_energy.channels[index].latest.energy_kwh = restored_wh / 1000.0;
    }
  }
}

/** @brief Persist changed totals at a wear-controlled interval. */
static void persist_energy_if_due(uint64_t now_ms) {
#if APP_ENERGY_PERSISTENCE_ENABLED
  if (!s_energy.nvs_open ||
      ((now_ms - s_energy.last_persist_ms) < APP_ENERGY_PERSIST_INTERVAL_MS)) {
    return;
  }
  s_energy.last_persist_ms = now_ms;

  bool changed = false;
  for (size_t index = 0U; index < APP_LOAD_COUNT; ++index) {
    const energy_channel_t *channel = &s_energy.channels[index];
    if (fabs(channel->energy_wh - channel->last_persisted_wh) >=
        APP_ENERGY_PERSIST_MINIMUM_CHANGE_WH) {
      changed = true;
      break;
    }
  }
  if (!changed) {
    return;
  }

  for (size_t index = 0U; index < APP_LOAD_COUNT; ++index) {
    const uint64_t energy_uwh = (uint64_t)llround(
        s_energy.channels[index].energy_wh * ENERGY_MICROWH_PER_WH);
    if (nvs_set_u64(s_energy.nvs_handle, energy_nvs_key((app_load_id_t)index),
                    energy_uwh) != ESP_OK) {
      ESP_LOGW(TAG, "Could not stage energy total for NVS");
      return;
    }
  }

  if (nvs_commit(s_energy.nvs_handle) == ESP_OK) {
    for (size_t index = 0U; index < APP_LOAD_COUNT; ++index) {
      s_energy.channels[index].last_persisted_wh =
          s_energy.channels[index].energy_wh;
    }
    ESP_LOGI(TAG, "Energy totals saved to NVS");
  } else {
    ESP_LOGW(TAG, "Energy NVS commit failed");
  }
#else
  (void)now_ms;
#endif
}

esp_err_t energy_service_init(void) {
  if (s_energy.initialized) {
    return ESP_OK;
  }

  s_energy.data_lock = xSemaphoreCreateMutex();
  if (s_energy.data_lock == NULL) {
    return ESP_ERR_NO_MEM;
  }

#if APP_ENERGY_PERSISTENCE_ENABLED
  if (nvs_open(ENERGY_NVS_NAMESPACE, NVS_READWRITE, &s_energy.nvs_handle) ==
      ESP_OK) {
    s_energy.nvs_open = true;
    restore_energy_totals();
  } else {
    ESP_LOGW(TAG, "Energy persistence unavailable; monitoring will continue");
  }
#endif

  s_energy.initialized = true;
  ESP_LOGI(TAG, "Energy service initialized");
  return ESP_OK;
}

esp_err_t energy_service_sample(uint64_t now_ms,
                                const bool relay_states[APP_LOAD_COUNT]) {
  if (!s_energy.initialized || relay_states == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  for (size_t index = 0U; index < APP_LOAD_COUNT; ++index) {
    const app_load_id_t load_id = (app_load_id_t)index;
    energy_channel_t *channel = &s_energy.channels[index];

    if ((channel->sensor == NULL) &&
        (now_ms >= channel->next_init_attempt_ms)) {
      (void)initialize_sensor(load_id, now_ms);
    }
    if (channel->sensor == NULL) {
      continue;
    }

    ina219_data_t measured = {0};
    const esp_err_t read_result = ina219_read(channel->sensor, &measured);
    if (read_result != ESP_OK) {
      mark_read_failure(load_id, now_ms);
      continue;
    }

    channel->read_failures = 0U;
    if ((channel->last_sample_ms != 0U) && (now_ms > channel->last_sample_ms)) {
      const double elapsed_hours = (double)(now_ms - channel->last_sample_ms) /
                                   ENERGY_MILLISECONDS_PER_HOUR;
      channel->energy_wh += (double)measured.power_w * elapsed_hours;
    }
    channel->last_sample_ms = now_ms;

    const app_energy_snapshot_t sample = {
        .sensor_online = true,
        .relay_on_at_sample = relay_states[index],
        .voltage_v = measured.bus_voltage_v,
        .current_a = measured.current_a,
        .power_w = measured.power_w,
        .energy_kwh = channel->energy_wh / 1000.0,
        .sample_time_ms = now_ms,
        .sequence = ++channel->sample_sequence,
    };

    xSemaphoreTake(s_energy.data_lock, portMAX_DELAY);
    channel->latest = sample;
    append_history(channel, &sample);
    xSemaphoreGive(s_energy.data_lock);
  }

  persist_energy_if_due(now_ms);
  return ESP_OK;
}

esp_err_t energy_service_get_snapshot(app_load_id_t load_id,
                                      app_energy_snapshot_t *snapshot) {
  if ((load_id < APP_LOAD_BULB) || (load_id >= APP_LOAD_COUNT) ||
      (snapshot == NULL)) {
    return ESP_ERR_INVALID_ARG;
  }

  xSemaphoreTake(s_energy.data_lock, portMAX_DELAY);
  *snapshot = s_energy.channels[load_id].latest;
  xSemaphoreGive(s_energy.data_lock);
  return ESP_OK;
}

esp_err_t energy_service_get_averaged_snapshot(
    app_load_id_t load_id, bool relay_on, uint64_t earliest_time_ms,
    uint64_t latest_time_ms, size_t sample_count,
    app_energy_snapshot_t *snapshot) {
  if ((load_id < APP_LOAD_BULB) || (load_id >= APP_LOAD_COUNT) ||
      (snapshot == NULL) || (sample_count == 0U) ||
      (sample_count > APP_ENERGY_HISTORY_LENGTH) ||
      (latest_time_ms < earliest_time_ms)) {
    return ESP_ERR_INVALID_ARG;
  }

  float voltage_sum = 0.0f;
  float current_sum = 0.0f;
  float power_sum = 0.0f;
  size_t found = 0U;
  app_energy_snapshot_t newest = {0};

  xSemaphoreTake(s_energy.data_lock, portMAX_DELAY);
  const energy_channel_t *channel = &s_energy.channels[load_id];
  const size_t oldest = (channel->history_head + APP_ENERGY_HISTORY_LENGTH -
                         channel->history_count) %
                        APP_ENERGY_HISTORY_LENGTH;

  for (size_t offset = 0U; offset < channel->history_count; ++offset) {
    const size_t position = (oldest + offset) % APP_ENERGY_HISTORY_LENGTH;
    const app_energy_snapshot_t *sample = &channel->history[position];
    if ((sample->sample_time_ms < earliest_time_ms) ||
        (sample->sample_time_ms > latest_time_ms) || !sample->sensor_online ||
        (sample->relay_on_at_sample != relay_on)) {
      continue;
    }

    voltage_sum += sample->voltage_v;
    current_sum += sample->current_a;
    power_sum += sample->power_w;
    newest = *sample;
    if (++found == sample_count) {
      break;
    }
  }
  xSemaphoreGive(s_energy.data_lock);

  if (found < sample_count) {
    return ESP_ERR_NOT_FOUND;
  }

  *snapshot = newest;
  snapshot->voltage_v = voltage_sum / (float)found;
  snapshot->current_a = current_sum / (float)found;
  snapshot->power_w = power_sum / (float)found;
  snapshot->sensor_online = true;
  return ESP_OK;
}
