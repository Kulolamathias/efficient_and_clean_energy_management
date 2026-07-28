/**
 * @file notification_service.c
 * @brief Fresh electrical snapshots plus serialized, retrying SIM800 delivery.
 * @author Matthithyahu
 *
 * Snapshot preparation and modem transmission are intentionally separate.
 * Electrical values are captured near the relay event even while an earlier
 * SMS is still being delivered. The proven blocking SIM800 send sequence then
 * runs in one dedicated task, preserving its reliable UART timing.
 */

#include "notification_service.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_types.h"
#include "control_service.h"
#include "energy_service.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gsm_sim800.h"

static const char *TAG = "notification";

/** @brief Fully prepared message waiting only for modem delivery. */
typedef struct {
  app_relay_event_t event;
  char text[APP_SMS_MESSAGE_MAX_LENGTH + 1U];
  uint32_t deferred_retry_count;
} prepared_sms_t;

/** @brief Notification service singleton state. */
typedef struct {
  QueueHandle_t relay_event_queue;
  QueueHandle_t prepared_sms_queue;
  TaskHandle_t preparation_task;
  TaskHandle_t sender_task;
  volatile bool gsm_ready;
  bool initialized;
} notification_context_t;

static notification_context_t s_notification = {0};

/** @brief Return monotonic milliseconds since ESP32 startup. */
static uint64_t monotonic_ms(void) {
  return (uint64_t)(esp_timer_get_time() / 1000LL);
}

/** @brief Build a bounded message with either fresh values or an honest fault.
 */
static void format_event_message(const app_relay_event_t *event,
                                 const app_energy_snapshot_t *energy,
                                 prepared_sms_t *prepared) {
  prepared->event = *event;
  const char *load_name = control_service_load_name(event->load_id);
  const char *state_name = event->relay_on ? "ON" : "OFF";
  const char *reason = control_service_reason_text(event->reason);

  if ((energy != NULL) && energy->sensor_online) {
    (void)snprintf(prepared->text, sizeof(prepared->text),
                   "%s %s (%s)\n"
                   "V:%.1fV I:%.3fA P:%.1fW\n"
                   "E:%.6fkWh",
                   load_name, state_name, reason, energy->voltage_v,
                   energy->current_a, energy->power_w, energy->energy_kwh);
  } else {
    (void)snprintf(prepared->text, sizeof(prepared->text),
                   "%s %s (%s)\nSensor data not fresh", load_name, state_name,
                   reason);
  }
}

/** @brief Wait only in this task for relay settling and three matching samples.
 */
static void prepare_one_event(const app_relay_event_t *event,
                              prepared_sms_t *prepared) {
  const uint32_t settle_ms = event->relay_on ? APP_NOTIFICATION_ON_SETTLE_MS
                                             : APP_NOTIFICATION_OFF_SETTLE_MS;
  const uint64_t earliest_sample_ms = event->event_time_ms + settle_ms;
  const uint64_t deadline_ms =
      earliest_sample_ms + APP_NOTIFICATION_SAMPLE_TIMEOUT_MS;
  app_energy_snapshot_t energy = {0};
  esp_err_t snapshot_result = ESP_ERR_NOT_FOUND;

  do {
    snapshot_result = energy_service_get_averaged_snapshot(
        event->load_id, event->relay_on, earliest_sample_ms, deadline_ms,
        APP_NOTIFICATION_SAMPLE_COUNT, &energy);
    if (snapshot_result == ESP_OK) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(APP_NOTIFICATION_SAMPLE_POLL_MS));
  } while (monotonic_ms() <= deadline_ms);

  if (snapshot_result != ESP_OK) {
    ESP_LOGW(TAG, "No fresh stable sensor window for %s %s event %lu",
             control_service_load_name(event->load_id),
             event->relay_on ? "ON" : "OFF", (unsigned long)event->sequence);
    format_event_message(event, NULL, prepared);
    return;
  }

  ESP_LOGI(TAG, "Event %lu snapshot: %.1fV %.3fA %.1fW at +%lums",
           (unsigned long)event->sequence, energy.voltage_v, energy.current_a,
           energy.power_w,
           (unsigned long)(energy.sample_time_ms - event->event_time_ms));
  format_event_message(event, &energy, prepared);
}

/** @brief Consume relay events promptly, independent of slow SMS delivery. */
static void notification_preparation_task(void *argument) {
  (void)argument;
  app_relay_event_t event;

  while (true) {
    if (xQueueReceive(s_notification.relay_event_queue, &event,
                      portMAX_DELAY) != pdTRUE) {
      continue;
    }

    prepared_sms_t prepared = {0};
    prepare_one_event(&event, &prepared);
    /* Blocking here is intentional: never discard a prepared event message.
     * The high-priority control task uses a separate, generously sized queue.
     */
    (void)xQueueSend(s_notification.prepared_sms_queue, &prepared,
                     portMAX_DELAY);
  }
}

/** @brief Establish SIM800 communication, retrying AT recovery after late
 * power-up. */
static void wait_for_gsm_ready(void) {
  const gsm_config_t configuration = {
      .uart_port = APP_GSM_UART_PORT,
      .tx_pin = APP_GSM_TX_GPIO,
      .rx_pin = APP_GSM_RX_GPIO,
      .baud_rate = APP_GSM_BAUD_RATE,
      .buf_size = APP_GSM_UART_BUFFER_SIZE,
      .timeout_ms = APP_GSM_COMMAND_TIMEOUT_MS,
      .retry_count = APP_GSM_DRIVER_RETRY_COUNT,
  };

  esp_err_t result = gsm_init(&configuration);
  while (result != ESP_OK) {
    ESP_LOGW(TAG, "SIM800 not ready; preserving queued events and retrying AT");
    vTaskDelay(pdMS_TO_TICKS(APP_GSM_RECOVERY_INTERVAL_MS));
    result = gsm_reset();
  }
  s_notification.gsm_ready = true;
}

/** @brief Send one message with application-level retries around the proven
 * driver. */
static esp_err_t send_with_retries(const char *message) {
  esp_err_t result = ESP_FAIL;
  for (uint32_t attempt = 0U; attempt <= APP_GSM_SEND_RETRY_COUNT; ++attempt) {
    result = gsm_send_sms(APP_NOTIFY_PHONE_NUMBER, message,
                          APP_GSM_COMMAND_TIMEOUT_MS);
    if (result == ESP_OK) {
      return ESP_OK;
    }

    ESP_LOGW(TAG, "SMS attempt %lu failed: %s", (unsigned long)(attempt + 1U),
             esp_err_to_name(result));
    if (attempt < APP_GSM_SEND_RETRY_COUNT) {
      vTaskDelay(pdMS_TO_TICKS(APP_GSM_SEND_RETRY_DELAY_MS));
    }
  }
  return result;
}

/** @brief Serialize all SIM800 transmissions in the driver's proven timing
 * path. */
static void gsm_sender_task(void *argument) {
  (void)argument;
  wait_for_gsm_ready();

  if (send_with_retries(APP_STARTUP_SMS_TEXT) != ESP_OK) {
    ESP_LOGE(TAG, "Startup SMS failed after configured retries");
  }

  prepared_sms_t prepared;
  while (true) {
    if (xQueueReceive(s_notification.prepared_sms_queue, &prepared,
                      portMAX_DELAY) != pdTRUE) {
      continue;
    }

    const esp_err_t result = send_with_retries(prepared.text);
    if (result == ESP_OK) {
      ESP_LOGI(TAG, "Event %lu SMS delivered",
               (unsigned long)prepared.event.sequence);
    } else {
      ++prepared.deferred_retry_count;
      const bool retry_allowed =
          (APP_GSM_DEFERRED_RETRY_LIMIT == 0U) ||
          (prepared.deferred_retry_count <= APP_GSM_DEFERRED_RETRY_LIMIT);
      if (retry_allowed) {
        ESP_LOGW(TAG, "Event %lu deferred for delivery retry cycle %lu",
                 (unsigned long)prepared.event.sequence,
                 (unsigned long)prepared.deferred_retry_count);
        /* The receive operation freed one queue slot, so requeueing the
         * same bounded message cannot grow memory use without limit. */
        (void)xQueueSend(s_notification.prepared_sms_queue, &prepared,
                         portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(APP_GSM_DEFERRED_RETRY_DELAY_MS));
      } else {
        ESP_LOGE(TAG,
                 "Event %lu SMS reached the configured deferred retry limit",
                 (unsigned long)prepared.event.sequence);
      }
    }
  }
}

esp_err_t notification_service_init(void) {
  if (s_notification.initialized) {
    return ESP_OK;
  }

  s_notification.relay_event_queue = control_service_get_event_queue();
  if (s_notification.relay_event_queue == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  s_notification.prepared_sms_queue =
      xQueueCreate(APP_PREPARED_SMS_QUEUE_DEPTH, sizeof(prepared_sms_t));
  if (s_notification.prepared_sms_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }

  s_notification.initialized = true;
  ESP_LOGI(TAG, "Notification service initialized");
  return ESP_OK;
}

esp_err_t notification_service_start(void) {
  if (!s_notification.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (s_notification.preparation_task == NULL) {
    const BaseType_t prepared = xTaskCreate(
        notification_preparation_task, "notification_prepare",
        APP_NOTIFICATION_TASK_STACK_SIZE, NULL, APP_NOTIFICATION_TASK_PRIORITY,
        &s_notification.preparation_task);
    if (prepared != pdPASS) {
      return ESP_ERR_NO_MEM;
    }
  }

  if (s_notification.sender_task == NULL) {
    const BaseType_t sender = xTaskCreate(
        gsm_sender_task, "gsm_sender", APP_GSM_SENDER_TASK_STACK_SIZE, NULL,
        APP_GSM_SENDER_TASK_PRIORITY, &s_notification.sender_task);
    if (sender != pdPASS) {
      return ESP_ERR_NO_MEM;
    }
  }
  return ESP_OK;
}

bool notification_service_is_ready(void) { return s_notification.gsm_ready; }
