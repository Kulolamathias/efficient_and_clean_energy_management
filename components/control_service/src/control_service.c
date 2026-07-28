/**
 * @file control_service.c
 * @brief Deterministic implementation of manual and occupancy control.
 * @author Matthithyahu
 *
 * This task performs GPIO work only. It never accesses I2C, the LCD, INA219,
 * NVS, or the SIM800, so those slower devices cannot hide a button press.
 */

#include "control_service.h"

#include <inttypes.h>
#include <string.h>

#include "app_config.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

static const char *TAG = "control";

/** @brief Automatic occupancy states for each independent load. */
typedef enum {
  AUTO_STATE_IDLE = 0,
  AUTO_STATE_ON_HOLD
} auto_state_t;

/** @brief Debounce state for an active-low momentary button. */
typedef struct {
  gpio_num_t gpio;
  int raw_level;
  int stable_level;
  uint64_t raw_changed_ms;
} button_state_t;

/** @brief Complete internal state of one controlled load. */
typedef struct {
  app_load_id_t load_id;
  gpio_num_t relay_gpio;
  button_state_t button;
  app_control_mode_t mode;
  uint64_t mode_until_ms;
  auto_state_t auto_state;
  uint64_t last_motion_ms;
  bool auto_demand;
  bool relay_on;
} load_controller_t;

/** @brief Singleton context owned by the control task. */
typedef struct {
  load_controller_t loads[APP_LOAD_COUNT];
  QueueHandle_t event_queue;
  TaskHandle_t task;
  portMUX_TYPE snapshot_lock;
  app_control_snapshot_t snapshot;
  uint64_t start_time_ms;
  bool pir_raw_motion;
  bool pir_stable_motion;
  uint64_t pir_raw_changed_ms;
  uint32_t event_sequence;
  bool initialized;
} control_context_t;

static control_context_t s_control = {
    .snapshot_lock = portMUX_INITIALIZER_UNLOCKED,
};

/** @brief Return monotonic milliseconds since ESP32 startup. */
static uint64_t monotonic_ms(void) {
  return (uint64_t)(esp_timer_get_time() / 1000LL);
}

/** @brief Return elapsed time without relying on a wrapping RTOS tick. */
static uint64_t elapsed_ms(uint64_t now_ms, uint64_t since_ms) {
  return now_ms - since_ms;
}

const char *control_service_load_name(app_load_id_t load_id) {
  switch (load_id) {
    case APP_LOAD_BULB:
      return "Bulb";
    case APP_LOAD_FAN:
      return "Fan";
    default:
      return "Unknown";
  }
}

const char *control_service_reason_text(app_relay_reason_t reason) {
  switch (reason) {
    case APP_RELAY_REASON_MANUAL:
      return "manual";
    case APP_RELAY_REASON_MOTION_DETECTED:
      return "motion_detected";
    case APP_RELAY_REASON_ABSENCE_TIMEOUT:
      return "absence_timeout";
    case APP_RELAY_REASON_MANUAL_TIMEOUT:
      return "manual_timeout";
    default:
      return "unknown";
  }
}

const char *control_service_mode_text(app_control_mode_t mode) {
  switch (mode) {
    case APP_CONTROL_MODE_AUTO:
      return "AUTO";
    case APP_CONTROL_MODE_MANUAL_ON:
      return "MANON";
    case APP_CONTROL_MODE_MANUAL_OFF_LOCKOUT:
      return "LOCK";
    default:
      return "?";
  }
}

/** @brief Configure all application GPIOs and establish safe output levels. */
static esp_err_t configure_gpio(void) {
  const gpio_config_t relay_config = {
      .pin_bit_mask =
          (1ULL << APP_RELAY_BULB_GPIO) | (1ULL << APP_RELAY_FAN_GPIO),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&relay_config), TAG,
                      "Relay GPIO config failed");

  ESP_RETURN_ON_ERROR(gpio_set_level(APP_RELAY_BULB_GPIO, APP_RELAY_OFF_LEVEL),
                      TAG, "Bulb safe-state write failed");
  ESP_RETURN_ON_ERROR(gpio_set_level(APP_RELAY_FAN_GPIO, APP_RELAY_OFF_LEVEL),
                      TAG, "Fan safe-state write failed");

  const gpio_config_t button_config = {
      .pin_bit_mask =
          (1ULL << APP_BUTTON_BULB_GPIO) | (1ULL << APP_BUTTON_FAN_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&button_config), TAG,
                      "Button GPIO config failed");

  const gpio_config_t pir_config = {
      .pin_bit_mask = (1ULL << APP_PIR_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = APP_PIR_INTERNAL_PULLUP_ENABLED ? GPIO_PULLUP_ENABLE
                                                    : GPIO_PULLUP_DISABLE,
      .pull_down_en = APP_PIR_INTERNAL_PULLDOWN_ENABLED ? GPIO_PULLDOWN_ENABLE
                                                        : GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&pir_config), TAG, "PIR GPIO config failed");
  return ESP_OK;
}

/** @brief Reset automatic demand without modifying manual ownership. */
static void reset_automatic_state(load_controller_t *load, uint64_t now_ms) {
  load->auto_state = AUTO_STATE_IDLE;
  load->last_motion_ms = now_ms;
  load->auto_demand = false;
}

/** @brief Queue a physical relay transition without blocking the control task.
 */
static void queue_relay_event(const load_controller_t *load,
                              app_relay_reason_t reason, uint64_t now_ms) {
  app_relay_event_t event = {
      .load_id = load->load_id,
      .relay_on = load->relay_on,
      .mode = load->mode,
      .reason = reason,
      .event_time_ms = now_ms,
      .sequence = ++s_control.event_sequence,
  };

  if (xQueueSend(s_control.event_queue, &event, 0) != pdTRUE) {
    ESP_LOGE(TAG,
             "Relay event queue full; event %" PRIu32 " could not be queued",
             event.sequence);
  }
}

/** @brief Apply a relay state exactly once and emit an event only on change. */
static void set_relay(load_controller_t *load, bool requested_on,
                      app_relay_reason_t reason, uint64_t now_ms) {
  if (requested_on == load->relay_on) {
    return;
  }

  const int level = requested_on ? APP_RELAY_ON_LEVEL : APP_RELAY_OFF_LEVEL;
  if (gpio_set_level(load->relay_gpio, level) != ESP_OK) {
    ESP_LOGE(TAG, "%s relay GPIO write failed",
             control_service_load_name(load->load_id));
    return;
  }

  load->relay_on = requested_on;
  ESP_LOGI(TAG, "%s %s (%s, mode=%s)", control_service_load_name(load->load_id),
           requested_on ? "ON" : "OFF", control_service_reason_text(reason),
           control_service_mode_text(load->mode));
  queue_relay_event(load, reason, now_ms);
}

/** @brief Return true once for every fully debounced press edge. */
static bool update_button(button_state_t *button, uint64_t now_ms) {
  const int level = gpio_get_level(button->gpio);
  if (level != button->raw_level) {
    button->raw_level = level;
    button->raw_changed_ms = now_ms;
  }

  if ((button->stable_level != button->raw_level) &&
      (elapsed_ms(now_ms, button->raw_changed_ms) >= APP_BUTTON_DEBOUNCE_MS)) {
    button->stable_level = button->raw_level;
    return button->stable_level == APP_BUTTON_PRESSED_LEVEL;
  }
  return false;
}

/** @brief Toggle the visible relay state immediately on a valid press. */
static void handle_manual_press(load_controller_t *load, uint64_t now_ms) {
  reset_automatic_state(load, now_ms);

  if (load->relay_on) {
    load->mode = APP_CONTROL_MODE_MANUAL_OFF_LOCKOUT;
    load->mode_until_ms = now_ms + APP_MANUAL_OFF_LOCKOUT_MS;
    set_relay(load, false, APP_RELAY_REASON_MANUAL, now_ms);
  } else {
    load->mode = APP_CONTROL_MODE_MANUAL_ON;
    load->mode_until_ms = now_ms + APP_MANUAL_ON_HOLD_MS;
    set_relay(load, true, APP_RELAY_REASON_MANUAL, now_ms);
  }
}

/** @brief Return a temporary manual mode to automatic control. */
static void update_manual_timeout(load_controller_t *load, bool pir_motion,
                                  uint64_t now_ms) {
  if ((load->mode == APP_CONTROL_MODE_AUTO) || (now_ms < load->mode_until_ms)) {
    return;
  }

  const app_control_mode_t expired_mode = load->mode;
  load->mode = APP_CONTROL_MODE_AUTO;
  load->mode_until_ms = 0U;
  reset_automatic_state(load, now_ms);

  if (expired_mode == APP_CONTROL_MODE_MANUAL_ON) {
    if (pir_motion) {
      load->auto_state = AUTO_STATE_ON_HOLD;
      load->auto_demand = true;
      load->last_motion_ms = now_ms;
    } else {
      set_relay(load, false, APP_RELAY_REASON_MANUAL_TIMEOUT, now_ms);
    }
  }
}

/** @brief Advance one load's automatic occupancy state machine. */
static void update_automatic_control(load_controller_t *load, bool pir_motion,
                                     uint64_t now_ms) {
  bool requested_on = load->auto_demand;
  app_relay_reason_t reason = APP_RELAY_REASON_MOTION_DETECTED;

  switch (load->auto_state) {
    case AUTO_STATE_IDLE:
      requested_on = false;
      if (pir_motion) {
        requested_on = true;
        load->last_motion_ms = now_ms;
        load->auto_state = AUTO_STATE_ON_HOLD;
        reason = APP_RELAY_REASON_MOTION_DETECTED;
      }
      break;

    case AUTO_STATE_ON_HOLD:
      requested_on = true;
      if (pir_motion) {
        load->last_motion_ms = now_ms;
      } else if (elapsed_ms(now_ms, load->last_motion_ms) >=
                 APP_PIR_ABSENCE_HOLD_MS) {
        requested_on = false;
        load->auto_state = AUTO_STATE_IDLE;
        reason = APP_RELAY_REASON_ABSENCE_TIMEOUT;
      }
      break;

    default:
      reset_automatic_state(load, now_ms);
      requested_on = false;
      reason = APP_RELAY_REASON_ABSENCE_TIMEOUT;
      break;
  }

  load->auto_demand = requested_on;
  set_relay(load, requested_on, reason, now_ms);
}

/** @brief Debounce the configured PIR polarity after startup warm-up. */
static bool update_pir(uint64_t now_ms, bool *pir_ready) {
  *pir_ready =
      elapsed_ms(now_ms, s_control.start_time_ms) >= APP_PIR_STARTUP_IGNORE_MS;
  const bool raw_motion = gpio_get_level(APP_PIR_GPIO) == APP_PIR_ACTIVE_LEVEL;

  if (!*pir_ready) {
    s_control.pir_raw_motion = raw_motion;
    s_control.pir_stable_motion = false;
    s_control.pir_raw_changed_ms = now_ms;
    return false;
  }

  if (raw_motion != s_control.pir_raw_motion) {
    s_control.pir_raw_motion = raw_motion;
    s_control.pir_raw_changed_ms = now_ms;
  }

  if ((s_control.pir_stable_motion != s_control.pir_raw_motion) &&
      (elapsed_ms(now_ms, s_control.pir_raw_changed_ms) >=
       APP_PIR_DEBOUNCE_MS)) {
    s_control.pir_stable_motion = s_control.pir_raw_motion;
    ESP_LOGI(TAG, "PIR %s", s_control.pir_stable_motion ? "MOTION" : "CLEAR");
  }
  return s_control.pir_stable_motion;
}

/** @brief Publish a consistent state for readers on either CPU core. */
static void publish_snapshot(bool pir_motion, bool pir_ready, uint64_t now_ms) {
  app_control_snapshot_t next = {
      .pir_motion = pir_motion,
      .pir_ready = pir_ready,
      .sample_time_ms = now_ms,
  };

  for (size_t index = 0; index < APP_LOAD_COUNT; ++index) {
    const load_controller_t *load = &s_control.loads[index];
    next.loads[index].relay_on = load->relay_on;
    next.loads[index].mode = load->mode;
    next.loads[index].mode_remaining_ms =
        (load->mode != APP_CONTROL_MODE_AUTO && load->mode_until_ms > now_ms)
            ? (uint32_t)(load->mode_until_ms - now_ms)
            : 0U;
  }

  portENTER_CRITICAL(&s_control.snapshot_lock);
  s_control.snapshot = next;
  portEXIT_CRITICAL(&s_control.snapshot_lock);
}

/** @brief High-priority task that is never blocked by peripheral services. */
static void control_task(void *argument) {
  (void)argument;
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(APP_CONTROL_POLL_INTERVAL_MS);

  while (true) {
    const uint64_t now_ms = monotonic_ms();
    bool pir_ready = false;
    const bool pir_motion = update_pir(now_ms, &pir_ready);

    for (size_t index = 0; index < APP_LOAD_COUNT; ++index) {
      load_controller_t *load = &s_control.loads[index];
      if (update_button(&load->button, now_ms)) {
        handle_manual_press(load, now_ms);
      }

      update_manual_timeout(load, pir_motion, now_ms);
      if (load->mode == APP_CONTROL_MODE_AUTO) {
        update_automatic_control(load, pir_motion, now_ms);
      }
    }

    publish_snapshot(pir_motion, pir_ready, now_ms);
    vTaskDelayUntil(&last_wake, period);
  }
}

esp_err_t control_service_init(void) {
  if (s_control.initialized) {
    return ESP_OK;
  }

  ESP_RETURN_ON_ERROR(configure_gpio(), TAG, "GPIO initialization failed");

  s_control.event_queue =
      xQueueCreate(APP_RELAY_EVENT_QUEUE_DEPTH, sizeof(app_relay_event_t));
  if (s_control.event_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }

  const uint64_t now_ms = monotonic_ms();
  s_control.start_time_ms = now_ms;
  s_control.pir_raw_motion =
      gpio_get_level(APP_PIR_GPIO) == APP_PIR_ACTIVE_LEVEL;
  s_control.pir_stable_motion = false;
  s_control.pir_raw_changed_ms = now_ms;

  s_control.loads[APP_LOAD_BULB] = (load_controller_t){
      .load_id = APP_LOAD_BULB,
      .relay_gpio = APP_RELAY_BULB_GPIO,
      .button =
          {
              .gpio = APP_BUTTON_BULB_GPIO,
              .raw_level = gpio_get_level(APP_BUTTON_BULB_GPIO),
              .stable_level = gpio_get_level(APP_BUTTON_BULB_GPIO),
              .raw_changed_ms = now_ms,
          },
      .mode = APP_CONTROL_MODE_AUTO,
      .auto_state = AUTO_STATE_IDLE,
      .relay_on = false,
  };

  s_control.loads[APP_LOAD_FAN] = (load_controller_t){
      .load_id = APP_LOAD_FAN,
      .relay_gpio = APP_RELAY_FAN_GPIO,
      .button =
          {
              .gpio = APP_BUTTON_FAN_GPIO,
              .raw_level = gpio_get_level(APP_BUTTON_FAN_GPIO),
              .stable_level = gpio_get_level(APP_BUTTON_FAN_GPIO),
              .raw_changed_ms = now_ms,
          },
      .mode = APP_CONTROL_MODE_AUTO,
      .auto_state = AUTO_STATE_IDLE,
      .relay_on = false,
  };

  publish_snapshot(false, false, now_ms);
  s_control.initialized = true;
  ESP_LOGI(TAG, "Control service initialized; relays are safely OFF");
  return ESP_OK;
}

esp_err_t control_service_start(void) {
  if (!s_control.initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  if (s_control.task != NULL) {
    return ESP_OK;
  }

  const BaseType_t created =
      xTaskCreate(control_task, "control_service", APP_CONTROL_TASK_STACK_SIZE,
                  NULL, APP_CONTROL_TASK_PRIORITY, &s_control.task);
  return created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

QueueHandle_t control_service_get_event_queue(void) {
  return s_control.event_queue;
}

esp_err_t control_service_get_snapshot(app_control_snapshot_t *snapshot) {
  if (snapshot == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  portENTER_CRITICAL(&s_control.snapshot_lock);
  *snapshot = s_control.snapshot;
  portEXIT_CRITICAL(&s_control.snapshot_lock);
  return ESP_OK;
}
