/**
 * @file display_service.c
 * @brief Readable, low-flicker LCD2004 screen implementation.
 * @author Matthithyahu
 */

#include "display_service.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_types.h"
#include "control_service.h"
#include "energy_service.h"
#include "esp_log.h"
#include "lcd_i2c.h"

static const char *TAG = "display";

/** @brief Display state kept between refreshes. */
typedef struct {
  lcd_handle_t *lcd;
  char displayed_lines[APP_LCD_ROWS][APP_LCD_COLUMNS + 1U];
  uint64_t last_screen_switch_ms;
  bool showing_status;
  bool initialized;
} display_context_t;

static display_context_t s_display = {
    .showing_status = true,
};

/** @brief Format and pad exactly one LCD row. */
static void make_line(char output[APP_LCD_COLUMNS + 1U], const char *format,
                      ...) {
  char temporary[64] = {0};
  va_list arguments;
  va_start(arguments, format);
  (void)vsnprintf(temporary, sizeof(temporary), format, arguments);
  va_end(arguments);

  memset(output, ' ', APP_LCD_COLUMNS);
  const size_t length = strnlen(temporary, APP_LCD_COLUMNS);
  memcpy(output, temporary, length);
  output[APP_LCD_COLUMNS] = '\0';
}

/** @brief Convert remaining milliseconds to a rounded-up display value. */
static unsigned long remaining_seconds(uint32_t remaining_ms) {
  return (unsigned long)((remaining_ms + 999U) / 1000U);
}

/** @brief Build the occupancy/manual-control screen. */
static void make_status_screen(const app_control_snapshot_t *control,
                               char lines[APP_LCD_ROWS][APP_LCD_COLUMNS + 1U]) {
  for (size_t index = 0U; index < APP_LOAD_COUNT; ++index) {
    const app_load_status_t *load = &control->loads[index];
    make_line(lines[index], "%s:%-3s %-5s %3lus",
              index == APP_LOAD_BULB ? "BULB" : "FAN ",
              load->relay_on ? "ON" : "OFF",
              control_service_mode_text(load->mode),
              remaining_seconds(load->mode_remaining_ms));
  }

  if (!control->pir_ready) {
    make_line(lines[2], "PIR: STARTUP WARMUP");
  } else {
    make_line(lines[2], "PIR: %s",
              control->pir_motion ? "MOTION" : "NO MOTION");
  }
  make_line(lines[3], "Buttons toggle loads");
}

/** @brief Build the voltage/current/power/energy screen. */
static void make_electrical_screen(
    char lines[APP_LCD_ROWS][APP_LCD_COLUMNS + 1U]) {
  app_energy_snapshot_t bulb = {0};
  app_energy_snapshot_t fan = {0};
  (void)energy_service_get_snapshot(APP_LOAD_BULB, &bulb);
  (void)energy_service_get_snapshot(APP_LOAD_FAN, &fan);
  const app_energy_snapshot_t values[APP_LOAD_COUNT] = {bulb, fan};

  for (size_t index = 0U; index < APP_LOAD_COUNT; ++index) {
    const size_t first_row = index * 2U;
    if (!values[index].sensor_online) {
      make_line(lines[first_row], "%s sensor offline",
                index == APP_LOAD_BULB ? "BULB" : "FAN");
      make_line(lines[first_row + 1U], "E:%10.6fkWh", values[index].energy_kwh);
      continue;
    }

    make_line(lines[first_row], "%s %4.1fV %5.3fA",
              index == APP_LOAD_BULB ? "BULB" : "FAN ", values[index].voltage_v,
              values[index].current_a);
    make_line(lines[first_row + 1U], "P:%5.1fW E:%8.6f", values[index].power_w,
              values[index].energy_kwh);
  }
}

/** @brief Write changed rows and stop immediately on the first bus error. */
static esp_err_t write_changed_lines(
    char lines[APP_LCD_ROWS][APP_LCD_COLUMNS + 1U]) {
  for (size_t row = 0U; row < APP_LCD_ROWS; ++row) {
    if (memcmp(lines[row], s_display.displayed_lines[row], APP_LCD_COLUMNS) ==
        0) {
      continue;
    }

    if (lcd_set_cursor(s_display.lcd, (uint8_t)row, 0U) != LCD_OK ||
        lcd_print_str(s_display.lcd, lines[row]) != LCD_OK) {
      ESP_LOGW(TAG, "LCD update failed on row %u", (unsigned)row);
      return ESP_FAIL;
    }
    memcpy(s_display.displayed_lines[row], lines[row], APP_LCD_COLUMNS + 1U);
  }
  return ESP_OK;
}

esp_err_t display_service_init(void) {
  if (s_display.initialized) {
    return ESP_OK;
  }

  const lcd_config_t configuration = {
      .i2c_port = APP_I2C_PORT,
      .i2c_addr = APP_LCD_I2C_ADDRESS,
      .rows = APP_LCD_ROWS,
      .cols = APP_LCD_COLUMNS,
      .backlight_enable = true,
      .i2c_timeout_ms = APP_LCD_TRANSACTION_TIMEOUT_MS,
      .cmd_delay_us = APP_LCD_COMMAND_DELAY_US,
  };
  s_display.lcd = lcd_i2c_init(&configuration);
  if (s_display.lcd == NULL) {
    ESP_LOGW(TAG,
             "LCD2004 not available; monitoring continues without display");
    return ESP_ERR_NOT_FOUND;
  }

  if (lcd_clear(s_display.lcd) != LCD_OK) {
    ESP_LOGW(TAG, "LCD clear failed during initialization");
  }
  memset(s_display.displayed_lines, 0, sizeof(s_display.displayed_lines));
  s_display.initialized = true;
  ESP_LOGI(TAG, "LCD display service initialized");
  return ESP_OK;
}

esp_err_t display_service_render(uint64_t now_ms) {
  if (!s_display.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if ((now_ms - s_display.last_screen_switch_ms) >=
      APP_LCD_SCREEN_SWITCH_INTERVAL_MS) {
    s_display.showing_status = !s_display.showing_status;
    s_display.last_screen_switch_ms = now_ms;
    memset(s_display.displayed_lines, 0, sizeof(s_display.displayed_lines));
  }

  char lines[APP_LCD_ROWS][APP_LCD_COLUMNS + 1U] = {{0}};
  if (s_display.showing_status) {
    app_control_snapshot_t control = {0};
    (void)control_service_get_snapshot(&control);
    make_status_screen(&control, lines);
  } else {
    make_electrical_screen(lines);
  }
  return write_changed_lines(lines);
}
