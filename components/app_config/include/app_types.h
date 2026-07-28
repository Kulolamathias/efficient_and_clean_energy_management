/**
 * @file app_types.h
 * @brief Shared domain types for the energy-management application.
 * @author Matthithyahu
 *
 * This file contains data contracts shared by the control, monitoring,
 * display, and notification services. It deliberately contains no hardware
 * access or business logic.
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Independently controlled loads in the installation. */
typedef enum { APP_LOAD_BULB = 0, APP_LOAD_FAN, APP_LOAD_COUNT } app_load_id_t;

/** @brief Current owner of a load output. */
typedef enum {
  APP_CONTROL_MODE_AUTO = 0,
  APP_CONTROL_MODE_MANUAL_ON,
  APP_CONTROL_MODE_MANUAL_OFF_LOCKOUT
} app_control_mode_t;

/** @brief Reason associated with a physical relay transition. */
typedef enum {
  APP_RELAY_REASON_MANUAL = 0,
  APP_RELAY_REASON_MOTION_DETECTED,
  APP_RELAY_REASON_ABSENCE_TIMEOUT,
  APP_RELAY_REASON_MANUAL_TIMEOUT
} app_relay_reason_t;

/** @brief Relay transition queued for event-time measurement and SMS. */
typedef struct {
  app_load_id_t load_id;     /**< Load whose relay changed. */
  bool relay_on;             /**< New physical relay state. */
  app_control_mode_t mode;   /**< Control mode after the transition. */
  app_relay_reason_t reason; /**< Cause of the transition. */
  uint64_t event_time_ms;    /**< Monotonic event timestamp. */
  uint32_t sequence;         /**< Increasing event identifier. */
} app_relay_event_t;

/** @brief Read-only state of one load for displays and diagnostics. */
typedef struct {
  bool relay_on;              /**< Logical relay state. */
  app_control_mode_t mode;    /**< Current control mode. */
  uint32_t mode_remaining_ms; /**< Time left in a temporary mode. */
} app_load_status_t;

/** @brief Atomic snapshot of the occupancy-control subsystem. */
typedef struct {
  bool pir_motion; /**< Debounced PIR motion state. */
  bool pir_ready;  /**< PIR startup-ignore time has elapsed. */
  app_load_status_t loads[APP_LOAD_COUNT]; /**< Bulb and fan states. */
  uint64_t sample_time_ms; /**< Time at which this state was copied. */
} app_control_snapshot_t;

/** @brief Electrical reading and accumulated energy for one load. */
typedef struct {
  bool sensor_online;      /**< True when the latest read succeeded. */
  bool relay_on_at_sample; /**< Relay state paired with this sample. */
  float voltage_v;         /**< Bus voltage in volts. */
  float current_a;         /**< Load current in amperes. */
  float power_w;           /**< Load power in watts. */
  double energy_kwh;       /**< Accumulated energy in kilowatt-hours. */
  uint64_t sample_time_ms; /**< Monotonic measurement timestamp. */
  uint32_t sequence;       /**< Increasing measurement identifier. */
} app_energy_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif /* APP_TYPES_H */
