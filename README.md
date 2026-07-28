# Efficient and Clean Energy Management

Author: **Matthithyahu**

This ESP-IDF application controls a bulb and fan from independent manual
buttons and an occupancy sensor, measures both loads with INA219 devices,
displays live state on an LCD2004, accumulates energy, and sends SIM800 SMS
notifications for every physical relay transition.

The project is a clean rewrite located at:

```text
C:\engRel\ai_assisted\efficient_and_clean_energy_management
```

The original project under `solar_pv_energy_monitoring_and_saving` is not used
as a build dependency and is not modified by this project.

## Reliability priorities

1. Button and relay control is isolated from GSM, LCD, INA219, and NVS work.
2. Every debounced press visibly toggles the current relay state.
3. Manual ON owns the relay for at least 60 seconds.
4. Electrical values are captured after relay/load settling and before modem
   delivery, so an earlier slow SMS cannot alter the event snapshot.
5. SIM800 transmissions remain serialized through the proven blocking
   AT-command sequence, with application-level retries.
6. All application I2C access is serialized by one monitoring task.
7. Energy uses actual elapsed time rather than assuming every loop took 500 ms.

## Default hardware configuration

The defaults preserve the pin map in the actual source project, which differs
from the older project description.

| Function | Default |
|---|---:|
| PIR OUT | GPIO23 |
| Bulb button | GPIO32 to GND |
| Fan button | GPIO33 to GND |
| Bulb relay | GPIO18 |
| Fan relay | GPIO19 |
| Relay ON level | HIGH |
| I2C SDA / SCL | GPIO21 / GPIO22 |
| LCD2004 address | `0x27` |
| Bulb INA219 | `0x44` |
| Fan INA219 | `0x40` |
| SIM800 ESP32 TX / RX | GPIO17 / GPIO16 |

All installation values are documented in
`components/app_config/include/app_config.h`. Change that file instead of
placing unexplained numbers in service code.

## Manual and automatic behavior

- A valid button press while a load is OFF selects `MANUAL_ON` and turns it ON
  immediately after the 30 ms debounce.
- A valid press while the same load is ON selects `MANUAL_OFF_LOCKOUT` and turns
  it OFF immediately. PIR input is ignored for the configured lockout interval.
- `MANUAL_ON` lasts 60 seconds by default. At expiry, an occupied room remains
  ON under automatic control; an unoccupied room turns OFF with reason
  `manual_timeout`.
- Automatic motion switches the load ON immediately after the short GPIO
  debounce. The load stays ON for the configurable six-second absence hold
  after the last qualified motion.

The bulb and fan use independent state machines even though they currently
share one PIR input.

## Fresh event measurements

Relay action is never delayed for measurement. For an ON event, notification
preparation waits 750 ms, then averages three INA219 samples that were recorded
while that relay was ON. OFF events use a 300 ms settle time. These values are
configuration constants.

Snapshot preparation and modem transmission use different tasks and queues:

```text
button/PIR -> control task -> relay event queue -> snapshot task
                                                -> prepared SMS queue
                                                -> SIM800 sender task
```

This prevents the value in an SMS from being replaced by either an old
pre-switch conversion or a much later reading taken after modem delivery began.

The INA219 driver is configured for the real 32 V bus range, ±320 mV PGA range,
and four-sample ADC averaging. With the configured 0.1 ohm shunt, the wider PGA
range supports the intended current range without the previous ±40 mV
saturation.

## SIM800 behavior

The existing stable UART rate, AT commands, one-second/two-second pacing, and
long confirmation timeouts remain in the GSM driver. Only scheduling around the
driver changed. All SMS transmissions are performed by one sender task and are
retried twice after an initial failure. If all immediate attempts fail, the
bounded prepared message is returned to the queue for a deferred retry instead
of being discarded. The retry interval and optional retry limit are named
configuration values.

Incoming inbox polling is disabled by default because remote commands are not
implemented and polling must not delay outgoing alerts. The setting is
`APP_GSM_INBOX_POLL_ENABLED` for future authenticated command work.

The SIM800 must use a suitable external supply capable of its current bursts,
with a common ground to the ESP32. Poor modem supply wiring cannot be corrected
in software.

## PIR/HW-740 validation

The installed sensor has now been verified as idle-LOW and motion-HIGH, so
`APP_PIR_ACTIVE_LEVEL` is configured as `1`.

If the sensor is replaced with a different PIR board, test it by itself before
changing the software polarity:

1. Confirm the board's `+`, `OUT`, and `-` pins; OUT is commonly the centre pin.
2. Use a common ground and a supply permitted by that exact board.
3. Disconnect OUT from GPIO23 and measure OUT-to-GND while idle and on motion.
4. A signal remaining around 1.64 V is not a valid ESP32 HIGH level. Do not try
   progressively stronger pull-ups; check pin order, supply, wiring, and the
   possibility of a damaged or different module.
5. If standalone OUT is clean 0/3.3 V active-high, set
   `APP_PIR_ACTIVE_LEVEL` to `1` and normally remove the external pull-up.
6. If the exact module is proven open-drain active-low, use an appropriate
   pull-up to 3.3 V and retain active level `0`.

## Component structure

```text
main/
  main.c                         startup and service composition only
components/
  app_config/                    named configuration and shared data types
  control_service/               buttons, PIR, modes, relays, events
  electrical_param_sensor/       corrected INA219 hardware driver
  energy_service/                sampling, history, integration, NVS totals
  display_service/               changed-line LCD2004 rendering
  monitoring_service/            single owner of the application I2C bus
  notification_service/          event snapshots and reliable SMS scheduling
  gsm_sim800/                     proven SIM800 transport
  lcd_i2c/                       proven LCD transport
```

## Build and flash

Use the ESP-IDF 5.4.2 terminal configured on the development computer:

```powershell
cd C:\engRel\ai_assisted\efficient_and_clean_energy_management
idf.py build
idf.py -p COM_PORT flash monitor
```

Replace `COM_PORT` with the ESP32 serial port. The supplied `sdkconfig` targets
the original ESP32 at 160 MHz with a 100 Hz FreeRTOS tick.

## Hardware verification checklist

1. With GSM disconnected, press and release each button repeatedly, including
   while PIR has already switched the load ON. Every press must visibly toggle.
2. Hold manual ON for a full minute and confirm PIR cannot turn it OFF early.
3. Confirm automatic control resumes after the minute.
4. Compare LCD and SMS values for ON and OFF events. ON SMS values should be the
   averaged post-start values, not the previous OFF reading.
5. Generate bulb and fan events close together and confirm both SMS messages
   are delivered in order.
6. Disconnect one INA219 and verify control and GSM continue while the display
   reports the sensor offline.
7. Test PIR voltage/polarity independently before judging software detection.

## Reference data sheets

- [Texas Instruments INA219 data sheet](https://www.ti.com/lit/ds/symlink/ina219.pdf)
- [Espressif ESP32 series data sheet](https://www.espressif.com/documentation/esp32_datasheet_en.pdf)
