# Changelog

Versions follow the policy in [`inc/j30d_version.h`](inc/j30d_version.h): this
versions the **source-level API contract**, not an ABI. While the major version
is 0, a MINOR bump may carry a source-breaking change; those are called out
explicitly below.

Dates are release dates. Releases before 0.5.0 were not tagged.

## 0.7.0 — 2026-11-16

### Added

- **`J30D_ERR_RESTARTED`, and a check for stopped conversions.** An I²C transfer that holds SCL low for about 10 ms sends the device its return-to-IIC signal. On a device that is converting, that clears `COMMAND`, so conversions stop and the data stops changing. In sleep mode every call rewrites `COMMAND` anyway. Converting continuously, nothing did, and the device stayed stopped. Now, when `j30d_get_data()` finds no new data for two conversion times (on every such call without `now_ms`), it reads `COMMAND` back. If conversions had stopped, it writes `COMMAND` again and returns `J30D_ERR_RESTARTED`. `j30d_get_data_blocking()` passes that code on.
- **`j30d_heat_flux()` and `j30d_heat_flux_x1000()`** — apply the unit's factory calibration to a `raw_ch1` sample: mW/cm² as a `float`, or mW/cm² × 1000 as an `int32_t` (12345 = 12.345 mW/cm²), the latter also in `J30D_NO_FLOAT` builds. Pure arithmetic with no bus access; pair them with `j30d_get_data_blocking()`. The formula follows the unit generation: `gain1 · raw_ch1[mV] + offset1` for REV1/REV2+, `gain1 · 1e‑5 · raw_ch1[code] − offset1` for CH1_NOTIMESTAMP.
- **`J30D_ERR_NO_CALIB`** — returned by the heat-flux functions for a unit with no usable calibration (CH2, UNKNOWN, or a block that could not be read).
- **`J30D_SLEEP_TIME_MS(ms)`** — the `SLEEP_TIME` register value for an interval in milliseconds, so `J30D_SLEEP_TIME_MS(1500)` instead of `0x0E`. A value that is not a multiple of 100 ms between 100 ms and 25.6 s fails to compile.
- **`J30D_SET(v)` and `J30D_SET_INIT(v)`** — set a config field's value and `J30D_FIELD_UPDATED` together, for an assignment and for an initializer. A `.value` written without its status is silently ignored; these make that mistake impossible.
- **`j30d_get_sample_interval_ms()`** — the polling period at which every `j30d_get_data()` call returns new data: 1796 ms at the defaults, where `SLEEP_TIME` is 1500 ms.
- **`j30d_get_data_blocking(h, out, timeout_ms)`** — waits for the next sample; `J30D_OK` means `raw_ch1` is `J30D_FRESH`. It goes through the same pacing as `j30d_get_data()`, so every sample comes from the conversion that ends a full sleep interval. Returns as soon as the INT is recorded in interrupt mode, after the worst-case sample time in polling. Requires `platform.now_ms`.

### Changed

- The data fields now state their units: `amb` in °C, `ch1`/`ch2` and `raw_ch1`/`raw_ch2` in mV. The STM32 port headers carry complete heat-flux examples.
- **The config field values are public.** `J30D_COMMAND_*`, `J30D_SYS_CFG1_*`, `J30D_SYS_CFG2_*`, `J30D_SYS_CHx_CFG_*` and `J30D_BIAS_CFG_*` moved from `j30d_regs.h`, which `j30d.h` does not include, to `j30d_config.h`, which it does — so `j30d_update_config()` at runtime no longer needs an internal header or magic numbers. `j30d_regs.h` includes `j30d_config.h`, so code that included it still compiles.
- **In interrupt mode, `j30d_get_data()` reads early only with an INT pending.** The shorter interrupt-mode wait used to apply to every call, so a call made without an INT — from a timer, or after a lost interrupt — could wake the device before the sample was complete and return the conversion done on entry to sleep. Without a pending INT the full polling wait now applies, and `j30d_notify_interrupt()` is what enables the shorter one.
- **`j30d_update_config()` ignores `interrupt_enable`.** Interrupt mode is chosen at `j30d_init()`. A zero-initialised config passed to `j30d_update_config()` used to switch interrupt mode off silently; it now changes only the fields it marks.

### Fixed

- **The FreeRTOS port could stall SCL.** Its polled HAL transfers hold SCL low while the task is not running, so a task preempted mid-transfer for about 10 ms stopped a converting device. Each transfer now runs with the scheduler suspended; interrupts stay enabled. `j30d_platform.h` states the requirement for every port: `i2c_read`/`i2c_write` must never pause with SCL held low for milliseconds.
- **`ch1`/`ch2` ignored `VT_SCALE`.** Their `.value` was always the CAL code divided by 2^19, which is right for `SYS_CFG2`'s `VT_SCALE` clear (±16 mV full scale) but reads 1/8 of the true voltage with it set (±128 mV). The scale now follows the bit; `J30D_VTP_SCALE_WIDE` / `J30D_VTP_SCALE_WIDE_SHIFT` (2^16) give the wide-range relationship for `J30D_NO_FLOAT` builds. The README and header examples now read `raw_ch1`, the input the factory heat-flux calibration applies to, instead of `ch1`.
- **Polling a sleeping device could return no new data at all.** Every read ends in a sleep command, which restarts the device's cycle — convert, sleep `SLEEP_TIME`, convert — and a wake before that second conversion completes halts it. `j30d_get_data()` paced from its own wake and counted only `SLEEP_TIME`, so polling slightly slower than `SLEEP_TIME` with `now_ms` (or at any rate below the true sample period without it) could restart the cycle on every call, and a timer at exactly `SLEEP_TIME` got every second call paced out. Pacing now counts from the last sleep command, whichever call issued it, and waits for the sleep interval and the conversions around it at their longest (oscillator ±15%). Conversion time is `(OSRP + 2 × OSRT + 1481) / Fosc_kHz` ms. The first `j30d_get_data()` after `j30d_init()`, `j30d_update_config()` or a raw call now returns stale data until that sample is complete, rather than waking early.
- **Re-initialising a handle is serialized.** `j30d_init()` takes the platform lock before clearing the handle, and a lock that cannot be acquired leaves the handle as it was.
- **`j30d_init()` checks the config it actually applies.** With `config == NULL`, the rules for `command`/`sleep` and interrupt mode now apply to `j30d_default_config`, and its `UNCHANGED` fields are read back from the device. A `DEFAULT` `command` is checked by its default value.
- **`chip_address` in `h->config` reflects the device** when left `UNCHANGED`, and after a `j30d_raw_write()` to `0x92`.
- **`j30d_update_config()` writes COMMAND once.** It no longer re-commands a device it has just put to sleep.
- **FreeRTOS port:** `configTICK_RATE_HZ` must divide 1000 (compile-time check); minimum delays are rounded up to whole ticks; a mutex that cannot be allocated reports `J30D_ERR_TIMEOUT` instead of faulting.
- **Zephyr sketch:** the return-to-IIC sequence holds SCL low through `scl-gpios`; `i2c_recover_bus()` does not perform it.

## 0.6.0 — 2026-10-02

- **Interrupt mode** (`config.interrupt_enable`): the sleep-mode INT line on SDA as a data-ready signal. Requires `platform.release_to_int`, `platform.now_ms` and `SLEEP_EN`, and makes the I²C bus exclusive to this device while it sleeps.
- **Fixed: a burst read could straddle a conversion.** The conversion engine is now stopped on every wake, so the data registers hold still for the read.

## 0.5.0 — 2026-09-21

### Added

- **`chip_address` in `j30d_config_t`** — the device's own 7-bit I²C address
  (register `0x92`), applied at every `j30d_init()` like any other field.

  Writing `0x92` moves the device **immediately**: no EEPROM programming and
  no reset. That is measured, not assumed — `J30D_LiveTest_AddressProbe()`
  watches the new address start answering and the old one fall silent. It
  behaves differently from `SYS_CFG1`'s `output_mode`, which *is* latched at
  EEPROM load and does nothing until programmed.

  Because nothing is stored, nothing can be corrupted: the driver still has no
  code path that can program EEPROM, which is deliberate. An interrupted EEPROM
  write destroys the factory calibration, and the only airtight protection is
  not having the capability at all.

  Defaults to `J30D_FIELD_UNCHANGED`, so a plain `j30d_init()` never touches
  the address. To set one, change **both** fields in `config/j30d_config.c`:

  ```c
  .chip_address = {
      .value  = 0x10,                  /* your 7-bit address */
      .status = J30D_FIELD_UPDATED,    /* without this, .value is ignored */
  },
  ```

  **Your port must target the 0x7F wildcard.** The part always answers 0x7F
  whatever its `Chip_Address`, so the driver keeps working across the move; a
  port aimed at a fixed address would lose the device the moment this is
  written. The reference ports already do this.

  Known limitation: several *unprogrammed* units on one bus all answer 0x7F, so
  this cannot single one out. Multi-sensor boards need distinct addresses set at
  calibration time.

- `J30D_LiveTest_AddressProbe()` — the on-target experiment behind the above.
  Safe on any unit: it never writes `BLOW_START` and never touches the
  calibration block, and `0x92` reloads from EEPROM at the next reset, so a
  power cycle undoes everything it does.

### Fixed

- **An erased chip-revision byte is no longer read as "revision 255".**
  `0xFF` in the revision field means the byte was never written, not a version
  number. The old check (`revision >= 2`) sent such units into the rev2+ path,
  which requires a valid CRC over the production block — but these units have a
  blank checksum too, so it could never match. They came back
  `J30D_UNIT_VERSION_UNKNOWN` and their factory trim was silently discarded,
  leaving the sensor running on defaults.

  **Affects anyone with units programmed before the revision field existed.**
  If you have seen `UNKNOWN` on a unit you know was calibrated, this is why.
  Those units now classify as `J30D_UNIT_VERSION_REV1` or
  `J30D_UNIT_VERSION_CH1_NOTIMESTAMP` and keep their calibration.

### Source-breaking

- `chip_address` was added at the **front** of `j30d_config_t`. Any positional
  (non-designated) initialiser for that struct now assigns the wrong fields and
  will not be caught by the compiler. Use designated initialisers, as
  `config/j30d_config.c` does:

  ```c
  j30d_config_t cfg = { .command = { ... }, .sleep = { ... } };
  ```

  No action needed if you copy `j30d_default_config` and modify fields, which is
  the documented approach.

### Notes

- `j30d_handle_t` embeds driver-private state by value, so its size and layout
  can change in any release. Compile the driver's `.c` files and your
  application against the same revision of these headers; never link a
  pre-built object against different ones.
