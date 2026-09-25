# Changelog

Versions follow the policy in [`inc/j30d_version.h`](inc/j30d_version.h): this
versions the **source-level API contract**, not an ABI. While the major version
is 0, a MINOR bump may carry a source-breaking change; those are called out
explicitly below.

Dates are release dates. Releases before 0.5.0 were not tagged.

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
