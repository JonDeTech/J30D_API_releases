# J30D_API

Firmware driver for the Jondetech **J30D / JIRS30D** heat-flux sensor.

Portable C99. No dependencies, no heap, no globals. Everything platform specific —
I²C, delays, GPIO, RTOS locking — is supplied by the integrator as a handful of
function pointers, so the driver core has no knowledge of any vendor HAL, RTOS or
register beyond the sensor's own.

---

## Design in one screen

- **No global state.** Every call takes an explicit `j30d_handle_t *`. Multiple
  sensors are just multiple handles, and two threads working on *different*
  handles need no locking at all.
- **Platform behind function pointers.** See [`inc/j30d_platform.h`](inc/j30d_platform.h).
  Four hooks are required, three are optional.
- **Sleep is handled for you.** If sleep is configured, every call that needs the
  bus wakes the device, stops the conversion engine so a burst read cannot straddle
  a conversion, and re-commands sleep afterwards. There is no public "wake"
  function and you never have to ask whether the device is asleep.
- **Freshness is per field, not per call.** A read returning `J30D_OK` tells you the
  bus worked; each field separately tells you whether *it* had a new sample.
- **Values and raw codes both.** Every field carries engineering units and the
  unscaled ADC code it came from.

## Quick start

**1. Write a port.** Fill in a `j30d_platform_t`. The four required hooks:

```c
static j30d_err_t my_read (void *ctx, uint8_t reg, uint8_t *buf, size_t len);
static j30d_err_t my_write(void *ctx, uint8_t reg, const uint8_t *buf, size_t len);
static void       my_delay(void *ctx, uint32_t ms);
static void       my_return_to_iic(void *ctx);   /* drive SCL low >10ms, then hand
                                                    the pins back to the I2C block */
```

`return_to_iic` is not a generic stuck-bus workaround — it is the ASIC's own
documented path back to I²C idle, from sleep (where SDA is the INT output) or from
PWM/Relay mode. [`inc/j30d_platform.h`](inc/j30d_platform.h) explains it in full, and
[`ports/`](ports/) has working STM32 implementations.

**2. Initialise and read.**

```c
#include "j30d.h"

j30d_handle_t h;
j30d_platform_t platform = {
    .i2c_read = my_read,   .i2c_write = my_write,
    .delay_ms = my_delay,  .return_to_iic = my_return_to_iic,
    .ctx = &my_bus,
    /* optional: .lock/.unlock for multithreading, .now_ms to pace polling */
};

if (j30d_init(&h, &platform, NULL) != J30D_OK) { /* handle it */ }   /* NULL == compiled-in defaults */

j30d_data_t d;
if (j30d_get_data(&h, &d) == J30D_OK) {
    if (d.ch1.state == J30D_FRESH) {
        use(d.ch1.value);      /* engineering units */
        log(d.ch1.raw_code);   /* the unscaled ADC code it came from */
    }
    /* fields that were not ready are J30D_STALE and still hold their
       last known-good value -- they are never left undefined */
}
```

That is the whole hot path. `j30d_get_data()` refreshes all six fields in one call.

## Layout

| Path | What it is |
|---|---|
| `inc/` | Public headers. `j30d.h` is the entry point and includes the rest. |
| `src/` | The driver core. Portable, no platform knowledge. |
| `config/j30d_config.c` | **Yours to edit.** The compiled-in `j30d_default_config`. |
| `ports/` | Reference platform implementations — see the caveat below. |
| `test/fake_sensor_test.c` | Host test suite. Runs anywhere, no hardware. |
| `test/j30d_live_test.*` | On-target stress test. Needs real hardware and FreeRTOS. |
| `CHANGELOG.md` | What changed in each release, and what to do about it. |

**About `ports/`:** these `#include` *your* project's headers (`main.h`, `i2c.h`, the
vendor HAL, FreeRTOS), so they do not compile standalone in this repository. Treat
them as reference implementations to copy into your tree and adjust.
`ports/platform_zephyr_mockup.c` is explicitly an uncompiled sketch showing the shape
a Zephyr port would take — it is not a working port and the build will not accept it.

## Building

The driver is four `.c` files. **A build system is optional** — if you have your own,
just compile these with `inc/` on the include path and skip this section:

```
src/j30d.c
src/j30d_version.c
config/j30d_config.c
ports/<the one port you want>.c      # or your own
```

### With CMake

Standalone, including the test suite:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

Vendored into your project (tests and install rules turn themselves off):

```cmake
add_subdirectory(J30D_API)
target_link_libraries(my_app PRIVATE j30d::j30d)
```

Installed:

```bash
cmake -S . -B build && cmake --build build && cmake --install build --prefix /your/prefix
```

```cmake
find_package(j30d REQUIRED)
target_link_libraries(my_app PRIVATE j30d::j30d)
```

### Choosing a compiler (Windows)

CMake defaults to the Visual Studio generator on Windows, so a plain
`cmake -S . -B build` picks MSVC even when you want gcc. Use a preset:

```bat
cmake --preset gcc
cmake --build --preset gcc
ctest --preset gcc
```

`CMakePresets.json` provides `gcc` (Ninja), `gcc-make` (MinGW Makefiles, no ninja
needed), `gcc-nofloat`, and `msvc`. The gcc presets expect `gcc` on `PATH` — with
w64devkit that is:

```bat
set PATH=C:\Program Files (x86)\w64devkit\bin;%PATH%
```

Or point CMake at it directly, without touching `PATH`:

```bat
cmake -S . -B build/gcc -G "MinGW Makefiles" ^
  -DCMAKE_C_COMPILER="C:/Program Files (x86)/w64devkit/bin/gcc.exe" ^
  -DCMAKE_MAKE_PROGRAM="C:/Program Files (x86)/w64devkit/bin/mingw32-make.exe"
```

Use a **fresh build directory** when switching compiler or generator — CMake cannot
change either in an existing one, and will refuse.

For machine-specific paths, put them in `CMakeUserPresets.json` (gitignored) rather
than editing `CMakePresets.json`, which ships to customers.

### Options

| Option | Default | Effect |
|---|---|---|
| `J30D_PORT` | `none` | Port to compile: `none`, `stm32-baremetal`, `stm32-freertos`. Needs your project's include paths. |
| `J30D_USE_BUNDLED_CONFIG` | `ON` | Compile `config/j30d_config.c`. Turn off to define `j30d_default_config` yourself. |
| `J30D_NO_FLOAT` | `OFF` | Remove every float from the API — see below. |
| `J30D_ALLOW_NON_I2C_OUTPUT` | `OFF` | Lift the OUT_MODE guard. **Read the safety section first.** |
| `J30D_ALLOW_EEPROM_BLOW` | `OFF` | Lift the fuse-blow guard. Production tooling only. |
| `J30D_ALLOW_CALIBRATION_WRITE` | `OFF` | Lift the calibration-block guard. Production tooling only. |
| `J30D_RESET_SETTLE_DELAY_MS` | 50 | Post-soft-reset settle delay. |
| `J30D_EEPROM_POLL_ATTEMPTS` | 5 | EEPROM-loaded handshake poll count. |
| `J30D_EEPROM_POLL_DELAY_MS` | 10 | Delay between those polls. |
| `J30D_BUILD_TESTS` | on if top-level | Build the host test suite. |
| `J30D_WERROR` | `OFF` | `-Werror`. Recommended in CI. |

Without CMake, all of these are plain `-D` defines.

### FPU-less targets

`-DJ30D_NO_FLOAT` removes all floating point from the driver: `j30d_field_t` loses
`.value`, and `j30d_calib_t` carries its trim data as raw fixed-point words. On a
Cortex-M0+ every conversion is a soft-float call and the library those pull in costs
several KB of flash. Nothing is lost — `.raw_code` is still there and the scale
relationship is published as `J30D_*_SCALE_SHIFT` constants in both builds.

It changes public struct layouts, so it must be set consistently across **every**
translation unit.

## Safety guards

Three writes can leave a unit in a state this driver cannot talk it out of. All are
refused with `J30D_ERR_PARAM` before the bus is touched, on both the config path and
`j30d_raw_write()`:

- **SYS_CFG1 (0x93) OUT_MODE moved off I²C.** This multiplexes SDA into a push-pull
  output and I²C stops responding. SYS_CFG1 is EEPROM-backed, so it survives power
  cycles and soft resets — the unit comes back up unreachable every time. The guard
  is *one-directional*: writing OUT_MODE back **to** I²C is always allowed, so a unit
  already in this state can still be recovered. Other SYS_CFG1 bits (OSR, filter) are
  unrestricted.
- **BLOW_START (0x40)** — blows the EEPROM fuse. Irreversible.
- **The factory calibration block (0xC0–0xEF)** — destroys the unit's calibration
  permanently. The driver only ever reads it.

A multi-byte write that merely *spans* one of these is refused too. Each has its own
build-time opt-out (above) — build-time, so enabling one is a decision recorded in
the build rather than something a stray call can reach.

Not guarded: a raw write to `SOFT_RESET`, which resets the device behind the driver's
back and invalidates its cached state. Call `j30d_init()` again instead.

## Things worth knowing

**Freshness.** `j30d_get_data()`'s return code reports transport health only. Each of
the six fields is marked `J30D_FRESH` / `J30D_STALE` / `J30D_ERROR` independently,
gated on its *own* data-ready bit — even where several fields share one burst read.
In the default single-conversion mode only the converting channel is ever fresh, so
`ch2` staying `J30D_STALE` is the honest answer, not a fault.

**Interrupt mode makes the bus exclusive.** Setting `config.interrupt_enable` uses the
device's sleep-mode INT line to signal that a new sample is available. The device has
no dedicated data-ready interrupt; the driver sets the sleep threshold to its minimum
(`0xAC/0xAD` = `0x8000`, −128 mV) so that every sample meets the comparator condition,
and hands SDA to the device on entry to sleep. **While the device sleeps, SDA does not
belong to the I²C peripheral, so no other device on that bus can be reached — for up
to 25.6 s at a time.** Do not enable interrupt mode on a shared bus. It also requires both
`platform.release_to_int` and `platform.now_ms`; `j30d_init()` returns
`J30D_ERR_PARAM` without either, and likewise if `SLEEP_EN` is not set — INT is the
sleep-mode comparator output on SDA, so with sleep off the pin is never handed over and
no interrupt can fire. `now_ms` is not optional here: commanding sleep makes
the device convert and raise INT again a few ms later, and the pacing is the only thing
that rejects those — without a clock there is no pacing, and the device never completes
a sleep interval.

Wire your EXTI handler to call `j30d_notify_interrupt()`, the only function in this
API that may be called from interrupt context, then signal a task and call
`j30d_get_data()` from that task. `j30d_get_data()` must not be called from the
handler: it blocks for tens of milliseconds and, on an RTOS port, sleeps. The flag is
recorded only and does not change what that read does: the pacing described below
applies to interrupt-driven reads as well, and is what rejects the extra interrupt the
device raises a few ms after every sleep command.

**Polling cost.** Supply `platform.now_ms` and `j30d_get_data()` paces itself to the
device's configured sleep interval: calling it faster cannot make it wake the device
more often than the sensor could possibly produce a new sample. Without it, every
call on a sleeping device pays a full wake.

In interrupt mode the threshold is **half** the configured interval instead. The
pacing has a second job there — rejecting the INT the device raises a few ms after
every sleep command — and the full interval cannot do both: `SLEEP_TIME` is untrimmed
(±15% on a 32 kHz oscillator), so a fast part raises its genuine interrupt before the
nominal interval has elapsed and would be rejected along with the extra one, halving
the sample rate on a part that is in spec.

**Config model.** Each field in `j30d_config_t` carries its own status, so
`j30d_update_config()` writes only what changed. `J30D_FIELD_UNCHANGED` means "leave
the device alone", `J30D_FIELD_DEFAULT` means "use the compiled-in default",
`J30D_FIELD_UPDATED` means "write my value". At `j30d_init()`, `command` and `sleep`
may **not** be `UNCHANGED` — `init` soft-resets the device, which returns those RAM
registers to their reset values, so there is nothing left to preserve; pass `NULL`
for defaults or state what you want. `h->config` always reflects what the device
actually holds, so the read-modify-write idiom is safe:

```c
j30d_config_t cfg = *j30d_get_config(&h);
cfg.sleep.status = J30D_FIELD_UPDATED;
cfg.sleep.value  = 0x0E;                  /* (0x0E + 1) * 100ms = 1500ms */
j30d_update_config(&h, &cfg);
```

**`raw_amb` is not in millivolts.** The ambient sensor bypasses the analog front end,
so it has no PGA gain and no millivolt equivalent — `raw_amb.value` is the unscaled
code. `raw_ch1`/`raw_ch2` do run through the PGA and are genuine millivolts.

**Locking.** `platform.lock` returns `bool`. If it reports failure, the call is
abandoned with `J30D_ERR_TIMEOUT` before any bus access and `unlock` is not called —
a timed-out mutex must never let a caller proceed unserialized. If other devices
share the I²C bus, they must cooperate on the *same* lock: `return_to_iic()`
reconfigures the bus pins for everyone.

## Versioning

`J30D_API_VERSION` versions the **source-level API**, not an ABI. `j30d_handle_t`
embeds driver-private state by value, so its size and layout can change in any
release. Compile the driver's `.c` files and your application against the same
revision of these headers; never link a pre-built object against different ones.

Releases are tagged `vMAJOR.MINOR.PATCH`. **Pin a tag** rather than tracking a
branch, so that the version `j30d_get_version_string()` reports corresponds to an
exact revision — that is what a support request needs in order to be actionable.
[`CHANGELOG.md`](CHANGELOG.md) records what changed in each one, including any
change that breaks existing source.

While the major version is 0, a MINOR bump may carry a source-breaking change.
Read the changelog entry before upgrading.

## Testing

`test/fake_sensor_test.c` runs the real driver against an in-memory fake that models
the awkward parts of the hardware — sleep cutting off I²C mid-sequence, and the
status registers clearing *all* their data-ready bits when read. `ctest` builds and
runs it across the float / no-float and guarded / unguarded combinations, since those
change struct layouts and refusal behaviour.

`test/j30d_live_test.c` is a separate on-target stress test: it hammers config
changes, sleep transitions and raw register access from several FreeRTOS tasks at
once against a real sensor, then re-inits to prove recovery. It is not wired into any
boot path — call `J30D_LiveTest_Run()` yourself. Read its header before extending it;
it deliberately stays away from the registers listed under Safety guards.

## Licence

See [LICENSE.txt](LICENSE.txt).
