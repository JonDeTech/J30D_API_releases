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
    if (d.raw_ch1.state == J30D_FRESH) {
        use(d.raw_ch1.value);      /* mV at the PGA input -- what heat flux is computed from */
        log(d.raw_ch1.raw_code);   /* the unscaled ADC code it came from */
    }
    /* fields that were not ready are J30D_STALE and still hold their
       last known-good value -- they are never left undefined */
}
```

That is the whole hot path. `j30d_get_data()` refreshes all six fields in one call.

**3. Configure.** `j30d_init(&h, &platform, NULL)` applies the defaults in
[`config/j30d_config.c`](config/j30d_config.c). To start from them with changes, copy
them and set fields. `J30D_SET()` sets a field's value and marks it to be written in
one go; the values themselves (gain, OSR, clock, mode, ...) are in
[`inc/j30d_config.h`](inc/j30d_config.h), which `j30d.h` includes.

```c
j30d_config_t cfg = j30d_default_config;
cfg.ch1_cfg = J30D_SET(J30D_SYS_CHx_CFG_OSRP_8192 | J30D_SYS_CHx_CFG_GAIN_32);
cfg.sleep   = J30D_SET(J30D_SLEEP_TIME_MS(1500));   /* 100 ms steps, 100 ms .. 25.6 s */
if (j30d_init(&h, &platform, &cfg) != J30D_OK) { /* handle it */ }
```

Later, change one setting and leave the rest alone — a zeroed `j30d_config_t` has
every field `J30D_FIELD_UNCHANGED`:

```c
j30d_config_t upd = {0};
upd.ch1_cfg = J30D_SET(J30D_SYS_CHx_CFG_OSRP_8192 | J30D_SYS_CHx_CFG_GAIN_64);
j30d_update_config(&h, &upd);
```

`J30D_SLEEP_TIME_MS()` refuses at compile time anything that is not a multiple of
100 ms between 100 ms and 25.6 s. The factory calibration assumes the default
1500 ms: another interval changes the ASIC's self-heating and introduces an offset.
In a static table such as `j30d_default_config`, use `J30D_SET_INIT()` instead of
`J30D_SET()`.

## Layout

| Path | What it is |
|---|---|
| `inc/` | Public headers. `j30d.h` is the entry point and includes the rest. |
| `src/` | The driver core. Portable, no platform knowledge. |
| `config/j30d_config.c` | **Yours to edit.** The compiled-in `j30d_default_config`. |
| `ports/` | Reference platform implementations — see the caveat below. |
| `test/fake_sensor_test.c` | Host test suite. Runs anywhere, no hardware. |
| `CHANGELOG.md` | What changed in each release, and what to do about it. |

**About `ports/`:** these `#include` *your* project's headers (`main.h`, `i2c.h`, the
vendor HAL, FreeRTOS), so they do not compile standalone in this repository. Treat
them as reference implementations to copy into your tree and adjust. The FreeRTOS port
needs `configTICK_RATE_HZ` to divide 1000 (1000, 500, 200, 100 Hz, ...) and refuses to
compile otherwise.
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
the device convert and raise INT one conversion later, and the pacing is the only thing
that rejects those — without a clock there is no pacing, and the device never completes
a sleep interval.

Interrupt mode is chosen at `j30d_init()`; `j30d_update_config()` ignores
`interrupt_enable`. Clearing `SLEEP_EN` with `j30d_update_config()` pauses it, since an
awake device raises no INT, and setting `SLEEP_EN` again resumes it.

Wire your EXTI handler to call `j30d_notify_interrupt()`, the only function in this
API that may be called from interrupt context, then signal a task and call
`j30d_get_data()` (or `j30d_get_data_blocking()`) from that task. Neither may be called
from the handler: they block for tens of milliseconds and, on an RTOS port, sleep. The
pacing described below applies to interrupt-driven reads as well, and is what rejects
the extra interrupt the device raises one conversion after every sleep command —
answer every INT promptly, so that one is consumed while it is still recognisable.

**Sample timing.** With sleep on, `SLEEP_TIME` is the gap between conversions, not the
sample period. Every read wakes the device and ends by commanding sleep again, which
restarts its cycle: it converts, sleeps for `SLEEP_TIME`, then converts again. A wake
before that second conversion is complete halts it, and the cycle starts over — so a
host that keeps waking the device too early never sees a sample at all. Conversion
time is `(OSRP + 2 × OSRT + 1481) / Fosc_kHz` ms, about 9 ms at the defaults.

Supply `platform.now_ms` and `j30d_get_data()` counts from the last sleep command and
does not wake the device until that sample is complete on any part in spec (`SLEEP_TIME`
is untrimmed, ±15% on a 32 kHz oscillator); earlier calls return the last data, marked
stale, without touching the bus. To poll on a timer, use a period of at least
`j30d_get_sample_interval_ms()` — 1796 ms at the defaults, not 1500 — and every call
returns new data. Without `now_ms`, calls closer together than that can return no new
data at all.

In interrupt mode an INT marks the sample, so with one pending the pacing only has to
reject the INT raised by the conversion on entry to sleep: it waits for that conversion
plus half the shortest possible sleep interval, which separates the two at every
`SLEEP_TIME`. With no INT pending — a timer-driven call, or a lost interrupt — the full
polling wait applies, so a read never lands before the sample is complete.

**Blocking read.** `j30d_get_data_blocking(&h, &d, timeout_ms)` waits for the next
sample and returns `J30D_OK` only when `d.raw_ch1` is `J30D_FRESH`. It is
`j30d_get_data()` called at the right moments, so it never wakes the device sooner and
never returns a conversion taken while the device was awake. In polling it returns
after the worst-case sample time; in interrupt mode, as soon as the INT is recorded.
It waits in `platform.delay_ms()` with the handle unlocked, needs `platform.now_ms`,
and returns `J30D_ERR_TIMEOUT` if nothing new arrives in time.

**Never stall SCL.** Holding SCL low for about 10 ms is the device's return-to-IIC
signal, the same one that wakes it from sleep. On a device that is awake and converting
it clears `COMMAND`, so conversions stop and the data stops changing. The threshold
varies from unit to unit and can be below 10 ms. A master holds SCL low whenever its
software is late in the middle of a transfer, for instance a task preempted between the
register byte and the read. Your `i2c_read`/`i2c_write` must therefore never pause for
milliseconds: keep each transfer free of preemption or starvation. The FreeRTOS port
suspends the scheduler around each HAL transfer for this reason. Converting
continuously (sleep off), `j30d_get_data()` also checks for it. If no new data has come
for two conversion times, it reads `COMMAND` back. If conversions had stopped, it
restarts them and returns `J30D_ERR_RESTARTED`, which tells you a transfer on your bus
stalled.

**Config model.** Each field in `j30d_config_t` carries its own status, so
`j30d_update_config()` writes only what changed. `J30D_FIELD_UNCHANGED` means "leave
the device alone", `J30D_FIELD_DEFAULT` means "use the compiled-in default",
`J30D_FIELD_UPDATED` means "write my value". At `j30d_init()`, `command` and `sleep`
may **not** be `UNCHANGED` — `init` soft-resets the device, which returns those RAM
registers to their reset values, so there is nothing left to preserve. That holds for
`j30d_default_config` too when you pass `NULL`. `h->config` always reflects what the
device actually holds, so the read-modify-write idiom is safe:

```c
j30d_config_t cfg = *j30d_get_config(&h);
cfg.sleep = J30D_SET(J30D_SLEEP_TIME_MS(1500));
j30d_update_config(&h, &cfg);
```

**`raw_amb` is not in millivolts.** The ambient sensor bypasses the analog front end,
so it has no PGA gain and no millivolt equivalent — `raw_amb.value` is the unscaled
code. `raw_ch1`/`raw_ch2` do run through the PGA and are genuine millivolts.

**`ch1`/`ch2` are not the heat-flux input.** They are the ASIC's corrected thermopile
voltage from its CAL registers, in millivolts, scaled by `SYS_CFG2`'s `VT_SCALE`
(±16 mV full scale when clear, ±128 mV when set). The factory heat-flux calibration
in `h->calib` applies to `raw_ch1`.

**Heat flux.** `j30d_heat_flux()` applies the unit's factory calibration to a `raw_ch1`
sample and gives mW/cm² as a `float`; `j30d_heat_flux_x1000()` gives the same as an
`int32_t` in mW/cm² × 1000 (12345 = 12.345 mW/cm²), and is the one available in
`J30D_NO_FLOAT` builds. Both are pure arithmetic — read the sample first, ideally with
`j30d_get_data_blocking()`, whose `J30D_OK` means it is fresh:

```c
j30d_data_t d;
float       hf;
if (j30d_get_data_blocking(&h, &d, 5000) == J30D_OK &&
    j30d_heat_flux(&h, &d.raw_ch1, &hf) == J30D_OK) {
    use(hf);   /* mW/cm2 */
}
```

Units with no usable calibration (the earliest CH2 units, or a block that failed its
check) return `J30D_ERR_NO_CALIB`. The formula depends on the unit generation; both are
in `j30d.h`. The calibration assumes the default 1500 ms `SLEEP_TIME`. The STM32 ports'
headers have complete examples.

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

## Licence

See [LICENSE.txt](LICENSE.txt).
