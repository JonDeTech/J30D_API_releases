/*
 * j30d.h
 *
 * Public API for the J30D / JIRS30D sensor driver.
 *
 * Design summary:
 *  - No global/static driver state anywhere -- every call takes an explicit
 *    j30d_handle_t*, so multiple sensors (multiple handles) are naturally
 *    supported and two threads touching *different* handles are safe with
 *    zero locking.
 *  - All platform/OS/I2C specifics are supplied via j30d_platform_t
 *    function pointers (see j30d_platform.h) -- this file and j30d.c have
 *    no dependency on any vendor HAL, RTOS, or GPIO register.
 *  - Optional platform.lock/unlock hooks serialize calls on the SAME handle
 *    across threads (FreeRTOS mutex, Zephyr k_mutex, or a critical section);
 *    leave them NULL on a single-threaded build. A lock that reports failure
 *    abandons the call with J30D_ERR_TIMEOUT before any bus access, so every
 *    j30d_* function below can return J30D_ERR_TIMEOUT for that reason alone.
 *  - Sleep is handled for you. If sleep is configured, every call below that
 *    needs the bus wakes the device, stops the conversion engine so a burst
 *    read cannot straddle a conversion, and re-commands sleep afterwards.
 *  - Given platform.now_ms, j30d_get_data() paces itself to the device's
 *    sample timing -- see its doc comment and
 *    j30d_get_sample_interval_ms() below.
 *
 * Usage sketch:
 *
 *   j30d_handle_t h;
 *   j30d_platform_t platform = { .i2c_read = ..., .i2c_write = ...,
 *                                 .delay_ms = ..., .return_to_iic = ...,
 *                                 .ctx = &my_i2c_bus };
 *   if (j30d_init(&h, &platform, NULL) != J30D_OK) { ... }
 *
 *   j30d_data_t data;
 *   if (j30d_get_data(&h, &data) == J30D_OK) {
 *       if (data.raw_ch1.state == J30D_FRESH) { ... }   -- the heat-flux input, mV
 *   }
 */

#ifndef J30D_H_
#define J30D_H_

#include <stdint.h>
#include <stdbool.h>

#include "j30d_version.h"
#include "j30d_types.h"
#include "j30d_platform.h"
#include "j30d_config.h"
#include "j30d_calib.h"
#include "j30d_internal.h"   /* driver-private working state -- do not access directly */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	j30d_platform_t   platform;      /* copied in at j30d_init() */
	j30d_config_t     config;        /* shadow of what's currently applied on the device */
	j30d_calib_t      calib;         /* factory calibration, read best-effort at init -- see j30d_calib.h */
	j30d_internal_t   internal;      /* driver-private -- do not read/write directly */
	bool              initialized;
} j30d_handle_t;

/* NOTE ON BINARY COMPATIBILITY: j30d_handle_t embeds j30d_internal_t BY
 * VALUE, so any change to driver-private state changes this struct's size
 * and layout. There is deliberately no attempt to keep a stable ABI: this
 * driver ships as source, and the headers and the .c files must be compiled
 * together from the same revision. Do not link a pre-built object or
 * library of this driver against a different revision's headers. See
 * j30d_version.h -- J30D_API_VERSION describes the SOURCE-level API
 * contract, not an ABI. */

/*
 * Resets the device, waits for the EEPROM-loaded handshake (polling
 * J30D_REG_ERROR_CODE; J30D_ERR_EEPROM_NOT_LOADED if it never completes),
 * and applies `config`, or j30d_default_config if `config` is NULL.
 *
 * Must be called once per handle before any other j30d_* function is used
 * on it. May be called again to re-initialise from scratch (e.g. after a
 * suspected bus fault). It takes `platform`'s lock before touching `h`, so
 * with the same platform it is serialized against other calls on the
 * handle; if the lock cannot be acquired it returns J30D_ERR_TIMEOUT and
 * leaves `h` as it was.
 *
 * `h` and `platform` must not be NULL; `platform->i2c_read`, `i2c_write`,
 * `delay_ms` and `return_to_iic` must all be set (J30D_ERR_PARAM otherwise).
 *
 * The config is checked as it will be applied -- j30d_default_config when
 * `config` is NULL, and j30d_default_config's value for any field marked
 * J30D_FIELD_DEFAULT -- before anything is written (J30D_ERR_PARAM
 * otherwise):
 *   - `command` and `sleep` must not be J30D_FIELD_UNCHANGED. They are RAM
 *     registers that the soft reset returns to their reset values, so there
 *     is nothing left to preserve, and COMMAND drives every sleep decision
 *     this driver makes.
 *   - `interrupt_enable` requires platform.release_to_int, platform.now_ms
 *     and SLEEP_EN in `command` -- see j30d_config.h.
 * The other six fields may be J30D_FIELD_UNCHANGED: they are read back from
 * the device after the reset, so h->config reflects what the device holds.
 */
j30d_err_t j30d_init(j30d_handle_t *h, const j30d_platform_t *platform, const j30d_config_t *config);

/*
 * Writes the fields marked J30D_FIELD_UPDATED (their own .value) or
 * J30D_FIELD_DEFAULT (j30d_default_config's value) and merges them into
 * h->config. Fields left J30D_FIELD_UNCHANGED are left alone on the device
 * and in h->config, so a zero-initialised j30d_config_t with one field set
 * changes that field only.
 *
 * `interrupt_enable` is ignored: interrupt mode is chosen at j30d_init().
 * Clearing SLEEP_EN here pauses it -- an awake device raises no INT -- and
 * setting SLEEP_EN again resumes it.
 *
 * Returns J30D_ERR_NOT_INIT if called before a successful j30d_init().
 */
j30d_err_t j30d_update_config(j30d_handle_t *h, const j30d_config_t *config);

/*
 * Refreshes all of `out`'s fields (ch1, ch2, amb, raw_ch1, raw_ch2,
 * raw_amb) in one call; a subset cannot be requested. Reading the
 * DATA_READY/DATA_READY_RAW status registers clears every DRDY bit,
 * including those for fields you did not fetch, so a partial read would
 * starve them. Look at which fields came back J30D_FRESH and ignore the
 * rest.
 *
 * For each of the six fields, if the device's corresponding DRDY bit was
 * set, the register(s) are read and the field is marked J30D_FRESH; if
 * clear, the field is marked J30D_STALE and its value is left as the last
 * known-good reading. Each field is gated on its OWN DRDY bit even where
 * several share one burst read (ch1/ch2 come from one 6-byte read, the
 * three raw fields from one 9-byte read) -- sharing a transaction is a
 * transport detail and never promotes a field to J30D_FRESH on the
 * strength of some other field's bit. In single-conversion mode
 * (J30D_COMMAND_MODE_SEL_SINGLE, the default) only the converting channel
 * is ever reported fresh; the other stays J30D_STALE because the device
 * produced no sample for it. If the DRDY bit is set but reading the value
 * fails, the field is marked J30D_ERROR and its value is left unchanged.
 *
 * The return code reflects transport health only: J30D_OK unless the wake
 * failed to stop the conversion engine, or the initial status-register read
 * failed (in both cases nothing in `out` is touched), or at least one
 * field's read failed (J30D_ERR_I2C/TIMEOUT -- check per-field .state in
 * `out` to see which).
 *
 * Returns J30D_ERR_NOT_INIT if called before a successful j30d_init().
 *
 * Paced when sleep is configured and platform.now_ms is set. Every sleep
 * command -- this function's own, and those of j30d_init(),
 * j30d_update_config() and the raw calls -- restarts the device's
 * conversion cycle, and a wake halts a conversion under way. So until the
 * conversion ending the first sleep interval is certainly complete, on any
 * part within the oscillator tolerance, this function returns at once
 * without touching the bus, filling `out` from the last known-good data
 * with every .state J30D_STALE. In interrupt mode an INT recorded by
 * j30d_notify_interrupt() lets it read sooner, once past the INT the device
 * raises on entry to sleep; with no INT pending the full polling wait
 * applies.
 *
 * Without platform.now_ms every call on a sleeping device wakes it, and
 * calls closer together than j30d_get_sample_interval_ms() can then return
 * no new data at all: each one halts the conversion the last one started.
 *
 * Converting continuously (sleep off, MODE_EN set), the device can stop on
 * its own: an I2C transfer that holds SCL low for about 10 ms clears its
 * COMMAND register (see j30d_platform.h). So when a call finds no new data
 * and none has arrived for longer than two conversions, it reads COMMAND
 * back, and if conversions have stopped it writes COMMAND again and returns
 * J30D_ERR_RESTARTED, with `out` filled as for any call with no new data.
 * The next sample follows one conversion later. Without platform.now_ms it
 * reads COMMAND on every call that finds no new data. J30D_ERR_RESTARTED
 * means a transfer on your bus stalled; the fix belongs in the port.
 */
j30d_err_t j30d_get_data(j30d_handle_t *h, j30d_data_t *out);

/* Records an interrupt on the J30D's INT line. The only function here that
 * may be called from interrupt context: it stores one flag, nothing else.
 *
 * Call it from your EXTI handler, then signal a task and call
 * j30d_get_data() or j30d_get_data_blocking() from that task -- never from
 * the handler, which would block for tens of milliseconds and, on an RTOS
 * port, sleep.
 *
 * In interrupt mode a pending flag lets the next j30d_get_data() read
 * before the full polling wait, provided the window after the last sleep
 * command -- in which the device raises an INT for its conversion on entry
 * to sleep -- has passed. Each j30d_get_data() clears the flag. So call it
 * only for the J30D's INT, and answer every INT promptly: an INT left
 * pending past that window is taken as the sample. Outside interrupt mode
 * it has no effect. h may be NULL. */
void j30d_notify_interrupt(j30d_handle_t *h);

/*
 * Waits for the next new sample and returns it. J30D_OK means raw_ch1 --
 * the heat-flux input -- is J30D_FRESH; the other fields carry their own
 * .state as with j30d_get_data().
 *
 * Every attempt goes through j30d_get_data() and its pacing, so with sleep
 * configured each sample it returns comes from the conversion that ends a
 * full sleep interval -- never from one taken while the device is awake.
 * In polling it waits out the worst-case sample time (see
 * j30d_get_sample_interval_ms()). In interrupt mode it returns as soon as
 * the INT marking the sample has been recorded by j30d_notify_interrupt(),
 * which on a nominal part is sooner; if no INT arrives it still returns
 * after the worst-case time. Without sleep it returns the next conversion.
 *
 * Waits in platform.delay_ms() without holding the platform lock, so other
 * calls on the handle can run meanwhile -- though one that re-commands
 * sleep restarts the wait. Returns J30D_ERR_TIMEOUT if no new sample has
 * arrived within timeout_ms, with `out` holding what the last attempt
 * returned; transport errors and J30D_ERR_RESTARTED are returned as
 * j30d_get_data() reports them.
 * Requires platform.now_ms (J30D_ERR_PARAM otherwise). Never call it from
 * interrupt context.
 */
j30d_err_t j30d_get_data_blocking(j30d_handle_t *h, j30d_data_t *out, uint32_t timeout_ms);

/*
 * Heat flux from a raw_ch1 sample, applying the unit's factory calibration
 * (h->calib, read at j30d_init()):
 *
 *   REV1, REV2PLUS     HF = gain1 * raw_ch1 [mV] + offset1
 *   CH1_NOTIMESTAMP    HF = gain1 * 1e-5 * raw_ch1 [raw code] - offset1
 *
 * j30d_heat_flux() gives HF in mW/cm2 as a float. j30d_heat_flux_x1000()
 * gives the same quantity as an integer in mW/cm2 x 1000 -- 12345 means
 * 12.345 mW/cm2 -- and is the one available in J30D_NO_FLOAT builds.
 *
 *   j30d_data_t d;
 *   float       hf;
 *   if (j30d_get_data_blocking(&h, &d, 5000) == J30D_OK &&
 *       j30d_heat_flux(&h, &d.raw_ch1, &hf) == J30D_OK) { ... hf, mW/cm2 ... }
 *
 * Pure arithmetic: no bus access, no lock. It converts whatever the field
 * holds, so check raw_ch1->state, or take the sample from
 * j30d_get_data_blocking(), whose J30D_OK means it is fresh.
 *
 * Returns J30D_ERR_NO_CALIB for a unit whose calibration is missing or not
 * trusted (calib.valid false, or version CH2 / UNKNOWN), and J30D_ERR_PARAM
 * / J30D_ERR_NOT_INIT as usual.
 *
 * The CH1_NOTIMESTAMP form works on the raw code, so it holds only at the
 * PGA gain the unit was calibrated at. In J30D_NO_FLOAT builds the mV form
 * uses the ch1 gain in effect now, so convert a sample before changing
 * ch1_cfg. The calibration assumes the default 1500 ms SLEEP_TIME -- see
 * J30D_SLEEP_TIME_MS in j30d_config.h.
 */
#if !defined(J30D_NO_FLOAT)
j30d_err_t j30d_heat_flux(const j30d_handle_t *h, const j30d_field_t *raw_ch1, float *mw_cm2);
#endif
j30d_err_t j30d_heat_flux_x1000(const j30d_handle_t *h, const j30d_field_t *raw_ch1, int32_t *mw_cm2_x1000);

/*
 * Raw register pass-through -- bypasses the config/data model entirely.
 * For debug/production tooling that needs to read or write an arbitrary
 * register address (e.g. driven by a command arriving over UART/USB from
 * a host tool), not for normal application use: prefer
 * j30d_update_config()/j30d_get_data() for anything the config/data model
 * already covers, since those also apply the field-freshness contract --
 * this does not. Both take the same platform lock as every other j30d_*
 * call, so raw access is correctly serialized against everything else
 * happening on this handle (and, if you've shared the lock across a whole
 * bus per j30d_platform.h, against other devices on it too). Both also
 * wake/re-sleep automatically like j30d_get_data().
 *
 * A successful j30d_raw_write() that lands on a register the config model
 * covers (COMMAND, SLEEP_TIME, CHIP_ADDRESS, SYS_CFG1/2, SYS_CH1/2_CFG,
 * BIAS_CFG -- including as part of a multi-byte burst that spans one) updates
 * h->config's shadow to match, before the automatic re-sleep step runs. So
 * writing COMMAND here does what it says: clear SLEEP_EN and the device is
 * left awake, set it and the device sleeps, and h->config agrees either
 * way.
 *
 * The one raw write still outside this: J30D_REG_SOFT_RESET. That resets
 * the device and reloads EEPROM behind the driver's back, invalidating the
 * shadow and the calibration data read at init -- call j30d_init() again
 * rather than trying to carry on.
 *
 * GUARDED WRITES. Three targets are refused with J30D_ERR_PARAM, before
 * the bus is touched, because they can leave a unit in a state this driver
 * cannot talk it out of. This applies to j30d_raw_write() and, for the
 * first one, to j30d_init()/j30d_update_config() as well:
 *
 *   - SYS_CFG1 (0x93) with OUT_MODE set to anything but I2C. That
 *     multiplexes SDA into a push-pull output and the IIC interface stops
 *     responding -- and because SYS_CFG1 is EEPROM-backed, it survives
 *     power cycles and soft resets, so the unit comes back in PWM/Relay
 *     mode every time. The guard is one-directional: writing OUT_MODE back
 *     TO I2C is always allowed, so a unit already in that state can still
 *     be recovered. Other SYS_CFG1 bits (OSR, filter) are unrestricted.
 *   - BLOW_START (0x40), which blows the EEPROM fuse irreversibly.
 *   - the factory calibration block, 0xC0..0xEF, which this driver only
 *     ever reads (see j30d_calib.h).
 *
 * A multi-byte write that merely spans one of these is refused too. If you
 * genuinely need one -- production tooling, or a product that really does
 * run the sensor in PWM mode -- build with J30D_ALLOW_NON_I2C_OUTPUT,
 * J30D_ALLOW_EEPROM_BLOW, or J30D_ALLOW_CALIBRATION_WRITE respectively.
 * They are build-time switches so that enabling one is a decision recorded
 * in the build, not something a stray call can reach.
 */
j30d_err_t j30d_raw_read(j30d_handle_t *h, uint8_t reg, uint8_t *buf, size_t len);
j30d_err_t j30d_raw_write(j30d_handle_t *h, uint8_t reg, const uint8_t *buf, size_t len);

/*
 * Read-only views of the handle's state. Prefer these over reaching into
 * j30d_handle_t directly: the members are visible only because this driver
 * never heap-allocates (see j30d_internal.h), not because poking at them is
 * supported, and going through a function keeps your code compiling if the
 * layout is rearranged.
 *
 * All three return NULL / false for a NULL handle or one that has not been
 * through a successful j30d_init(). None of them touch the bus, take the
 * platform lock, or block -- they are plain accessors, safe to call from
 * anywhere, including while another thread is mid-call on the same handle
 * (you may read a value that is about to change, but never a torn one:
 * every field is a single word or is only written under the lock).
 *
 * The pointers stay valid for the life of the handle. Contents change on
 * the next j30d_update_config() (config) or j30d_init() (both).
 */
const j30d_calib_t  *j30d_get_calib(const j30d_handle_t *h);
const j30d_config_t *j30d_get_config(const j30d_handle_t *h);
bool                 j30d_is_initialized(const j30d_handle_t *h);

/*
 * The polling period at which every j30d_get_data() call returns new data,
 * on any part within the oscillator tolerance. Set a polling timer to at
 * least this.
 *
 * With sleep configured, the sample period is not SLEEP_TIME. Each read ends
 * in a sleep command, after which the device converts, sleeps for
 * SLEEP_TIME and converts again, so a sample is ready that long after the
 * read -- plus the read itself. Both are taken at their longest:
 * SLEEP_TIME's oscillator is untrimmed (+/-15%). At the defaults (1.5 s,
 * OSR 8192/512, 1.2 MHz) this comes to 1796 ms. Without sleep, it is one
 * conversion at its longest.
 *
 * Calling more often is harmless with platform.now_ms set -- the extra
 * calls return without touching the bus -- but gains nothing. In interrupt
 * mode, read on the INT instead.
 *
 * Same standing as the accessors above: no bus access, no lock. 0 for a
 * NULL handle or one that has not been through a successful j30d_init().
 */
uint32_t             j30d_get_sample_interval_ms(const j30d_handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* J30D_H_ */
