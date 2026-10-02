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
 *  - Optional platform.lock/unlock hooks let the driver serialize calls on
 *    the SAME handle across threads (FreeRTOS mutex, Zephyr k_mutex, or a
 *    critical section). On a single-threaded bare-metal build, leave them
 *    NULL and no locking happens at all. If lock reports failure (it
 *    returns bool -- see j30d_platform.h), the call is abandoned with
 *    J30D_ERR_TIMEOUT before any bus access rather than proceeding
 *    unserialized, so every j30d_* function below can return
 *    J30D_ERR_TIMEOUT for that reason alone. True parallel I2C transactions
 *    are never possible (single physical bus) -- the guarantee locking
 *    provides is "callers block and don't corrupt state", not "run at the
 *    same time".
 *  - Sleep is handled for you. If sleep ends up enabled in h->config
 *    (whether via j30d_init()'s config or a later j30d_update_config()),
 *    every call below that needs the bus wakes the device, stops the
 *    conversion engine so a burst read cannot straddle a conversion, and
 *    re-commands sleep afterward, automatically. There is no public
 *    "wake" call and none of the functions below need you to think about
 *    whether the device happens to be asleep right now.
 *  - If you also give platform.now_ms, j30d_get_data() paces itself to the
 *    device's own configured sleep interval -- calling it faster than that
 *    (from a fast poll loop, multiple threads, whatever) will not wake the
 *    device or touch the bus more often than the interval allows; you just
 *    get the same last known-good data back, marked stale, on the calls
 *    that land too soon. See j30d_get_data()'s doc comment below.
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
 *       if (data.ch1.state == J30D_FRESH) { ... }
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
	j30d_calib_t      calib;         /* factory calibration/production data, read once at init -- see j30d_calib.h.
	                                   * Best-effort: a read/parse failure leaves calib.valid == false and
	                                   * calib.version == J30D_UNIT_VERSION_UNKNOWN rather than failing
	                                   * j30d_init() outright -- units without calibration data are expected
	                                   * to exist (see j30d_unit_version_t). */
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
 * Resets the device, applies `config` (or j30d_default_config if `config`
 * is NULL), and waits for the EEPROM-loaded handshake (polling
 * J30D_REG_ERROR_CODE, see j30d_types.h's J30D_ERR_EEPROM_NOT_LOADED).
 *
 * Must be called once per handle before any other j30d_* function is used
 * on it. Safe to call again later to fully re-init (e.g. after a suspected
 * bus fault) -- it re-copies `platform` and reapplies `config` from
 * scratch.
 *
 * `h` and `platform` must not be NULL; `platform->i2c_read`, `i2c_write`,
 * `delay_ms` and `return_to_iic` must all be set (J30D_ERR_PARAM otherwise).
 *
 * If `config` is non-NULL, its `command` and `sleep` fields must NOT be
 * J30D_FIELD_UNCHANGED (J30D_ERR_PARAM otherwise). Those two are RAM
 * registers that this function's own soft reset returns to their hardware
 * reset values, so "leave whatever is currently on the device" has nothing
 * left to preserve by the time it would apply -- and COMMAND in particular
 * drives every sleep decision this driver makes, so it must be a value the
 * caller actually chose. Pass config == NULL to take j30d_default_config
 * for everything. The five EEPROM-backed fields DO survive the reset, so
 * J30D_FIELD_UNCHANGED is honoured for them: those registers are read back
 * off the device to seed h->config, which therefore reflects what the
 * device really holds rather than the compiled-in defaults.
 */
j30d_err_t j30d_init(j30d_handle_t *h, const j30d_platform_t *platform, const j30d_config_t *config);

/*
 * Writes only the fields marked J30D_FIELD_UPDATED in `config` to the
 * device (RAM registers immediately, EEPROM registers per the device's
 * normal EEPROM write sequence), then merges the written values into
 * h->config. Fields left J30D_FIELD_UNCHANGED are left alone both on the
 * device and in h->config. Fields marked J30D_FIELD_DEFAULT are written
 * from j30d_default_config.
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
 * strength of some other field's bit. In practice this means that in
 * single-conversion mode (J30D_COMMAND_MODE_SEL_SINGLE, the default) only
 * the converting channel is ever reported fresh and the other stays
 * J30D_STALE -- which is the honest answer: the device produced no sample
 * for it. If the DRDY bit is set but reading the value itself
 * fails, the field is marked J30D_ERROR and the value is likewise left
 * unchanged.
 *
 * The function's own return code reflects transport health only: J30D_OK
 * unless the wake failed to stop the conversion engine, or the initial
 * status-register read failed (in both cases nothing in `out` is touched),
 * or at least one field's read failed (J30D_ERR_I2C/TIMEOUT -- check
 * per-field .state in `out` to see which).
 *
 * Returns J30D_ERR_NOT_INIT if called before a successful j30d_init().
 * Wakes the device and stops the conversion engine first (and re-commands
 * sleep afterward) if sleep is currently configured -- see the file header
 * note on sleep handling.
 *
 * Paced to the sleep interval if platform.now_ms is set -- or to half of it
 * in interrupt mode, where the pacing must also reject the INT the device
 * raises a few ms after every sleep command without rejecting a genuine one
 * from a part whose untrimmed sleep timer runs fast. If sleep is
 * configured and less time than that has passed
 * since this function last actually woke the device, it returns
 * immediately without touching the bus at all -- `out` is filled from the
 * same last known-good data as always, with every field's .state forced
 * to J30D_STALE (there was, by construction, no opportunity for anything
 * new). Call as often as you like from as many threads as you like; you
 * cannot make this function wake the device more often than its own
 * configured sleep interval. Without platform.now_ms, every call wakes
 * the device if sleep is configured, same as before -- see j30d_platform.h.
 */
j30d_err_t j30d_get_data(j30d_handle_t *h, j30d_data_t *out);

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
 * covers (COMMAND, SLEEP_TIME, SYS_CFG1/2, SYS_CH1/2_CFG, BIAS_CFG --
 * including as part of a multi-byte burst that spans one) updates
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
/* Records an interrupt on the J30D's INT line. The only function here that
 * may be called from interrupt context: it stores one flag, nothing else.
 *
 * Call it from your EXTI handler, then signal a task and call
 * j30d_get_data() from that task -- never from the handler, which would
 * block for tens of milliseconds and, on an RTOS port, sleep.
 *
 * The flag is recorded only: it does not change what the next
 * j30d_get_data() does. The sleep-interval pacing applies to
 * interrupt-driven reads too, and is what rejects the extra interrupt the
 * device raises a few ms after every sleep command. Extra calls are
 * harmless, as is calling it with interrupt mode off. h may be NULL. */
void j30d_notify_interrupt(j30d_handle_t *h);

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

#ifdef __cplusplus
}
#endif

#endif /* J30D_H_ */
