/*
 * j30d.c
 *
 * Core driver implementation. No OS, HAL or GPIO calls anywhere in this
 * file: every side effect goes through the j30d_platform_t hooks.
 *
 * The driver does not select an active channel. Both ch1 and ch2 are
 * exposed, each marked fresh only when its own data-ready bit is set, so in
 * single-conversion mode ch2 simply stays stale.
 *
 * The factory calibration block is read at init (see j30d_calib.h), and
 * j30d_heat_flux() applies it to a raw_ch1 sample the caller has read --
 * pure arithmetic, no bus access.
 */

#include "j30d.h"
#include "j30d_regs.h"
#include "j30d_util.h"

#include <string.h>
#include <stdbool.h>

/* Init handshake timing. Overridable from the build (-D on the compiler
 * command line, or a project config header) so a customer can retune the
 * reset/EEPROM-load handshake for their board without editing this file --
 * the defaults below are what this EVK needs, not a device-wide constant.
 * Worst-case time in j30d_init() before J30D_ERR_EEPROM_NOT_LOADED is
 * J30D_RESET_SETTLE_DELAY_MS + ATTEMPTS * POLL_DELAY_MS. */
#ifndef J30D_RESET_SETTLE_DELAY_MS
#define J30D_RESET_SETTLE_DELAY_MS   50u
#endif
#ifndef J30D_EEPROM_POLL_ATTEMPTS
#define J30D_EEPROM_POLL_ATTEMPTS    5u
#endif
#ifndef J30D_EEPROM_POLL_DELAY_MS
#define J30D_EEPROM_POLL_DELAY_MS    10u
#endif

#if J30D_EEPROM_POLL_ATTEMPTS < 1u
#error "J30D_EEPROM_POLL_ATTEMPTS must be at least 1"
#endif

/* j30d_get_data() pacing -- see the sample-timing section below. Overridable
 * the same way. J30D_PACING_MARGIN_MS covers the resolution of
 * platform.now_ms: one tick of a 100 Hz clock. J30D_READ_OVERHEAD_MS is what
 * a waking j30d_get_data() spends before it re-commands sleep -- the >10 ms
 * return-to-IIC pulse plus bus traffic -- and only feeds
 * j30d_get_sample_interval_ms(). */
#ifndef J30D_PACING_MARGIN_MS
#define J30D_PACING_MARGIN_MS        10u
#endif
#ifndef J30D_READ_OVERHEAD_MS
#define J30D_READ_OVERHEAD_MS        40u
#endif

/* ---- calibration block parsing ----------------------------------------
 * The block layout and its CRC are fixed by what the factory programmed
 * into the part -- see j30d_calib.h.
 */

static j30d_unit_version_t j30d_detect_unit_version(const uint8_t *data)
{
	uint8_t revision = data[J30D_CAL_OFF_CHIP_REVISION];

	bool all_ff = true;
	for (int i = 0; i < J30D_CUSTOM_MEMORY_SIZE; i++)
	{
		if (data[i] != 0xFFu)
		{
			all_ff = false;
			break;
		}
	}
	if (all_ff)
	{
		return J30D_UNIT_VERSION_CH2;
	}

	/* 0xFF is an unwritten EEPROM byte, not "revision 255": units programmed
	 * before the revision field existed read 0xFF here and have a blank
	 * checksum. They take the timestamp path below (REV1 / CH1_NOTIMESTAMP);
	 * the CRC branch would reject their valid calibration as UNKNOWN. */
	if (revision >= 2 && revision != 0xFFu)
	{
		uint16_t calc_crc   = j30d_crc16(data, J30D_CUSTOM_MEMORY_CRC_LEN);
		uint16_t stored_crc = j30d_be16(&data[J30D_CAL_OFF_CHECKSUM]);
		return (calc_crc == stored_crc) ? J30D_UNIT_VERSION_REV2PLUS : J30D_UNIT_VERSION_UNKNOWN;
	}

	uint32_t timestamp = j30d_be32(&data[J30D_CAL_OFF_PROD_TIMESTAMP]);
	if (timestamp == 0xFFFFFFFFu || timestamp == 0x00000000u)
	{
		return J30D_UNIT_VERSION_UNKNOWN;
	}

	if (revision == 1)
	{
		return J30D_UNIT_VERSION_REV1;
	}

	return J30D_UNIT_VERSION_CH1_NOTIMESTAMP;
}

/* Best-effort: leaves h->calib zeroed (valid == false) on any I2C failure.
 * Never fails j30d_init() -- a unit with no/garbled calibration data is an
 * expected, supported case (see j30d_unit_version_t), not an init error. */
static void j30d_read_calibration(j30d_handle_t *h)
{
	uint8_t data[J30D_CUSTOM_MEMORY_SIZE];

	j30d_err_t err = h->platform.i2c_read(h->platform.ctx, J30D_REG_CUSTOM_MEMORY_START, data, sizeof(data));
	if (err != J30D_OK)
	{
		return;
	}

	j30d_calib_t *c = &h->calib;
	memset(c, 0, sizeof(*c));

	c->prod_timestamp = j30d_be32(&data[J30D_CAL_OFF_PROD_TIMESTAMP]);
	c->uid            = j30d_be32(&data[J30D_CAL_OFF_UNIQUE_ID]);
	c->revision       = data[J30D_CAL_OFF_CHIP_REVISION];
	c->lot_no         = j30d_be16(&data[J30D_CAL_OFF_LOT_NUMBER]);
	c->wafer_no       = j30d_be16(&data[J30D_CAL_OFF_WAFER_NUMBER]);
	c->crc            = j30d_be16(&data[J30D_CAL_OFF_CHECKSUM]);

	c->version = j30d_detect_unit_version(data);

	/* Trim words are S7Q8 on rev>=2 units (the only ones with a verified
	 * CRC) and U8Q8 on older ones. CH2/UNKNOWN carry no trustworthy trim
	 * data and keep the zeroes from the memset above. */
	if (c->version == J30D_UNIT_VERSION_REV2PLUS ||
	    c->version == J30D_UNIT_VERSION_CH1_NOTIMESTAMP ||
	    c->version == J30D_UNIT_VERSION_REV1)
	{
#if defined(J30D_NO_FLOAT)
		/* Hand back the undecoded words; the caller applies the Q format
		 * for this `version` (see j30d_calib.h). */
		c->sensor_offset1_raw = j30d_trim_word(data[J30D_CAL_OFF_SENSOR_OFFSET_1], data[J30D_CAL_OFF_SENSOR_OFFSET_1 + 1]);
		c->sensor_gain1_raw   = j30d_trim_word(data[J30D_CAL_OFF_SENSOR_GAIN_1],   data[J30D_CAL_OFF_SENSOR_GAIN_1 + 1]);
		c->sensor_offset2_raw = j30d_trim_word(data[J30D_CAL_OFF_SENSOR_OFFSET_2], data[J30D_CAL_OFF_SENSOR_OFFSET_2 + 1]);
		c->sensor_gain2_raw   = j30d_trim_word(data[J30D_CAL_OFF_SENSOR_GAIN_2],   data[J30D_CAL_OFF_SENSOR_GAIN_2 + 1]);
#else
		if (c->version == J30D_UNIT_VERSION_REV2PLUS)
		{
			c->sensor_offset1 = j30d_s7q8_to_float(data[J30D_CAL_OFF_SENSOR_OFFSET_1], data[J30D_CAL_OFF_SENSOR_OFFSET_1 + 1]);
			c->sensor_gain1   = j30d_s7q8_to_float(data[J30D_CAL_OFF_SENSOR_GAIN_1],   data[J30D_CAL_OFF_SENSOR_GAIN_1 + 1]);
			c->sensor_offset2 = j30d_s7q8_to_float(data[J30D_CAL_OFF_SENSOR_OFFSET_2], data[J30D_CAL_OFF_SENSOR_OFFSET_2 + 1]);
			c->sensor_gain2   = j30d_s7q8_to_float(data[J30D_CAL_OFF_SENSOR_GAIN_2],   data[J30D_CAL_OFF_SENSOR_GAIN_2 + 1]);
		}
		else
		{
			c->sensor_offset1 = j30d_u8q8_to_float(data[J30D_CAL_OFF_SENSOR_OFFSET_1], data[J30D_CAL_OFF_SENSOR_OFFSET_1 + 1]);
			c->sensor_gain1   = j30d_u8q8_to_float(data[J30D_CAL_OFF_SENSOR_GAIN_1],   data[J30D_CAL_OFF_SENSOR_GAIN_1 + 1]);
			c->sensor_offset2 = j30d_u8q8_to_float(data[J30D_CAL_OFF_SENSOR_OFFSET_2], data[J30D_CAL_OFF_SENSOR_OFFSET_2 + 1]);
			c->sensor_gain2   = j30d_u8q8_to_float(data[J30D_CAL_OFF_SENSOR_GAIN_2],   data[J30D_CAL_OFF_SENSOR_GAIN_2 + 1]);
		}
#endif
	}

	c->valid = true;
}

/* ---- destructive-write guards ------------------------------------------
 * Three registers can leave a unit in a state this driver cannot recover.
 * All are refused with J30D_ERR_PARAM on every write path, each with its
 * own compile-time opt-out.
 *
 *   SYS_CFG1 (0x93) OUT_MODE -- anything but I2C turns SDA into a push-pull
 *     output and stops the I2C interface. It is EEPROM-backed, so the unit
 *     comes back that way after a reset. The guard is one-directional:
 *     writing OUT_MODE back to I2C is always allowed, so a unit already in
 *     that state can be recovered. Other SYS_CFG1 bits are unaffected.
 *
 *   BLOW_START (0x40) -- blows the EEPROM fuse. Irreversible.
 *
 *   CUSTOM_MEMORY (0xC0..0xEF) -- the factory calibration block. The driver
 *     only reads it; see j30d_calib.h.
 */

#if !defined(J30D_ALLOW_NON_I2C_OUTPUT)
static bool sys_cfg1_selects_i2c(uint8_t value)
{
	return (value & J30D_SYS_CFG1_OUT_MODE_Msk) == J30D_SYS_CFG1_OUT_I2C;
}
#endif

/* True if a write of `len` bytes starting at `reg` would land on a register
 * the guards above protect. `buf` is consulted for SYS_CFG1, whose contents
 * decide whether the write is dangerous. */
static bool raw_write_is_blocked(uint8_t reg, const uint8_t *buf, size_t len)
{
#if !defined(J30D_ALLOW_NON_I2C_OUTPUT)
	if (J30D_REG_SYS_CFG1 >= reg && (size_t)(J30D_REG_SYS_CFG1 - reg) < len &&
	    !sys_cfg1_selects_i2c(buf[J30D_REG_SYS_CFG1 - reg]))
	{
		return true;
	}
#endif
#if !defined(J30D_ALLOW_EEPROM_BLOW)
	if (J30D_REG_BLOW_START >= reg && (size_t)(J30D_REG_BLOW_START - reg) < len)
	{
		return true;
	}
#endif
#if !defined(J30D_ALLOW_CALIBRATION_WRITE)
	/* Any overlap with 0xC0..0xEF. */
	if (reg < (uint8_t)(J30D_REG_CUSTOM_MEMORY_START + J30D_CUSTOM_MEMORY_SIZE) &&
	    (size_t)(J30D_REG_CUSTOM_MEMORY_START) < (size_t)reg + len)
	{
		return true;
	}
#endif
	(void)buf; (void)len; (void)reg;
	return false;
}

/* ---- config application ----------------------------------------------
 * Shared by j30d_init() and j30d_update_config(). `reg` is the device
 * register this field maps to -- the mapping lives in apply_config() below
 * and nowhere else, so a caller cannot redirect a config field at an
 * arbitrary register. `desired` is what the caller passed in; `shadow` is
 * h->config's copy of that field; `dflt` is the matching field in
 * j30d_default_config, used when desired->status == J30D_FIELD_DEFAULT.
 */

/* The value a field writes when applied: its own, or the compiled-in default
 * for J30D_FIELD_DEFAULT. Meaningless for J30D_FIELD_UNCHANGED, which writes
 * nothing. */
static uint8_t resolve_field(const j30d_config_field_t *f, const j30d_config_field_t *dflt)
{
	return (f->status == J30D_FIELD_DEFAULT) ? dflt->value : f->value;
}

static j30d_err_t apply_field(j30d_handle_t *h, uint8_t reg, j30d_config_field_t *shadow,
                               const j30d_config_field_t *desired, const j30d_config_field_t *dflt)
{
	if (desired->status == J30D_FIELD_UNCHANGED)
	{
		return J30D_OK;
	}

	uint8_t value = resolve_field(desired, dflt);

#if !defined(J30D_ALLOW_NON_I2C_OUTPUT)
	if (reg == J30D_REG_SYS_CFG1 && !sys_cfg1_selects_i2c(value))
	{
		/* Would switch the device off I2C, permanently (EEPROM) -- see the
		 * destructive-write guards above. Nothing is written. */
		return J30D_ERR_PARAM;
	}
#endif

	j30d_err_t err = h->platform.i2c_write(h->platform.ctx, reg, &value, 1);
	if (err != J30D_OK)
	{
		return err;
	}

	shadow->value  = value;
	shadow->status = J30D_FIELD_UNCHANGED;   /* shadow now matches the device -- nothing pending */
	return J30D_OK;
}

/* CHIP_ADDRESS and the five EEPROM-backed config registers are contiguous,
 * so the shadow is seeded from the device in one burst read. */
#if (J30D_REG_SYS_CFG1    != J30D_REG_CHIP_ADDRESS + 1) || \
    (J30D_REG_SYS_CFG2    != J30D_REG_CHIP_ADDRESS + 2) || \
    (J30D_REG_SYS_CH1_CFG != J30D_REG_CHIP_ADDRESS + 3) || \
    (J30D_REG_SYS_CH2_CFG != J30D_REG_CHIP_ADDRESS + 4) || \
    (J30D_REG_BIAS_CFG    != J30D_REG_CHIP_ADDRESS + 5)
#error "seed_shadow_from_device() needs CHIP_ADDRESS..BIAS_CFG to be contiguous"
#endif
#define J30D_SEED_COUNT   6u

static bool any_seeded_field_unchanged(const j30d_config_t *cfg)
{
	return cfg->chip_address.status == J30D_FIELD_UNCHANGED
	    || cfg->sys_cfg1.status     == J30D_FIELD_UNCHANGED
	    || cfg->sys_cfg2.status     == J30D_FIELD_UNCHANGED
	    || cfg->ch1_cfg.status      == J30D_FIELD_UNCHANGED
	    || cfg->ch2_cfg.status      == J30D_FIELD_UNCHANGED
	    || cfg->bias_cfg.status     == J30D_FIELD_UNCHANGED;
}

/* Loads the shadow with what the device holds, so fields that j30d_init()
 * leaves J30D_FIELD_UNCHANGED are not reported as the compiled-in defaults.
 * decode_gain() scales raw_ch1/raw_ch2 from ch1_cfg/ch2_cfg here, so a
 * stale shadow means wrong millivolts. */
static j30d_err_t seed_shadow_from_device(j30d_handle_t *h)
{
	uint8_t buf[J30D_SEED_COUNT];

	j30d_err_t err = h->platform.i2c_read(h->platform.ctx, J30D_REG_CHIP_ADDRESS, buf, sizeof(buf));
	if (err != J30D_OK)
	{
		return err;
	}

	h->config.chip_address.value = buf[0];
	h->config.sys_cfg1.value     = buf[1];
	h->config.sys_cfg2.value     = buf[2];
	h->config.ch1_cfg.value      = buf[3];
	h->config.ch2_cfg.value      = buf[4];
	h->config.bias_cfg.value     = buf[5];
	return J30D_OK;
}

/* ---- interrupt mode ------------------------------------------------------
 * The device has no data-ready interrupt, but its sleep-mode comparator
 * raises INT after any conversion that meets the threshold. The most
 * negative threshold the register can express (-128mV) is at or below the
 * bottom of the range under either VT_SCALE setting, so every sample
 * qualifies and INT means "new sample".
 *
 * Written at each init: the registers are EEPROM-backed and this driver
 * does not program EEPROM. */
static j30d_err_t apply_interrupt_mode(j30d_handle_t *h)
{
	/* Verified, not written: some SYS_CFG2 bits take effect only at EEPROM
	 * load, so a write here would not reliably apply. */
	uint8_t sys_cfg2 = 0;
	j30d_err_t err = h->platform.i2c_read(h->platform.ctx, J30D_REG_SYS_CFG2, &sys_cfg2, 1);
	if (err != J30D_OK)
	{
		return err;
	}
	if (sys_cfg2 & J30D_SYS_CFG2_SLEEP_POL_Msk)
	{
		/* "Trigger when lower" can never be met by the minimum threshold. */
		return J30D_ERR_PARAM;
	}

	uint8_t threshold[2] = { J30D_INT_THRESHOLD_MSB, J30D_INT_THRESHOLD_LSB };
	return h->platform.i2c_write(h->platform.ctx, J30D_REG_T_THRESHOLD_SLEEP_MSB, threshold, 2);
}

/* What interrupt mode needs to deliver anything: release_to_int to hand SDA
 * to the device, SLEEP_EN because INT is the sleep-mode comparator output,
 * and now_ms because only the pacing rejects the INT the device raises one
 * conversion after every sleep command. Without the clock each of those would
 * be answered with a read that re-commands sleep and retriggers it, and the
 * device would never complete a sleep interval. */
static bool interrupt_mode_supported(const j30d_platform_t *platform, uint8_t command)
{
	return platform->release_to_int != NULL &&
	       platform->now_ms != NULL &&
	       (command & J30D_COMMAND_SLEEP_EN) != 0;
}

/* COMMAND is written last. The device stops acknowledging I2C the moment
 * SLEEP_EN lands, so any register written after it would go to a sleeping
 * device and fail. */
static j30d_err_t apply_config(j30d_handle_t *h, const j30d_config_t *cfg)
{
	j30d_err_t err;

	/* Address first. Writing CHIP_ADDRESS moves the device immediately, so
	 * everything after it is addressed to a part that has already moved --
	 * which works because the port talks to the 0x7F wildcard, and that
	 * answers whatever address the device holds. See j30d_config.h. */
	if ((err = apply_field(h, J30D_REG_CHIP_ADDRESS, &h->config.chip_address, &cfg->chip_address, &j30d_default_config.chip_address)) != J30D_OK) return err;

	if ((err = apply_field(h, J30D_REG_SYS_CFG1,    &h->config.sys_cfg1, &cfg->sys_cfg1, &j30d_default_config.sys_cfg1)) != J30D_OK) return err;
	if ((err = apply_field(h, J30D_REG_SYS_CFG2,    &h->config.sys_cfg2, &cfg->sys_cfg2, &j30d_default_config.sys_cfg2)) != J30D_OK) return err;
	if ((err = apply_field(h, J30D_REG_SYS_CH1_CFG, &h->config.ch1_cfg,  &cfg->ch1_cfg,  &j30d_default_config.ch1_cfg))  != J30D_OK) return err;
	if ((err = apply_field(h, J30D_REG_SYS_CH2_CFG, &h->config.ch2_cfg,  &cfg->ch2_cfg,  &j30d_default_config.ch2_cfg))  != J30D_OK) return err;
	if ((err = apply_field(h, J30D_REG_SLEEP_TIME,  &h->config.sleep,    &cfg->sleep,    &j30d_default_config.sleep))    != J30D_OK) return err;
	if ((err = apply_field(h, J30D_REG_BIAS_CFG,    &h->config.bias_cfg, &cfg->bias_cfg, &j30d_default_config.bias_cfg)) != J30D_OK) return err;
	/* The mode itself is set by j30d_init(); this re-asserts the threshold. */
	if (h->config.interrupt_enable)
	{
		if ((err = apply_interrupt_mode(h)) != J30D_OK) return err;
	}

	if ((err = apply_field(h, J30D_REG_COMMAND,     &h->config.command,  &cfg->command,  &j30d_default_config.command))  != J30D_OK) return err;

	/* Every field now matches the device or was deliberately left alone, so
	 * nothing is pending. A status left at J30D_FIELD_DEFAULT would rewrite
	 * that register from the defaults when the caller passes h->config back
	 * to j30d_update_config(). */
	h->config.chip_address.status = J30D_FIELD_UNCHANGED;
	h->config.command.status      = J30D_FIELD_UNCHANGED;
	h->config.sleep.status        = J30D_FIELD_UNCHANGED;
	h->config.sys_cfg1.status     = J30D_FIELD_UNCHANGED;
	h->config.sys_cfg2.status     = J30D_FIELD_UNCHANGED;
	h->config.ch1_cfg.status      = J30D_FIELD_UNCHANGED;
	h->config.ch2_cfg.status      = J30D_FIELD_UNCHANGED;
	h->config.bias_cfg.status     = J30D_FIELD_UNCHANGED;

	return J30D_OK;
}

/* ---- sleep handling -----------------------------------------------------
 * Sleep is configured iff SLEEP_EN is set in the shadow COMMAND value. Every
 * call that touches the bus wakes the device first and re-commands sleep
 * afterwards, which costs a full return_to_iic() per call even when the
 * device was already awake. Only j30d_get_data() is paced, and only when
 * platform.now_ms is set.
 */

static bool sleep_is_configured(const j30d_handle_t *h)
{
	return (h->config.command.value & J30D_COMMAND_SLEEP_EN) != 0;
}

/* A wake clears SLEEP_EN but leaves MODE_EN set, so the device comes back
 * converting and a burst read can straddle a conversion boundary. Stopping
 * the engine first freezes the data registers, so preemption between the
 * wake and the read cannot tear a sample.
 *
 * SLEEP_EN must be cleared in the same write: the shadow still carries it,
 * and writing it back would put the device straight back to sleep.
 * restore_sleep_if_needed() re-arms both bits afterwards.
 *
 * Sleeping configurations only -- a continuously converting device is doing
 * what the caller asked for. */
static j30d_err_t halt_conversions(j30d_handle_t *h)
{
	uint8_t value = (uint8_t)(h->config.command.value &
	                          ~(J30D_COMMAND_SLEEP_EN | J30D_COMMAND_MODE_EN));
	return h->platform.i2c_write(h->platform.ctx, J30D_REG_COMMAND, &value, 1);
}

static j30d_err_t ensure_awake(j30d_handle_t *h)
{
	if (!sleep_is_configured(h))
	{
		return J30D_OK;
	}

	h->platform.return_to_iic(h->platform.ctx);
	return halt_conversions(h);
}

/* Hand SDA to the device so it can drive INT. Only in interrupt mode, only
 * with sleep configured, and only after the sleep command has been written
 * -- releasing first would leave that write with no bus. Called from both
 * places the device ends up asleep; idempotent. */
static void release_bus_to_int(j30d_handle_t *h)
{
	if (h->config.interrupt_enable &&
	    sleep_is_configured(h) &&
	    h->platform.release_to_int != NULL)
	{
		h->platform.release_to_int(h->platform.ctx);
	}
}

/* Records when sleep was commanded; the sample-timing section below counts
 * from here. Every path that ends in a sleep command calls it once the write
 * has succeeded. */
static void note_sleep_command(j30d_handle_t *h)
{
	if (h->platform.now_ms != NULL)
	{
		h->internal.sleep_command_ms = h->platform.now_ms(h->platform.ctx);
		h->internal.sleep_commanded  = true;
	}
}

/* Best-effort: re-commands the shadow COMMAND value, SLEEP_EN included, and
 * does not report failure. The call it wraps has already succeeded or failed
 * on its own; a device left awake is put back to sleep by the next call. */
static void restore_sleep_if_needed(j30d_handle_t *h)
{
	if (sleep_is_configured(h))
	{
		uint8_t value = h->config.command.value;
		if (h->platform.i2c_write(h->platform.ctx, J30D_REG_COMMAND, &value, 1) == J30D_OK)
		{
			note_sleep_command(h);
		}

		release_bus_to_int(h);
	}
}

/* ---- sample timing -------------------------------------------------------
 * A sleep command starts the device's cycle over: it converts once, sleeps
 * for SLEEP_TIME, converts, sleeps, and so on. Waking it halts whatever
 * conversion is under way, and every wake ends in another sleep command, so
 * a host that wakes it before the conversion ending the first sleep interval
 * is complete restarts the cycle without ever seeing that sample -- and does
 * so again on every call if it polls at the same rate.
 *
 * j30d_get_data() therefore counts from the last sleep command, whichever
 * call issued it, and does not wake the device before that conversion is
 * complete on any part within tolerance. SLEEP_TIME runs on an untrimmed
 * 32 kHz oscillator, +/-15%; the same allowance is applied to the conversion
 * time, which errs long.
 */

#define J30D_OSC_TOLERANCE_PCT   15u

/* SLEEP_TIME register -> milliseconds: register value 0x00 == 100ms, then
 * +100ms per step -- see the comment on J30D_REG_SLEEP_TIME in
 * j30d_regs.h. */
static uint32_t sleep_interval_ms(const j30d_handle_t *h)
{
	return ((uint32_t)h->config.sleep.value + 1u) * 100u;
}

/* OSR field to the oversampling ratio it selects. SYS_CFG1 and SYS_CHx_CFG
 * use the same 3-bit encoding. */
static uint32_t osr_ratio(uint8_t field)
{
	static const uint16_t ratio[8] = { 512u, 1024u, 2048u, 4096u, 128u, 256u, 8192u, 16384u };
	return ratio[field & 7u];
}

/* One conversion, nominal, in ms rounded up:
 *
 *     (OSRP + 2 x OSRT + 1481) / Fosc_kHz
 *
 * measured on the bench in single mode, where channel 2 does not convert, so
 * OSRP is channel 1's ratio. The dual modes are unmeasured; the driver adds
 * both channels' ratios there, which errs long if they do not add. */
static uint32_t conversion_ms(const j30d_handle_t *h)
{
	const uint8_t cmd  = h->config.command.value;
	const uint32_t osrt = osr_ratio((uint8_t)((h->config.sys_cfg1.value & J30D_SYS_CFG1_OSR_AMB_Msk) >> J30D_SYS_CFG1_OSRT_AMB_Pos));
	uint32_t       osrp = osr_ratio((uint8_t)((h->config.ch1_cfg.value & J30D_SYS_CHx_CFG_OSRP_Msk) >> J30D_SYS_CHx_CFG_OSRP_Pos));

	if ((cmd & J30D_COMMAND_MODE_SEL_Msk) != J30D_COMMAND_MODE_SEL_SINGLE)
	{
		osrp += osr_ratio((uint8_t)((h->config.ch2_cfg.value & J30D_SYS_CHx_CFG_OSRP_Msk) >> J30D_SYS_CHx_CFG_OSRP_Pos));
	}

	const uint32_t fosc_khz = (cmd & J30D_COMMAND_CLK_1200KHZ) ? 1200u : 600u;
	return (osrp + 2u * osrt + 1481u + fosc_khz - 1u) / fosc_khz;
}

static uint32_t longest_ms(uint32_t nominal_ms)
{
	return (nominal_ms * (100u + J30D_OSC_TOLERANCE_PCT) + 99u) / 100u;
}

static uint32_t shortest_ms(uint32_t nominal_ms)
{
	return nominal_ms * (100u - J30D_OSC_TOLERANCE_PCT) / 100u;
}

/* Polling: from the sleep command until the conversion ending the first
 * sleep interval is certainly complete -- the conversion on entry, the sleep
 * interval and that conversion, all at their longest -- plus the resolution
 * of now_ms. */
static uint32_t polling_wait_ms(const j30d_handle_t *h)
{
	return longest_ms(sleep_interval_ms(h) + 2u * conversion_ms(h)) + J30D_PACING_MARGIN_MS;
}

/* Interrupt mode: an INT marks the sample, so with one pending the pacing
 * only has to reject the INT raised by the conversion on entry to sleep.
 * That conversion at its longest plus half the shortest sleep interval sits
 * between the two at every SLEEP_TIME setting: at 100 ms it ends 42 ms after
 * the latest the on-entry INT can arrive, and at least 43 ms before the
 * earliest genuine one. */
static uint32_t interrupt_wait_ms(const j30d_handle_t *h)
{
	return longest_ms(conversion_ms(h)) + shortest_ms(sleep_interval_ms(h)) / 2u;
}

/* True when j30d_get_data() should skip the bus and hand back the last data
 * as stale. The polling wait always suffices. The shorter interrupt-mode
 * wait applies only with an INT pending: without one, nothing says the
 * sample is complete, and reading at that point could return the conversion
 * the device did on entry to sleep. j30d_raw_read()/j30d_raw_write() are
 * never paced, but their sleep commands restart the count like any other. */
static bool should_skip_get_data_wake(const j30d_handle_t *h, uint32_t now, bool int_pending)
{
	if (!sleep_is_configured(h) || !h->internal.sleep_commanded)
	{
		return false;
	}
	uint32_t elapsed = now - h->internal.sleep_command_ms;   /* wraparound-safe: unsigned subtraction */
	if (elapsed >= polling_wait_ms(h))
	{
		return false;
	}
	return !(h->config.interrupt_enable && int_pending && elapsed >= interrupt_wait_ms(h));
}

/* ---- stopped-conversion check --------------------------------------------
 * SCL held low for about 10 ms is the device's return-to-IIC signal, and on
 * a device that is converting it clears COMMAND: conversions stop and the
 * data registers keep their last values. A host transfer that stalls that
 * long does it by accident (see j30d_platform.h). In sleep mode every call
 * ends in a sleep command, which writes COMMAND again; converting
 * continuously, nothing does, and the device would stay stopped.
 *
 * So when j30d_get_data() finds no new data on a continuously converting
 * device, and none has come for longer than two conversions at their
 * longest, it reads COMMAND back. If MODE_EN is gone it writes COMMAND from
 * the shadow and reports J30D_ERR_RESTARTED. Without platform.now_ms there
 * is nothing to time by, so it reads COMMAND on every call that finds no
 * new data.
 */

static bool converting_continuously(const j30d_handle_t *h)
{
	return !sleep_is_configured(h) && (h->config.command.value & J30D_COMMAND_MODE_EN) != 0;
}

static uint32_t stopped_check_wait_ms(const j30d_handle_t *h)
{
	return 2u * longest_ms(conversion_ms(h)) + J30D_PACING_MARGIN_MS;
}

/* New data arrived: conversions are running. */
static void note_converting(j30d_handle_t *h)
{
	if (h->platform.now_ms != NULL)
	{
		h->internal.converting_ms   = h->platform.now_ms(h->platform.ctx);
		h->internal.converting_seen = true;
	}
}

/* Called when the status read found nothing new. J30D_OK if the device is
 * converting or it is too soon to tell, J30D_ERR_RESTARTED if COMMAND had to
 * be written again. */
static j30d_err_t restart_if_stopped(j30d_handle_t *h)
{
	if (h->platform.now_ms != NULL)
	{
		const uint32_t now = h->platform.now_ms(h->platform.ctx);
		if (!h->internal.converting_seen)
		{
			/* Nothing to measure from yet: start the count here. */
			h->internal.converting_ms   = now;
			h->internal.converting_seen = true;
			return J30D_OK;
		}
		if (now - h->internal.converting_ms <= stopped_check_wait_ms(h))   /* wraparound-safe */
		{
			return J30D_OK;
		}
		/* Checked now; the next check is one wait from here. */
		h->internal.converting_ms = now;
	}

	uint8_t command = 0;
	j30d_err_t err = h->platform.i2c_read(h->platform.ctx, J30D_REG_COMMAND, &command, 1);
	if (err != J30D_OK)
	{
		return err;
	}
	if ((command & J30D_COMMAND_MODE_EN) != 0)
	{
		return J30D_OK;
	}
	err = h->platform.i2c_write(h->platform.ctx, J30D_REG_COMMAND, &h->config.command.value, 1);
	return (err != J30D_OK) ? err : J30D_ERR_RESTARTED;
}

/* ---- raw-write shadow sync ---------------------------------------------
 * j30d_raw_write() bypasses the config model, but the driver still reads
 * h->config for COMMAND, SLEEP_TIME and the channel configs, and
 * j30d_get_config() reports all of it. A raw write to a register the shadow
 * mirrors must be reflected in it, or the driver acts on a stale copy --
 * most visibly by restoring sleep to the old COMMAND value on the way out
 * of the same call, undoing the write.
 *
 * Addresses auto-increment, so a write of `len` bytes covers
 * reg .. reg+len-1.
 *
 * A raw write to SOFT_RESET is deliberately not covered: it reloads EEPROM
 * behind the driver and invalidates the whole shadow. Call j30d_init()
 * again instead. */
static void sync_shadow_after_raw_write(j30d_handle_t *h, uint8_t reg, const uint8_t *buf, size_t len)
{
	const struct
	{
		uint8_t              reg;
		j30d_config_field_t *field;
	} map[] = {
		{ J30D_REG_COMMAND,      &h->config.command      },
		{ J30D_REG_SLEEP_TIME,   &h->config.sleep        },
		{ J30D_REG_CHIP_ADDRESS, &h->config.chip_address },
		{ J30D_REG_SYS_CFG1,     &h->config.sys_cfg1     },
		{ J30D_REG_SYS_CFG2,     &h->config.sys_cfg2     },
		{ J30D_REG_SYS_CH1_CFG,  &h->config.ch1_cfg      },
		{ J30D_REG_SYS_CH2_CFG,  &h->config.ch2_cfg      },
		{ J30D_REG_BIAS_CFG,     &h->config.bias_cfg     },
	};

	for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++)
	{
		/* Subtract only after the >= test, so this can't wrap. */
		if (map[i].reg >= reg && (size_t)(map[i].reg - reg) < len)
		{
			map[i].field->value = buf[map[i].reg - reg];
		}
	}
}

/* ---- handle locking -----------------------------------------------------
 * lock/unlock are optional. A lock that reports failure aborts the call with
 * J30D_ERR_TIMEOUT before any bus access, and unlock is then not called. */

static j30d_err_t handle_lock(j30d_handle_t *h)
{
	if (h->platform.lock && !h->platform.lock(h->platform.ctx))
	{
		return J30D_ERR_TIMEOUT;
	}
	return J30D_OK;
}

static void handle_unlock(j30d_handle_t *h)
{
	if (h->platform.unlock)
	{
		h->platform.unlock(h->platform.ctx);
	}
}

/* ---- public API -------------------------------------------------------- */

j30d_err_t j30d_init(j30d_handle_t *h, const j30d_platform_t *platform, const j30d_config_t *config)
{
	if (h == NULL || platform == NULL)
	{
		return J30D_ERR_PARAM;
	}
	if (!platform->i2c_read || !platform->i2c_write || !platform->delay_ms || !platform->return_to_iic)
	{
		return J30D_ERR_PARAM;
	}

	/* Validated as it will be applied: NULL means j30d_default_config, and a
	 * J30D_FIELD_DEFAULT field means its value from there. */
	const j30d_config_t *want = (config != NULL) ? config : &j30d_default_config;

	/* COMMAND and SLEEP_TIME are RAM registers that the soft reset below
	 * returns to their reset values, so J30D_FIELD_UNCHANGED has nothing left
	 * to preserve for them. The shadow would claim a COMMAND the device does
	 * not hold, and the first re-arm would write it out. */
	if (want->command.status == J30D_FIELD_UNCHANGED || want->sleep.status == J30D_FIELD_UNCHANGED)
	{
		return J30D_ERR_PARAM;
	}
	if (want->interrupt_enable &&
	    !interrupt_mode_supported(platform, resolve_field(&want->command, &j30d_default_config.command)))
	{
		return J30D_ERR_PARAM;
	}

	/* Locked before anything in `h` is touched: on a re-init another thread
	 * may be mid-call on this handle. */
	if (platform->lock && !platform->lock(platform->ctx))
	{
		return J30D_ERR_TIMEOUT;   /* nothing acquired, handle untouched */
	}

	memset(h, 0, sizeof(*h));
	h->platform = *platform;

	/* Return-to-IIC-idle/wake sequence -- see j30d_platform.h. Must run
	 * before the soft reset since the device may be asleep, or left in
	 * PWM/Relay output mode (IIC unusable) by a previous run. */
	h->platform.return_to_iic(h->platform.ctx);

	uint8_t reset_val = J30D_SOFT_RESET_Msk;
	j30d_err_t err = h->platform.i2c_write(h->platform.ctx, J30D_REG_SOFT_RESET, &reset_val, 1);
	if (err != J30D_OK)
	{
		goto done;
	}

	h->platform.delay_ms(h->platform.ctx, J30D_RESET_SETTLE_DELAY_MS);

	bool eeprom_loaded = false;
	for (uint32_t attempt = 0; attempt < J30D_EEPROM_POLL_ATTEMPTS; ++attempt)
	{
		uint8_t error_code = 0;
		err = h->platform.i2c_read(h->platform.ctx, J30D_REG_ERROR_CODE, &error_code, 1);
		if (err != J30D_OK)
		{
			goto done;
		}
		if (error_code & J30D_ERROR_CODE_EEPROM_LOADED_Msk)
		{
			eeprom_loaded = true;
			break;
		}
		if (attempt + 1u < J30D_EEPROM_POLL_ATTEMPTS)
		{
			h->platform.delay_ms(h->platform.ctx, J30D_EEPROM_POLL_DELAY_MS);   /* no point sleeping after the last look */
		}
	}
	if (!eeprom_loaded)
	{
		err = J30D_ERR_EEPROM_NOT_LOADED;
		goto done;
	}

	/* Best-effort: never fails init, see j30d_read_calibration(). */
	j30d_read_calibration(h);

	/* The shadow starts as the defaults. Fields left J30D_FIELD_UNCHANGED are
	 * not written, so they are read back from the device instead; the rest
	 * take what apply_config() writes. */
	h->config = j30d_default_config;
	h->config.interrupt_enable = want->interrupt_enable;

	if (any_seeded_field_unchanged(want))
	{
		err = seed_shadow_from_device(h);
		if (err != J30D_OK)
		{
			goto done;
		}
	}

	err = apply_config(h, want);
	if (err != J30D_OK)
	{
		goto done;
	}

	h->initialized = true;
	err = J30D_OK;

	/* apply_config() wrote COMMAND last; with SLEEP_EN set, that put the
	 * device to sleep. */
	if (sleep_is_configured(h))
	{
		note_sleep_command(h);
	}
	release_bus_to_int(h);

done:
	handle_unlock(h);
	return err;
}

j30d_err_t j30d_update_config(j30d_handle_t *h, const j30d_config_t *config)
{
	if (h == NULL || config == NULL)
	{
		return J30D_ERR_PARAM;
	}
	if (!h->initialized)
	{
		return J30D_ERR_NOT_INIT;
	}

	{
		j30d_err_t lock_err = handle_lock(h);
		if (lock_err != J30D_OK)
		{
			return lock_err;
		}
	}

	/* Wake against the current config, before this call changes it. */
	j30d_err_t err = ensure_awake(h);
	if (err == J30D_OK)
	{
		err = apply_config(h, config);
	}

	if (err == J30D_OK && config->command.status != J30D_FIELD_UNCHANGED)
	{
		/* apply_config() wrote COMMAND last, so the device is already in its
		 * new state. Commanding it again would address a sleeping device. */
		if (sleep_is_configured(h))
		{
			note_sleep_command(h);
		}
		release_bus_to_int(h);
	}
	else
	{
		restore_sleep_if_needed(h);
	}

	handle_unlock(h);
	return err;
}

j30d_err_t j30d_get_data(j30d_handle_t *h, j30d_data_t *out)
{
	if (h == NULL || out == NULL)
	{
		return J30D_ERR_PARAM;
	}
	if (!h->initialized)
	{
		return J30D_ERR_NOT_INIT;
	}

	{
		j30d_err_t lock_err = handle_lock(h);
		if (lock_err != J30D_OK)
		{
			return lock_err;
		}
	}

	/* Consumed by every call. The device raises INT one conversion after
	 * every sleep command as well as at the end of each sleep interval; a
	 * call that comes in the first of those windows takes the on-entry INT
	 * off the flag, so it is not mistaken for the sample later. */
	const bool int_pending = h->internal.interrupt_pending;
	h->internal.interrupt_pending = false;

	if (h->platform.now_ms != NULL &&
	    should_skip_get_data_wake(h, h->platform.now_ms(h->platform.ctx), int_pending))
	{
		/* Too soon for a new sample: the last data, all stale, without
		 * touching the bus. */
		*out = h->internal.last_data;
		out->ch1.state = out->ch2.state = out->amb.state = J30D_STALE;
		out->raw_ch1.state = out->raw_ch2.state = out->raw_amb.state = J30D_STALE;

		handle_unlock(h);
		return J30D_OK;
	}

	{
		j30d_err_t wake_err = ensure_awake(h);
		if (wake_err != J30D_OK)
		{
			/* The conversion engine could not be stopped, so anything read
			 * now could tear across a conversion. Leave `out` untouched
			 * entirely, per the get_data contract in j30d.h. */
			restore_sleep_if_needed(h);
			handle_unlock(h);
			return wake_err;
		}
	}

	uint8_t status[3];   /* DATA_READY, DATA_READY_RAW, ERROR_CODE -- one burst read */
	j30d_err_t err = h->platform.i2c_read(h->platform.ctx, J30D_REG_DATA_READY, status, sizeof(status));
	if (err != J30D_OK)
	{
		/* Don't know anything new -- leave `out` untouched entirely, per
		 * the get_data contract in j30d.h. */
		restore_sleep_if_needed(h);
		handle_unlock(h);
		return err;
	}

	uint8_t data_ready     = status[0];
	uint8_t data_ready_raw = status[1];
	/* status[2] (ERROR_CODE) only matters during the init handshake -- ignored here. */

	/* Seed with last known-good values, mark everything stale by default;
	 * only fields actually attempted below get overwritten. */
	*out = h->internal.last_data;
	out->ch1.state = out->ch2.state = out->amb.state = J30D_STALE;
	out->raw_ch1.state = out->raw_ch2.state = out->raw_amb.state = J30D_STALE;

	j30d_err_t worst = J30D_OK;

	if (converting_continuously(h))
	{
		const bool any_new =
			(data_ready & (J30D_TA_DRDY_Msk | J30D_TO1_DRDY_Msk | J30D_TO2_DRDY_Msk)) != 0 ||
			(data_ready_raw & (J30D_TA_RAW_DRDY_Msk | J30D_TO1_RAW_DRDY_Msk | J30D_TO2_RAW_DRDY_Msk)) != 0;
		if (any_new)
		{
			note_converting(h);
		}
		else
		{
			worst = restart_if_stopped(h);
		}
	}

	/* The status read above cleared every DRDY bit, so each group flagged
	 * ready is read now or its sample is lost. */
	if (data_ready & J30D_TA_DRDY_Msk)
	{
		uint8_t buf[3];
		j30d_err_t e = h->platform.i2c_read(h->platform.ctx, J30D_REG_AMB_MSB, buf, sizeof(buf));
		if (e == J30D_OK)
		{
			out->amb.raw_code = sign_extend24(buf);
#if !defined(J30D_NO_FLOAT)
			out->amb.value    = (float)out->amb.raw_code / J30D_AMB_SCALE;
#endif
			out->amb.state    = J30D_FRESH;
		}
		else
		{
			out->amb.state = J30D_ERROR;
			worst = e;
		}
	}

	/* ch1 and ch2 share one burst but have separate DRDY bits (TO1/TO2):
	 * read if either is set, and mark fresh only the channels flagged. The
	 * other channel's registers hold whatever was last converted there. */
	if (data_ready & (J30D_TO1_DRDY_Msk | J30D_TO2_DRDY_Msk))
	{
		uint8_t buf[6];   /* ch1 (3) + ch2 (3) */
		j30d_err_t e = h->platform.i2c_read(h->platform.ctx, J30D_REG_CAL_MSB, buf, sizeof(buf));
		if (e == J30D_OK)
		{
			if (data_ready & J30D_TO1_DRDY_Msk)
			{
				out->ch1.raw_code = sign_extend24(&buf[0]);
#if !defined(J30D_NO_FLOAT)
				out->ch1.value    = (float)out->ch1.raw_code / vtp_scale(h->config.sys_cfg2.value);
#endif
				out->ch1.state    = J30D_FRESH;
			}
			if (data_ready & J30D_TO2_DRDY_Msk)
			{
				out->ch2.raw_code = sign_extend24(&buf[3]);
#if !defined(J30D_NO_FLOAT)
				out->ch2.value    = (float)out->ch2.raw_code / vtp_scale(h->config.sys_cfg2.value);
#endif
				out->ch2.state    = J30D_FRESH;
			}
		}
		else
		{
			/* Only the channels that HAD data pending lost anything. */
			if (data_ready & J30D_TO1_DRDY_Msk)
			{
				out->ch1.state = J30D_ERROR;
			}
			if (data_ready & J30D_TO2_DRDY_Msk)
			{
				out->ch2.state = J30D_ERROR;
			}
			worst = e;
		}
	}

	/* Same rule for the raw group: one burst, three DRDY bits
	 * (TO1_RAW/TO2_RAW/TA_RAW). */
	if (data_ready_raw & (J30D_TO1_RAW_DRDY_Msk | J30D_TO2_RAW_DRDY_Msk | J30D_TA_RAW_DRDY_Msk))
	{
		uint8_t buf[9];   /* ch1 (3) + ch2 (3) + amb (3) */
		j30d_err_t e = h->platform.i2c_read(h->platform.ctx, J30D_REG_RAW_MSB, buf, sizeof(buf));
		if (e == J30D_OK)
		{
			if (data_ready_raw & J30D_TO1_RAW_DRDY_Msk)
			{
				out->raw_ch1.raw_code = sign_extend24(&buf[0]);
#if !defined(J30D_NO_FLOAT)
				out->raw_ch1.value    = raw_to_mv(out->raw_ch1.raw_code, decode_gain(h->config.ch1_cfg.value));
#endif
				out->raw_ch1.state    = J30D_FRESH;
			}
			if (data_ready_raw & J30D_TO2_RAW_DRDY_Msk)
			{
				out->raw_ch2.raw_code = sign_extend24(&buf[3]);
#if !defined(J30D_NO_FLOAT)
				out->raw_ch2.value    = raw_to_mv(out->raw_ch2.raw_code, decode_gain(h->config.ch2_cfg.value));
#endif
				out->raw_ch2.state    = J30D_FRESH;
			}
			if (data_ready_raw & J30D_TA_RAW_DRDY_Msk)
			{
				out->raw_amb.raw_code = sign_extend24(&buf[6]);
#if !defined(J30D_NO_FLOAT)
				/* The ambient sensor bypasses the analog front end: no
				 * gain, no millivolt equivalent. .value is the unscaled
				 * code. */
				out->raw_amb.value    = (float)out->raw_amb.raw_code;
#endif
				out->raw_amb.state    = J30D_FRESH;
			}
		}
		else
		{
			if (data_ready_raw & J30D_TO1_RAW_DRDY_Msk)
			{
				out->raw_ch1.state = J30D_ERROR;
			}
			if (data_ready_raw & J30D_TO2_RAW_DRDY_Msk)
			{
				out->raw_ch2.state = J30D_ERROR;
			}
			if (data_ready_raw & J30D_TA_RAW_DRDY_Msk)
			{
				out->raw_amb.state = J30D_ERROR;
			}
			worst = e;
		}
	}

	h->internal.last_data      = *out;
	h->internal.status_reg_raw = data_ready;
	h->internal.status_raw_reg_raw = data_ready_raw;

	restore_sleep_if_needed(h);

	handle_unlock(h);
	return worst;
}

/* ---- blocking read -------------------------------------------------------
 * j30d_get_data() called at the right moments, nothing more: every attempt
 * goes through its pacing, so this never wakes the device sooner than
 * j30d_get_data() would, and every sample it returns ends a full sleep
 * interval. It waits in delay_ms() with the handle unlocked.
 */

#define J30D_INT_POLL_MS   1u   /* how often the INT flag is looked at */

/* How long to wait, from `now`, before the next attempt can read a sample. */
static uint32_t next_attempt_ms(const j30d_handle_t *h, uint32_t now)
{
	if (!sleep_is_configured(h))
	{
		/* Converting continuously: look again a quarter-conversion on. */
		uint32_t quarter = conversion_ms(h) / 4u;
		return (quarter > 0u) ? quarter : 1u;
	}
	if (!h->internal.sleep_commanded)
	{
		return J30D_INT_POLL_MS;
	}

	uint32_t elapsed   = now - h->internal.sleep_command_ms;
	uint32_t wait      = polling_wait_ms(h);
	uint32_t remaining = (elapsed < wait) ? (wait - elapsed) : 0u;

	if (remaining == 0u)
	{
		return J30D_INT_POLL_MS;
	}
	/* In interrupt mode the sample can come sooner: watch for its INT. */
	return h->config.interrupt_enable ? J30D_INT_POLL_MS : remaining;
}

j30d_err_t j30d_get_data_blocking(j30d_handle_t *h, j30d_data_t *out, uint32_t timeout_ms)
{
	if (h == NULL || out == NULL)
	{
		return J30D_ERR_PARAM;
	}
	if (!h->initialized)
	{
		return J30D_ERR_NOT_INIT;
	}
	if (h->platform.now_ms == NULL)
	{
		return J30D_ERR_PARAM;   /* nothing to pace or time out by */
	}

	const uint32_t start = h->platform.now_ms(h->platform.ctx);

	for (;;)
	{
		j30d_err_t err = j30d_get_data(h, out);
		if (err != J30D_OK || out->raw_ch1.state == J30D_FRESH)
		{
			return err;
		}

		uint32_t now     = h->platform.now_ms(h->platform.ctx);
		uint32_t elapsed = now - start;
		if (elapsed >= timeout_ms)
		{
			return J30D_ERR_TIMEOUT;
		}

		/* Read under the lock: another thread's sleep command moves it. */
		uint32_t wait;
		err = handle_lock(h);
		if (err != J30D_OK)
		{
			return err;
		}
		wait = next_attempt_ms(h, now);
		handle_unlock(h);

		uint32_t left = timeout_ms - elapsed;
		h->platform.delay_ms(h->platform.ctx, (wait < left) ? wait : left);
	}
}

void j30d_notify_interrupt(j30d_handle_t *h)
{
	if (h == NULL)
	{
		return;   /* a handler racing teardown must not fault */
	}
	/* A single volatile store, and nothing more: no locking and no bus
	 * access from interrupt context. */
	h->internal.interrupt_pending = true;
}

j30d_err_t j30d_raw_read(j30d_handle_t *h, uint8_t reg, uint8_t *buf, size_t len)
{
	if (h == NULL || buf == NULL)
	{
		return J30D_ERR_PARAM;
	}
	if (!h->initialized)
	{
		return J30D_ERR_NOT_INIT;
	}

	{
		j30d_err_t lock_err = handle_lock(h);
		if (lock_err != J30D_OK)
		{
			return lock_err;
		}
	}

	j30d_err_t err = ensure_awake(h);
	if (err == J30D_OK)
	{
		err = h->platform.i2c_read(h->platform.ctx, reg, buf, len);
	}
	restore_sleep_if_needed(h);

	handle_unlock(h);
	return err;
}

j30d_err_t j30d_raw_write(j30d_handle_t *h, uint8_t reg, const uint8_t *buf, size_t len)
{
	if (h == NULL || buf == NULL)
	{
		return J30D_ERR_PARAM;
	}
	if (!h->initialized)
	{
		return J30D_ERR_NOT_INIT;
	}

	if (raw_write_is_blocked(reg, buf, len))
	{
		/* Refused before the lock and before any bus access -- see the
		 * destructive-write guards above for what each one protects and
		 * how to opt out at build time. */
		return J30D_ERR_PARAM;
	}

	{
		j30d_err_t lock_err = handle_lock(h);
		if (lock_err != J30D_OK)
		{
			return lock_err;
		}
	}

	j30d_err_t err = ensure_awake(h);
	if (err == J30D_OK)
	{
		err = h->platform.i2c_write(h->platform.ctx, reg, buf, len);
	}
	if (err == J30D_OK)
	{
		/* Before restore_sleep_if_needed(), so that step re-commits the
		 * value just written rather than the one it replaced. */
		sync_shadow_after_raw_write(h, reg, buf, len);
	}
	restore_sleep_if_needed(h);

	handle_unlock(h);
	return err;
}

/* ---- read-only accessors ------------------------------------------------
 * No bus access, no locking, no blocking -- see j30d.h. */

const j30d_calib_t *j30d_get_calib(const j30d_handle_t *h)
{
	if (h == NULL || !h->initialized)
	{
		return NULL;
	}
	return &h->calib;
}

const j30d_config_t *j30d_get_config(const j30d_handle_t *h)
{
	if (h == NULL || !h->initialized)
	{
		return NULL;
	}
	return &h->config;
}

bool j30d_is_initialized(const j30d_handle_t *h)
{
	return (h != NULL) && h->initialized;
}

uint32_t j30d_get_sample_interval_ms(const j30d_handle_t *h)
{
	if (h == NULL || !h->initialized)
	{
		return 0u;
	}
	if (!sleep_is_configured(h))
	{
		return longest_ms(conversion_ms(h));
	}
	return polling_wait_ms(h) + J30D_READ_OVERHEAD_MS;
}

/* ---- heat flux ------------------------------------------------------------
 * Channel 1's factory offset/gain applied to raw_ch1, in mW/cm2. The
 * formula, and what the trim words mean, depend on the unit generation --
 * see j30d_calib.h:
 *
 *   REV1, REV2PLUS    HF = gain1 * raw_ch1 [mV] + offset1
 *   CH1_NOTIMESTAMP   HF = gain1 * 1e-5 * raw_ch1 [code] - offset1
 *
 * CH2 and UNKNOWN units carry no calibration to apply.
 */

static j30d_err_t heat_flux_check(const j30d_handle_t *h, const j30d_field_t *raw_ch1, const void *out)
{
	if (h == NULL || raw_ch1 == NULL || out == NULL)
	{
		return J30D_ERR_PARAM;
	}
	if (!h->initialized)
	{
		return J30D_ERR_NOT_INIT;
	}
	const j30d_unit_version_t v = h->calib.version;
	if (!h->calib.valid ||
	    (v != J30D_UNIT_VERSION_REV1 && v != J30D_UNIT_VERSION_REV2PLUS && v != J30D_UNIT_VERSION_CH1_NOTIMESTAMP))
	{
		return J30D_ERR_NO_CALIB;
	}
	return J30D_OK;
}

#if !defined(J30D_NO_FLOAT)
j30d_err_t j30d_heat_flux(const j30d_handle_t *h, const j30d_field_t *raw_ch1, float *mw_cm2)
{
	j30d_err_t err = heat_flux_check(h, raw_ch1, mw_cm2);
	if (err != J30D_OK)
	{
		return err;
	}

	const j30d_calib_t *c = &h->calib;
	if (c->version == J30D_UNIT_VERSION_CH1_NOTIMESTAMP)
	{
		*mw_cm2 = c->sensor_gain1 * 1e-5f * (float)raw_ch1->raw_code - c->sensor_offset1;
	}
	else
	{
		*mw_cm2 = c->sensor_gain1 * raw_ch1->value + c->sensor_offset1;
	}
	return J30D_OK;
}
#endif

#if defined(J30D_NO_FLOAT)
/* num / den rounded to nearest, den > 0. */
static int32_t div_round(int64_t num, int64_t den)
{
	return (int32_t)(((num >= 0) ? (num + den / 2) : (num - den / 2)) / den);
}
#endif

j30d_err_t j30d_heat_flux_x1000(const j30d_handle_t *h, const j30d_field_t *raw_ch1, int32_t *mw_cm2_x1000)
{
	j30d_err_t err = heat_flux_check(h, raw_ch1, mw_cm2_x1000);
	if (err != J30D_OK)
	{
		return err;
	}

#if defined(J30D_NO_FLOAT)
	/* The trim words are gain1 and offset1 times 256: S7Q8 (signed) on
	 * REV2PLUS units, U8Q8 on the older ones. Each term is computed and
	 * rounded on its own; the products stay well inside int64_t. */
	const j30d_calib_t *c    = &h->calib;
	const int64_t       code = raw_ch1->raw_code;
	const bool          sgn  = (c->version == J30D_UNIT_VERSION_REV2PLUS);
	const int64_t       g    = sgn ? (int64_t)(int16_t)c->sensor_gain1_raw   : (int64_t)c->sensor_gain1_raw;
	const int64_t       o    = sgn ? (int64_t)(int16_t)c->sensor_offset1_raw : (int64_t)c->sensor_offset1_raw;

	if (c->version == J30D_UNIT_VERSION_CH1_NOTIMESTAMP)
	{
		/* 1000 * (g/256) * 1e-5 * code  =  g * code / 25600 */
		*mw_cm2_x1000 = div_round(g * code, 25600) - div_round(o * 1000, 256);
	}
	else
	{
		/* raw_ch1 in mV is 1000 * code * 1.1 / (pga * 2^23), so
		 * 1000 * (g/256) * mV  =  g * code * 1100000 / (pga * 2^31) */
		const int64_t pga = (int64_t)pga_gain(h->config.ch1_cfg.value);
		*mw_cm2_x1000 = div_round(g * code * 1100000, pga << 31) + div_round(o * 1000, 256);
	}
#else
	float hf;
	(void)j30d_heat_flux(h, raw_ch1, &hf);
	hf *= 1000.0f;
	*mw_cm2_x1000 = (int32_t)((hf >= 0.0f) ? (hf + 0.5f) : (hf - 0.5f));
#endif
	return J30D_OK;
}
