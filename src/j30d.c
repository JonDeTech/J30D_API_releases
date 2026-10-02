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
 * Heat-flux conversion from the calibrated values is application math and
 * lives in the integrator's code, not here. Reading the factory calibration
 * block is done here, since it is a register read the driver needs anyway --
 * see j30d_calib.h.
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

/* ---- calibration block parsing ----------------------------------------
 * The block layout and its CRC are fixed by what the factory programmed
 * into the part, so none of it is ours to choose -- see j30d_calib.h.
 * Kept as free functions rather than static-local to j30d_init(), so they
 * can be unit-tested in isolation against known-good byte sequences.
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

	/* 0xFF is an unwritten EEPROM byte, not "revision 255". Units programmed
	 * before the revision field existed read 0xFF here, and their checksum
	 * field is blank too -- so treating them as rev2+ sends them into the CRC
	 * branch below, where they fail and come back UNKNOWN, and the caller
	 * silently drops calibration that is perfectly good. They belong in the
	 * timestamp path further down (REV1 / CH1_NOTIMESTAMP). */
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
	c->revision        = data[J30D_CAL_OFF_CHIP_REVISION];
	c->lot_no          = j30d_be16(&data[J30D_CAL_OFF_LOT_NUMBER]);
	c->wafer_no        = j30d_be16(&data[J30D_CAL_OFF_WAFER_NUMBER]);
	c->crc             = j30d_be16(&data[J30D_CAL_OFF_CHECKSUM]);

	c->version = j30d_detect_unit_version(data);

	/* Q8.8 or S7Q8 depending on the unit revision: rev>=2 units (the
	 * only ones with a verified CRC) store signed trim values; everything
	 * else predates that and used the unsigned format. CH2/UNKNOWN carry no
	 * trustworthy trim data and stay at the memset-zero below. */
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
	/* J30D_UNIT_VERSION_CH2 / _UNKNOWN: no trustworthy offset/gain data --
	 * leave sensor_offset1/gain1/offset2/gain2 at the memset-zero above. */

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

static j30d_err_t apply_field(j30d_handle_t *h, uint8_t reg, j30d_config_field_t *shadow,
                               const j30d_config_field_t *desired, const j30d_config_field_t *dflt)
{
	if (desired->status == J30D_FIELD_UNCHANGED)
	{
		return J30D_OK;
	}

	uint8_t value = (desired->status == J30D_FIELD_DEFAULT) ? dflt->value : desired->value;

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

/* The five EEPROM-backed config registers are contiguous, so the shadow can
 * be seeded from the device in one burst read. Checked here rather than
 * trusted, since a future map edit that breaks contiguity would otherwise
 * silently seed the wrong fields. */
#if (J30D_REG_SYS_CFG2   != J30D_REG_SYS_CFG1 + 1) ||     (J30D_REG_SYS_CH1_CFG != J30D_REG_SYS_CFG1 + 2) ||     (J30D_REG_SYS_CH2_CFG != J30D_REG_SYS_CFG1 + 3) ||     (J30D_REG_BIAS_CFG    != J30D_REG_SYS_CFG1 + 4)
#error "EEPROM config registers are no longer contiguous -- fix seed_shadow_from_device()"
#endif
#define J30D_EEPROM_CFG_COUNT   5u

static bool any_eeprom_field_unchanged(const j30d_config_t *cfg)
{
	return cfg->sys_cfg1.status == J30D_FIELD_UNCHANGED
	    || cfg->sys_cfg2.status == J30D_FIELD_UNCHANGED
	    || cfg->ch1_cfg.status  == J30D_FIELD_UNCHANGED
	    || cfg->ch2_cfg.status  == J30D_FIELD_UNCHANGED
	    || cfg->bias_cfg.status == J30D_FIELD_UNCHANGED;
}

/* Overwrite the shadow's EEPROM-field values with what the device actually
 * holds, so that fields the caller leaves J30D_FIELD_UNCHANGED end up with a
 * shadow that matches reality instead of the compiled-in defaults. Only
 * called when at least one such field IS unchanged -- when the caller
 * specifies everything (including the config == NULL case), every field gets
 * written below anyway and this read would be pure overhead.
 *
 * Matters beyond tidiness: decode_gain() reads ch1_cfg/ch2_cfg out of this
 * shadow to scale raw_ch1/raw_ch2 into millivolts, so a stale shadow here
 * silently produces wrong mV. */
static j30d_err_t seed_shadow_from_device(j30d_handle_t *h)
{
	uint8_t buf[J30D_EEPROM_CFG_COUNT];

	j30d_err_t err = h->platform.i2c_read(h->platform.ctx, J30D_REG_SYS_CFG1, buf, sizeof(buf));
	if (err != J30D_OK)
	{
		return err;
	}

	h->config.sys_cfg1.value = buf[0];
	h->config.sys_cfg2.value = buf[1];
	h->config.ch1_cfg.value  = buf[2];
	h->config.ch2_cfg.value  = buf[3];
	h->config.bias_cfg.value = buf[4];
	return J30D_OK;
}

/* Order matters here and is NOT arbitrary: SYS_CFG1, SYS_CFG2, CH1, CH2,
 * SLEEP_TIME, BIAS_CFG, and only THEN the command/sleep-enable write.
 * COMMAND is what actually puts the device to sleep the
 * moment SLEEP_EN lands on the bus -- write it any earlier than last and
 * every subsequent register write in this same call goes out to a
 * sleeping device and fails, since the device stops ACKing I2C until the
 * next return_to_iic() wake. So: everything else first, command last,
 * always. Do not reorder this without re-checking that constraint. */
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
	/* Before COMMAND: a sleeping device does not acknowledge I2C. */
	h->config.interrupt_enable = cfg->interrupt_enable;
	if (h->config.interrupt_enable)
	{
		if ((err = apply_interrupt_mode(h)) != J30D_OK) return err;
	}

	if ((err = apply_field(h, J30D_REG_COMMAND,     &h->config.command,  &cfg->command,  &j30d_default_config.command))  != J30D_OK) return err;

	/* #4: every field now either matches the device (just written) or was
	 * deliberately left alone, so nothing is pending. Normalising here --
	 * rather than only in apply_field()'s write path -- is what makes
	 * `cfg = h->config; cfg.x = ...; j30d_update_config(&h, &cfg)` safe: a
	 * status left at J30D_FIELD_DEFAULT would re-write that register from
	 * the compiled-in defaults on the caller's next update. */
	h->config.chip_address.status = J30D_FIELD_UNCHANGED;
	h->config.command.status  = J30D_FIELD_UNCHANGED;
	h->config.sleep.status    = J30D_FIELD_UNCHANGED;
	h->config.sys_cfg1.status = J30D_FIELD_UNCHANGED;
	h->config.sys_cfg2.status = J30D_FIELD_UNCHANGED;
	h->config.ch1_cfg.status  = J30D_FIELD_UNCHANGED;
	h->config.ch2_cfg.status  = J30D_FIELD_UNCHANGED;
	h->config.bias_cfg.status = J30D_FIELD_UNCHANGED;

	return J30D_OK;
}

/* ---- sleep handling -----------------------------------------------------
 * Sleep is configured iff SLEEP_EN is set in the shadow COMMAND value;
 * there is no separate runtime flag. Every call that touches the bus wakes
 * the device first and re-commands sleep afterwards, so callers never have
 * to know whether it is asleep. That costs a full return_to_iic() per call
 * even when the device was already awake -- always correct, at the price of
 * being occasionally wasteful.
 *
 * j30d_get_data() adds one layer: given platform.now_ms, it will not wake
 * more often than the sleep interval, since no new sample can exist sooner.
 * A pending j30d_notify_interrupt() overrides that for one call. Nothing
 * else is paced.
 */

static bool sleep_is_configured(const j30d_handle_t *h)
{
	return (h->config.command.value & J30D_COMMAND_SLEEP_EN) != 0;
}

/* A wake clears SLEEP_EN but leaves MODE_EN set, so the device comes back
 * converting and a burst read can straddle a conversion boundary. Stopping
 * the engine first freezes the data registers, so preemption between the
 * wake and the read no longer matters.
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

/* Best-effort: re-commands whatever the shadow COMMAND value currently is
 * (which includes SLEEP_EN if sleep is configured). Failure here isn't
 * reported -- the primary operation this wraps already succeeded or failed
 * on its own merits, and there's no reasonable way for a caller to react
 * differently to "my read succeeded but re-arming sleep afterward failed"
 * versus just leaving the device awake until the next call touches it. */
static void restore_sleep_if_needed(j30d_handle_t *h)
{
	if (sleep_is_configured(h))
	{
		uint8_t value = h->config.command.value;
		(void)h->platform.i2c_write(h->platform.ctx, J30D_REG_COMMAND, &value, 1);

		release_bus_to_int(h);
	}
}

/* SLEEP_TIME register -> milliseconds: register value 0x00 == 100ms, then
 * +100ms per step -- see the comment on J30D_REG_SLEEP_TIME in
 * j30d_regs.h. */
static uint32_t sleep_interval_ms(const j30d_handle_t *h)
{
	return ((uint32_t)h->config.sleep.value + 1u) * 100u;
}

/* j30d_get_data()-only pacing: while the device is configured to sleep and
 * the platform gave us a clock, there is no point paying a full wake +
 * register read more often than the device's own sleep interval could
 * possibly produce new data -- the sensor simply hasn't sampled anything
 * new yet. Returns true when the caller should skip the bus entirely this
 * call and just hand back the last known-good data as stale.
 *
 * Deliberately narrow: only j30d_get_data() consults this. j30d_raw_read/
 * write always do exactly what you asked (see their doc comment in
 * j30d.h) -- they're the escape hatch for when you specifically need to
 * talk to the device right now, sleep pacing or not. */
/* How much time must pass before j30d_get_data() will wake the device again.
 *
 * Polling: the full sleep interval. The pacing is pure rate-limiting there --
 * there is nothing new to read before the device has slept and converted.
 *
 * Interrupt mode: half of it, because the pacing has a second job. The device
 * raises INT again a few ms after every sleep command (measured: ~9 ms after
 * the last byte, whatever MODE_EN is doing), so an interrupt-driven read must
 * reject that one and accept the one that ends a sleep interval. Pacing at the
 * full interval cannot: SLEEP_TIME is untrimmed, +/-15% on a 32 kHz
 * oscillator, so a fast part raises its genuine interrupt at ~0.87 of the
 * nominal interval and the full-interval threshold would reject it as well --
 * halving the sample rate on a part that is in spec.
 *
 * Half separates them at every SLEEP_TIME setting. The worst case is the
 * shortest interval, 100 ms: 50 ms against an extra interrupt that arrives
 * ~30 ms after the wake, and against a genuine one no earlier than ~115 ms. */
static uint32_t pacing_interval_ms(const j30d_handle_t *h)
{
	uint32_t interval = sleep_interval_ms(h);
	return h->config.interrupt_enable ? (interval / 2u) : interval;
}

static bool should_skip_get_data_wake(const j30d_handle_t *h)
{
	if (!sleep_is_configured(h) || !h->platform.now_ms || !h->internal.get_data_has_woken)
	{
		return false;
	}
	uint32_t now     = h->platform.now_ms(h->platform.ctx);
	uint32_t elapsed = now - h->internal.last_get_data_wake_ms;   /* wraparound-safe: unsigned subtraction */
	return elapsed < pacing_interval_ms(h);
}

/* ---- raw-write shadow sync ---------------------------------------------
 * j30d_raw_write() bypasses the config model, but the driver still reads
 * h->config for COMMAND, SLEEP_TIME and the channel configs. A raw write to
 * one of those must be reflected in the shadow, or the driver acts on a
 * stale copy -- most visibly by restoring sleep to the old COMMAND value on
 * the way out of the same call, undoing the write.
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
		{ J30D_REG_COMMAND,     &h->config.command  },
		{ J30D_REG_SLEEP_TIME,  &h->config.sleep    },
		{ J30D_REG_SYS_CFG1,    &h->config.sys_cfg1 },
		{ J30D_REG_SYS_CFG2,    &h->config.sys_cfg2 },
		{ J30D_REG_SYS_CH1_CFG, &h->config.ch1_cfg  },
		{ J30D_REG_SYS_CH2_CFG, &h->config.ch2_cfg  },
		{ J30D_REG_BIAS_CFG,    &h->config.bias_cfg },
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
 * platform.lock/unlock are optional; when absent the driver does no locking
 * at all (correct for a single-threaded build, see j30d_platform.h). When
 * present, a lock that reports failure aborts the call with
 * J30D_ERR_TIMEOUT before any bus access, and unlock is NOT called -- there
 * is nothing held to release. */

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
		return J30D_ERR_PARAM;   /* lock/unlock are the only genuinely optional hooks */
	}
	/* J30D_FIELD_UNCHANGED means "leave whatever is currently on the device".
	 * For the two RAM registers that is not a coherent request HERE: this
	 * function soft-resets the device a few lines below, which returns
	 * COMMAND and SLEEP_TIME to their hardware reset values -- whatever they
	 * held is already gone by the time there would be anything to preserve.
	 * Accepting it silently left the shadow claiming the compiled-in default
	 * (which has SLEEP_EN set) while the device held its reset value, and the
	 * first j30d_get_data()'s re-arm step then wrote that default out -- i.e.
	 * "don't touch COMMAND" put the device to sleep. Reject it instead; say
	 * what you want these two set to, or pass config == NULL for the
	 * defaults. EEPROM-backed fields are unaffected: they survive the reset,
	 * so UNCHANGED is meaningful for them and is handled by
	 * seed_shadow_from_device() below. */
	if (config != NULL &&
	    (config->command.status == J30D_FIELD_UNCHANGED ||
	     config->sleep.status   == J30D_FIELD_UNCHANGED))
	{
		return J30D_ERR_PARAM;
	}

	/* Without the hook the device never gets its pin, so INT never fires.
	 *
	 * now_ms is required too, and for a subtler reason: commanding sleep
	 * makes the device convert and raise INT again a few ms later, whatever
	 * MODE_EN was doing -- measured on the bench, not assumed. Only the
	 * pacing in should_skip_get_data_wake() rejects those, and that pacing
	 * needs a clock. Without one, every extra interrupt would be answered
	 * with a read that re-commands sleep and retriggers the same interrupt,
	 * and the device would never complete a sleep interval.
	 *
	 * Sleep itself is required as well: INT is the sleep-mode comparator
	 * output on SDA, so with SLEEP_EN clear the pin is never handed to the
	 * device and no interrupt can ever fire. Accepting that combination
	 * initialises cleanly and then signals nothing, forever. */
	{
		const j30d_config_t *want = (config != NULL) ? config : &j30d_default_config;
		if (want->interrupt_enable &&
		    (platform->release_to_int == NULL || platform->now_ms == NULL ||
		     (want->command.value & J30D_COMMAND_SLEEP_EN) == 0))
		{
			return J30D_ERR_PARAM;
		}
	}

	memset(h, 0, sizeof(*h));
	h->platform = *platform;

	j30d_err_t err = handle_lock(h);
	if (err != J30D_OK)
	{
		return err;   /* nothing acquired, nothing to unlock */
	}

	/* Return-to-IIC-idle/wake sequence -- see j30d_platform.h. Must run
	 * before the soft reset since the device may be asleep, or left in
	 * PWM/Relay output mode (IIC unusable) by a previous run. */
	h->platform.return_to_iic(h->platform.ctx);

	uint8_t reset_val = J30D_SOFT_RESET_Msk;
	err = h->platform.i2c_write(h->platform.ctx, J30D_REG_SOFT_RESET, &reset_val, 1);
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

	/* Best-effort: never fails init, see j30d_read_calibration(). Read
	 * before apply_config() below, though nothing here currently depends
	 * on that ordering. */
	j30d_read_calibration(h);

	/* Seed the shadow with the compiled-in defaults, then apply what the
	 * caller asked for (NULL means use them wholesale).
	 *
	 * Fields left J30D_FIELD_UNCHANGED are not written, so their shadow is
	 * seeded by reading the device back instead -- otherwise h->config would
	 * claim a default for a register nobody wrote.
	 *
	 * Every field ends up J30D_FIELD_UNCHANGED, since the shadow now matches
	 * the device either way. That is what makes `cfg = h->config; cfg.x =
	 * ...; j30d_update_config()` safe. */
	h->config = j30d_default_config;

	if (config != NULL && any_eeprom_field_unchanged(config))
	{
		err = seed_shadow_from_device(h);
		if (err != J30D_OK)
		{
			goto done;
		}
	}

	err = apply_config(h, config ? config : &j30d_default_config);
	if (err != J30D_OK)
	{
		goto done;
	}

	h->initialized = true;
	err = J30D_OK;

	/* apply_config()'s COMMAND write has put the device to sleep. */
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

	/* Wake against whatever mode the device is in right now (old config),
	 * before we potentially change that mode below. */
	j30d_err_t err = ensure_awake(h);
	if (err == J30D_OK)
	{
		err = apply_config(h, config);
	}

	/* Re-check against the NEW config -- this call may have just turned
	 * sleep on or off. If it's on (whether newly, or unchanged), put the
	 * device back into it; if off, leave it awake. */
	restore_sleep_if_needed(h);

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

	/* Consumed whether or not it is acted on. It records that at least one
	 * interrupt arrived, not how many.
	 *
	 * It does not override the pacing below. Commanding the device to sleep
	 * makes it convert and raise INT almost immediately, so answering every
	 * interrupt with a read would re-command sleep and retrigger the same
	 * interrupt, and the device would never complete a sleep interval. The
	 * pacing rejects those: a genuine post-sleep interrupt arrives one
	 * conversion after the interval has elapsed, so it passes. */
	h->internal.interrupt_pending = false;

	if (should_skip_get_data_wake(h))
	{
		/* Too soon since our last wake for this sleeping device to
		 * plausibly have anything new -- don't touch the bus at all.
		 * Same "seed with last known-good, mark everything stale" shape
		 * as the normal no-DRDY-set path below. */
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

	if (sleep_is_configured(h) && h->platform.now_ms)
	{
		h->internal.last_get_data_wake_ms = h->platform.now_ms(h->platform.ctx);
		h->internal.get_data_has_woken    = true;
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

	/* Always read every group whose DRDY bit is set -- see the doc comment
	 * on j30d_get_data() in j30d.h for why there's no `select` mask
	 * anymore: DATA_READY/DATA_READY_RAW clear ALL their bits just from
	 * being read above, whether or not this call goes on to fetch the
	 * corresponding data, so leaving any group unread here would silently
	 * lose that sample. */
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

	/* ch1 and ch2 share one 6-byte burst, but they have SEPARATE DRDY bits
	 * (TO1/TO2) and the device can flag them independently -- in dual mode
	 * one channel can be ready while the other isn't, and in single mode
	 * only the converting channel is ever flagged. So: issue the burst if
	 * EITHER bit is set (one transaction, same as before), but mark only
	 * the channels the device actually said were ready. Marking both fresh
	 * off TO1 alone would report whatever bytes happen to be sitting in
	 * the ch2 registers as a new ch2 sample. */
	if (data_ready & (J30D_TO1_DRDY_Msk | J30D_TO2_DRDY_Msk))
	{
		uint8_t buf[6];   /* ch1 (3) + ch2 (3) -- one burst covers both, see file header note */
		j30d_err_t e = h->platform.i2c_read(h->platform.ctx, J30D_REG_CAL_MSB, buf, sizeof(buf));
		if (e == J30D_OK)
		{
			if (data_ready & J30D_TO1_DRDY_Msk)
			{
				out->ch1.raw_code = sign_extend24(&buf[0]);
#if !defined(J30D_NO_FLOAT)
				out->ch1.value    = (float)out->ch1.raw_code / J30D_VTP_SCALE;
#endif
				out->ch1.state    = J30D_FRESH;
			}
			if (data_ready & J30D_TO2_DRDY_Msk)
			{
				out->ch2.raw_code = sign_extend24(&buf[3]);
#if !defined(J30D_NO_FLOAT)
				out->ch2.value    = (float)out->ch2.raw_code / J30D_VTP_SCALE;
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

	/* Same per-bit rule as the CAL group above, across three fields this
	 * time: raw_ch1/raw_ch2/raw_amb sit in one 9-byte burst but each has
	 * its own DRDY bit (TO1_RAW/TO2_RAW/TA_RAW). Note TO2_RAW was not even
	 * part of the condition before, so a ch2-only raw sample never
	 * triggered the read at all. */
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
				/* No mV conversion here on purpose: the ambient/internal
				 * temperature sensor does not go through the analog front
				 * end, so there is no PGA gain and no AFE reference to
				 * scale by -- a millivolt figure would be meaningless.
				 * .value is the unscaled code, same as .raw_code; use
				 * .raw_code. See the file header note. */
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
 * Deliberately trivial and side-effect free: no bus access, no locking, no
 * blocking (see their doc comment in j30d.h). They exist so application code
 * has a supported way to read handle state without depending on
 * j30d_handle_t's layout. */

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
