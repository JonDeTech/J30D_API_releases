/*
 * j30d.c
 *
 * Core driver implementation: j30d_init(), j30d_update_config(),
 * j30d_get_data(). No OS/HAL/GPIO calls anywhere in this file -- every
 * side effect goes through the j30d_platform_t hooks in the handle.
 *
 * Scope note: this file intentionally does NOT port
 * JIRS30D_CalibrateOffset/CalibrateGain (one-shot factory operations, never
 * verified working in the legacy driver either) or the heat-flux (k/m)
 * conversion in legacy JIRS30D_Convert() -- that's product-specific
 * application math layered on top of the raw calibrated values, and lives
 * in new_evk_sw's own j30d_adapter.c instead, not in this portable driver.
 * Reading the factory calibration/production data block itself IS ported
 * here (j30d_read_calibration(), populating h->calib) -- see j30d_calib.h
 * -- since that's just a register read the driver needs to do anyway, not
 * application logic.
 *
 * Design deviations from the legacy jirs30d.c worth knowing about:
 *
 *   - No "active channel" selection. The legacy driver picks one of
 *     ch1/ch2 at init (from calibration/unit version) and only reports
 *     that one. Since j30d_data_t already has separate ch1/ch2 (and
 *     raw_ch1/raw_ch2) fields, this driver just always reports both --
 *     whichever one is physically wired on a given product, the
 *     customer's application already knows and simply reads that field.
 *     Simpler, and removes a class of "wrong channel selected" bugs.
 *
 *   - raw_amb is NOT converted to millivolts, deliberately. The legacy
 *     RescaleRaw_mV() reused whatever channel's PGA gain (settings_gain)
 *     happened to be cached last for ch1, ch2 *and* amb, and an earlier
 *     revision of this driver carried that forward as a fixed unity gain.
 *     Both are wrong for the same reason: the ambient/internal temperature
 *     sensor does not go through the analog front end at all, so there is
 *     no PGA gain and no AFE reference in its path -- which is also why
 *     the register map has no gain register for it. A millivolt number for
 *     raw_amb does not correspond to anything physical, so none is
 *     produced: raw_amb.value carries the unscaled ADC code and raw_amb is
 *     there to expose exactly that. raw_ch1/raw_ch2 DO run through the
 *     PGA, so their mV conversion is real and is unchanged.
 *
 *   - No struct-over-bytes union tricks (the legacy DataReg24Bit/union
 *     overlays). Multi-byte reads are parsed byte-by-byte with explicit
 *     shifts instead -- overlaying a struct on a byte buffer depends on
 *     compiler packing/alignment/endianness, which isn't something a
 *     driver meant to be portable across arbitrary customer toolchains
 *     should rely on.
 */

#include "j30d.h"
#include "j30d_regs.h"

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

/* J30D_AMB_SCALE / J30D_VTP_SCALE / J30D_RAW_ADC_SCALE are now public,
 * declared in j30d_types.h (raw_code/value round-tripping needs them
 * outside this file too). */
#if !defined(J30D_NO_FLOAT)
#define J30D_ADC_VREF        1.1f
#endif

/* ---- small local helpers -------------------------------------------- */

static int32_t sign_extend24(const uint8_t *b /* 3 bytes, MSB first */)
{
	uint32_t v = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[2];
	if (v & 0x800000u)
	{
		v |= 0xFF000000u;
	}
	return (int32_t)v;
}

#if !defined(J30D_NO_FLOAT)
static float decode_gain(uint8_t cfg_reg_value)
{
	static const float gain_table[8] = { 8.0f, 12.0f, 16.0f, 32.0f, 48.0f, 64.0f, 96.0f, 128.0f };
	uint8_t idx = (cfg_reg_value & J30D_SYS_CHx_CFG_GAIN_Msk) >> J30D_SYS_CHx_CFG_GAIN_Pos;
	return gain_table[idx & 0x7u];
}

static float raw_to_mv(int32_t raw, float gain)
{
	return 1000.0f * ((float)raw * J30D_ADC_VREF) / (gain * J30D_RAW_ADC_SCALE);
}
#endif /* !J30D_NO_FLOAT */

/* ---- calibration block parsing ----------------------------------------
 * Ported bit-for-bit from the legacy driver's crc16()/detect_unit_version()
 * /ReadProductionData() (see j30d_calib.h). Kept as free functions (not
 * static-local to j30d_init) so they're easy to unit-test in isolation
 * against known-good byte sequences from the legacy implementation.
 */

#define J30D_CRC16_POLY   0x8408u   /* reflected CCITT poly, matches legacy verbatim */

static uint16_t j30d_crc16(const uint8_t *data, uint16_t length)
{
	uint32_t crc = 0xFFFFu;

	if (length == 0)
	{
		return (uint16_t)(~crc);
	}

	while (length--)
	{
		uint32_t byte = *data++;
		for (uint8_t i = 0; i < 8; i++, byte >>= 1)
		{
			if ((crc & 0x0001u) ^ (byte & 0x0001u))
			{
				crc = (crc >> 1) ^ J30D_CRC16_POLY;
			}
			else
			{
				crc >>= 1;
			}
		}
	}

	crc = (~crc) & 0xFFFFu;
	uint32_t swapped = ((crc << 8) | ((crc >> 8) & 0xFFu)) & 0xFFFFu;
	return (uint16_t)swapped;
}

/* The raw 16-bit trim word, before any Q-format interpretation. */
static uint16_t j30d_trim_word(uint8_t msb, uint8_t lsb)
{
	return (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}

#if !defined(J30D_NO_FLOAT)
/* U8Q8: 8-bit unsigned integer part + 8-bit fraction (range 0..255.996). */
static float j30d_u8q8_to_float(uint8_t msb, uint8_t lsb)
{
	return (float)j30d_trim_word(msb, lsb) / 256.0f;
}

/* S7Q8: sign bit + 7-bit integer part + 8-bit fraction (range -128..127.996). */
static float j30d_s7q8_to_float(uint8_t msb, uint8_t lsb)
{
	return (float)(int16_t)j30d_trim_word(msb, lsb) / 256.0f;
}
#endif /* !J30D_NO_FLOAT */

static uint32_t j30d_be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static uint16_t j30d_be16(const uint8_t *p)
{
	return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

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

	/* Same Q8.8-vs-S7Q8 split the legacy driver used: rev>=2 units (the
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
 * Three registers can put a unit into a state this driver cannot talk it
 * out of. All three are refused with J30D_ERR_PARAM by default, on every
 * path that writes (j30d_init, j30d_update_config, j30d_raw_write), and
 * each has its own compile-time opt-out for the rare deliberate case.
 *
 *   SYS_CFG1 (0x93) OUT_MODE -- selects I2C / PWM / Relay. Setting it to
 *     anything but I2C multiplexes SDA into a push-pull output and the IIC
 *     interface stops working. The real problem is that SYS_CFG1 is
 *     EEPROM-backed, so it survives power cycles and soft resets: the unit
 *     comes back up in PWM/Relay mode every time, and return_to_iic()
 *     buys only a momentary window rather than a fix. In practice this has
 *     bricked units on the bench. Note the guard is one-directional -- it
 *     rejects moving OFF I2C but always permits writing OUT_MODE back TO
 *     I2C, so recovering a unit that is already in this state still works.
 *     Every other bit of SYS_CFG1 (OSR, filter) is unaffected.
 *
 *   BLOW_START (0x40) -- blows the EEPROM fuse. Irreversible, production
 *     tooling only.
 *
 *   CUSTOM_MEMORY (0xC0..0xEF) -- the factory calibration block. Writing
 *     it destroys this unit's calibration permanently; the driver only
 *     ever reads it (see j30d_calib.h).
 *
 * These are build-time escapes, not runtime flags, so enabling one is a
 * deliberate decision recorded in the build rather than something a stray
 * call can do. */

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

/* Order matters here and is NOT arbitrary -- ported from the legacy
 * driver's setup sequence (SysCfg1, SysCfg2, Ch1Cfg, Ch2Cfg, sleep,
 * BpsCfg, and only THEN the command/sleep-enable write, "lastly, start
 * the device"). COMMAND is what actually puts the device to sleep the
 * moment SLEEP_EN lands on the bus -- write it any earlier than last and
 * every subsequent register write in this same call goes out to a
 * sleeping device and fails, since the device stops ACKing I2C until the
 * next return_to_iic() wake. So: everything else first, command last,
 * always. Do not reorder this without re-checking that constraint. */
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
 * Whether the device is configured to sleep is just the SLEEP_EN bit of
 * the shadow COMMAND value -- same convention the legacy driver used
 * (device_sleeping() there was exactly this bit test, not a separately
 * tracked runtime flag). Every call below that touches the bus wakes the
 * device first if sleep is configured, then re-commands sleep afterward --
 * the caller never has to know or care whether the device happens to be
 * asleep right now. This costs a full return_to_iic() (the GPIO wake
 * sequence, tens of ms) on every such call when sleep is enabled, even if
 * the device turns out to already be awake -- deliberately simple and
 * always-correct over trying to track actual runtime sleep state.
 *
 * j30d_get_data() alone gets a second layer on top: if platform.now_ms is
 * provided, it also refuses to wake more often than the configured sleep
 * interval, since a sleeping device simply cannot have produced a new
 * sample any faster than that -- see should_skip_get_data_wake() below.
 * Without now_ms, j30d_get_data() falls back to the always-wake behavior
 * above (correct, just potentially wasteful if called faster than the
 * sleep interval). j30d_update_config()/j30d_raw_read()/j30d_raw_write()
 * are never paced this way -- they always do what you asked, immediately.
 */

static bool sleep_is_configured(const j30d_handle_t *h)
{
	return (h->config.command.value & J30D_COMMAND_SLEEP_EN) != 0;
}

static void ensure_awake(j30d_handle_t *h)
{
	if (sleep_is_configured(h))
	{
		h->platform.return_to_iic(h->platform.ctx);
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
	}
}

/* SLEEP_TIME register -> milliseconds: register value 0x00 == 100ms, then
 * +100ms per step (matches the legacy driver's convention, see the comment
 * on J30D_REG_SLEEP_TIME in j30d_regs.h). */
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
static bool should_skip_get_data_wake(const j30d_handle_t *h)
{
	if (!sleep_is_configured(h) || !h->platform.now_ms || !h->internal.get_data_has_woken)
	{
		return false;
	}
	uint32_t now     = h->platform.now_ms(h->platform.ctx);
	uint32_t elapsed = now - h->internal.last_get_data_wake_ms;   /* wraparound-safe: unsigned subtraction */
	return elapsed < sleep_interval_ms(h);
}

/* ---- raw-write shadow sync ---------------------------------------------
 * j30d_raw_write() bypasses the config model, but the driver still acts on
 * h->config: sleep_is_configured() and restore_sleep_if_needed() read
 * COMMAND out of it, sleep_interval_ms() reads SLEEP_TIME, and decode_gain()
 * reads the channel configs to scale raw_ch1/raw_ch2 into millivolts. A raw
 * write that lands on one of those registers therefore has to be reflected
 * in the shadow, or the driver goes on acting from a stale copy -- most
 * visibly by "restoring" sleep to the OLD command value immediately after
 * the caller wrote a new one, undoing the write on the way out of the same
 * call.
 *
 * Register addresses auto-increment across a burst, so a write of `len`
 * bytes starting at `reg` covers reg .. reg+len-1; any modelled register in
 * that span is updated from the corresponding byte.
 *
 * Not covered, and deliberately so: a raw write to J30D_REG_SOFT_RESET.
 * That resets the device and reloads EEPROM behind the driver's back, which
 * invalidates the whole shadow (and the calibration read) in a way no
 * bookkeeping here can repair -- call j30d_init() again instead. */
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
	 * before apply_config() below, matching the legacy driver's order
	 * (ReadProductionData() before the sleep/config write), though nothing
	 * here currently depends on that ordering. */
	j30d_read_calibration(h);

	/* Seed the shadow with register addresses + compiled-in defaults, then
	 * apply whatever the caller actually asked for (NULL == "just use
	 * j30d_default_config wholesale").
	 *
	 * Fields left J30D_FIELD_UNCHANGED in a caller-supplied config are not
	 * written -- the device keeps whatever the EEPROM held before this
	 * reset -- so for those the shadow is seeded by reading the device back
	 * first, otherwise h->config would claim the compiled-in default for a
	 * register nobody wrote. (COMMAND/SLEEP_TIME can't reach here as
	 * UNCHANGED; see the param check at the top of this function.)
	 *
	 * Every field's status ends up J30D_FIELD_UNCHANGED: read-back or
	 * written, the shadow now matches the device either way, so there is
	 * nothing pending. That also makes the natural read-modify-write idiom
	 * (`cfg = h->config; cfg.x = ...; j30d_update_config()`) safe -- a
	 * status left at J30D_FIELD_DEFAULT would re-write that register from
	 * the compiled-in defaults on the next update. */
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
	ensure_awake(h);

	j30d_err_t err = apply_config(h, config);

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

	ensure_awake(h);
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
	 * off TO1 alone -- as this used to -- reported whatever bytes happened
	 * to be sitting in the ch2 registers as a new ch2 sample. */
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

	ensure_awake(h);
	j30d_err_t err = h->platform.i2c_read(h->platform.ctx, reg, buf, len);
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

	ensure_awake(h);
	j30d_err_t err = h->platform.i2c_write(h->platform.ctx, reg, buf, len);
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
