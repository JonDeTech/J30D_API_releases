/* Standalone test: a fake in-memory "sensor" implementing the platform
 * hooks, exercised through the real j30d_init/update_config/get_data. Not
 * part of the deliverable -- just verification that j30d.c actually does
 * what its comments claim before handing it over. */

#include "j30d.h"
#include "j30d_regs.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

typedef struct
{
	uint8_t  regs[256];
	int      fail_next_read;
	int      fail_next_write;
	int      return_to_iic_calls;
	int      i2c_read_calls;     /* every fake_read() call, pass or fail -- proves whether the bus was touched at all */
	uint32_t virtual_now_ms;     /* driven by the test, read back by fake_now_ms() */
	int      lock_calls;
	int      unlock_calls;
	int      lock_should_fail;   /* when set, fake_lock() reports it could not acquire */
	/* I2C addressing, as measured on hardware by J30D_LiveTest_AddressProbe():
	 * writing CHIP_ADDRESS moves the device IMMEDIATELY -- no EEPROM, no
	 * reset -- and the 0x7F wildcard keeps answering alongside whatever
	 * address is set. */
	uint8_t  bus_addr;           /* address the port is driving */
	uint8_t  chip_addr;          /* address the device currently answers on */
	int      device_asleep;      /* models real hardware: once COMMAND lands with SLEEP_EN set, the
	                               * device stops ACKing anything until the next return_to_iic() wake --
	                               * this is what apply_config()'s write ordering has to survive. */
} fake_ctx_t;

/* Every fake context starts on the 0x7F wildcard, because that is what the
 * reference ports target and what makes the driver survive a chip_address
 * change. A context left at address 0 only works while the compiled-in
 * default leaves chip_address UNCHANGED -- edit that default (as a customer
 * would) and the device moves out from under the test. */
#define FAKE_CTX_INIT  { .bus_addr = 0x7Fu }

/* The part answers its own address and, always, the 0x7F wildcard. */
static int fake_addressed(const fake_ctx_t *c)
{
	return c->bus_addr == c->chip_addr || c->bus_addr == 0x7Fu;
}

static j30d_err_t fake_read(void *ctx, uint8_t reg, uint8_t *buf, size_t len)
{
	fake_ctx_t *c = (fake_ctx_t *)ctx;
	c->i2c_read_calls++;
	if (!fake_addressed(c)) { return J30D_ERR_I2C; }
	if (c->device_asleep) { return J30D_ERR_I2C; }
	if (c->fail_next_read) { c->fail_next_read = 0; return J30D_ERR_I2C; }
	memcpy(buf, &c->regs[reg], len);
	/* Models real hardware (per field observation): reading DATA_READY/
	 * DATA_READY_RAW clears ALL of their bits, not just the ones the
	 * caller goes on to act on. j30d_get_data() reads both in the same
	 * 3-byte burst starting at J30D_REG_DATA_READY -- this is the only
	 * call site that ever hits this address, so gating on `reg` alone is
	 * enough to model it here. This is exactly the hazard the removed
	 * `select` parameter used to hide: a caller that only reads out some
	 * of the groups this same call saw ready would silently lose the
	 * DRDY for the rest, since it's gone by the next status read. */
	if (reg == J30D_REG_DATA_READY)
	{
		c->regs[J30D_REG_DATA_READY]     &= (uint8_t)~(J30D_TA_DRDY_Msk | J30D_TO1_DRDY_Msk | J30D_TO2_DRDY_Msk);
		c->regs[J30D_REG_DATA_READY_RAW] &= (uint8_t)~(J30D_TA_RAW_DRDY_Msk | J30D_TO1_RAW_DRDY_Msk | J30D_TO2_RAW_DRDY_Msk);
	}
	return J30D_OK;
}

static j30d_err_t fake_write(void *ctx, uint8_t reg, const uint8_t *buf, size_t len)
{
	fake_ctx_t *c = (fake_ctx_t *)ctx;
	if (!fake_addressed(c)) { return J30D_ERR_I2C; }
	if (c->device_asleep) { return J30D_ERR_I2C; }
	if (c->fail_next_write) { c->fail_next_write = 0; return J30D_ERR_I2C; }
	memcpy(&c->regs[reg], buf, len);
	/* Takes effect at once -- this is the behaviour the hardware probe
	 * measured, and what makes EEPROM programming unnecessary. */
	if (reg == J30D_REG_CHIP_ADDRESS) { c->chip_addr = (uint8_t)(buf[0] & 0x7Fu); }
	/* SOFT_RESET write "loads the eeprom" in this fake */
	if (reg == J30D_REG_SOFT_RESET) { c->regs[J30D_REG_ERROR_CODE] |= J30D_ERROR_CODE_EEPROM_LOADED_Msk; }
	/* Mirrors real hardware: this write itself still succeeds (the device
	 * was awake to ACK it) -- it's every access AFTER this one that fails,
	 * until the next wake. */
	if (reg == J30D_REG_COMMAND) { c->device_asleep = (buf[0] & J30D_COMMAND_SLEEP_EN) ? 1 : 0; }
	return J30D_OK;
}

static void fake_delay(void *ctx, uint32_t ms) { (void)ctx; (void)ms; }
static void fake_return_to_iic(void *ctx) { fake_ctx_t *c = (fake_ctx_t *)ctx; c->return_to_iic_calls++; c->device_asleep = 0; }
static uint32_t fake_now_ms(void *ctx) { return ((fake_ctx_t *)ctx)->virtual_now_ms; }

static bool fake_lock(void *ctx)
{
	fake_ctx_t *c = (fake_ctx_t *)ctx;
	c->lock_calls++;
	return c->lock_should_fail ? false : true;
}
static void fake_unlock(void *ctx) { ((fake_ctx_t *)ctx)->unlock_calls++; }

static void set_amb(fake_ctx_t *c, int32_t raw)
{
	c->regs[J30D_REG_AMB_MSB + 0] = (uint8_t)((uint32_t)raw >> 16);
	c->regs[J30D_REG_AMB_MSB + 1] = (uint8_t)((uint32_t)raw >> 8);
	c->regs[J30D_REG_AMB_MSB + 2] = (uint8_t)raw;
	c->regs[J30D_REG_DATA_READY] |= J30D_TA_DRDY_Msk;
}

static void set_cal(fake_ctx_t *c, int32_t ch1_raw, int32_t ch2_raw)
{
	c->regs[J30D_REG_CAL_MSB + 0] = (uint8_t)((uint32_t)ch1_raw >> 16);
	c->regs[J30D_REG_CAL_MSB + 1] = (uint8_t)((uint32_t)ch1_raw >> 8);
	c->regs[J30D_REG_CAL_MSB + 2] = (uint8_t)ch1_raw;
	c->regs[J30D_REG_CAL_MSB + 3] = (uint8_t)((uint32_t)ch2_raw >> 16);
	c->regs[J30D_REG_CAL_MSB + 4] = (uint8_t)((uint32_t)ch2_raw >> 8);
	c->regs[J30D_REG_CAL_MSB + 5] = (uint8_t)ch2_raw;
	/* Stages a sample for BOTH channels, so raise both channels' DRDY bits
	 * -- ch1 and ch2 have separate bits and the driver gates each field on
	 * its own. Raising only TO1 here (as this used to) modelled a device
	 * that never flags ch2, which is what set_cal_ch1_only() below is for. */
	c->regs[J30D_REG_DATA_READY] |= J30D_TO1_DRDY_Msk | J30D_TO2_DRDY_Msk;
}

/* Stages a ch1 sample only: ch1's DRDY is raised, ch2's is not, but the ch2
 * registers still hold whatever bytes were there. This is what the device
 * actually looks like in single-conversion mode, and the case that used to
 * be misreported as a fresh ch2 sample. */
static void set_cal_ch1_only(fake_ctx_t *c, int32_t ch1_raw, int32_t stale_ch2_bytes)
{
	set_cal(c, ch1_raw, stale_ch2_bytes);
	c->regs[J30D_REG_DATA_READY] &= (uint8_t)~J30D_TO2_DRDY_Msk;
}

/* Raises the raw-group DRDY bits named in `bits` only, after staging all
 * three raw channels' registers. */
static void set_raw(fake_ctx_t *c, int32_t ch1, int32_t ch2, int32_t amb, uint8_t bits)
{
	const int32_t v[3] = { ch1, ch2, amb };
	for (int i = 0; i < 3; i++)
	{
		c->regs[J30D_REG_RAW_MSB + i * 3 + 0] = (uint8_t)((v[i] >> 16) & 0xFF);
		c->regs[J30D_REG_RAW_MSB + i * 3 + 1] = (uint8_t)((v[i] >> 8) & 0xFF);
		c->regs[J30D_REG_RAW_MSB + i * 3 + 2] = (uint8_t)(v[i] & 0xFF);
	}
	c->regs[J30D_REG_DATA_READY_RAW] |= bits;
}

/* --- test-side re-implementation of the driver's calibration-block codec,
 * used only to build known-good fake EEPROM byte sequences to feed the
 * driver. Deliberately independent code (not calling into j30d.c's static
 * helpers) so a passing test means the driver's own implementation agrees
 * with an independently-written one, not just with itself. */

static uint16_t test_crc16(const uint8_t *data, uint16_t length)
{
	uint32_t crc = 0xFFFFu;
	if (length == 0) { return (uint16_t)(~crc); }
	while (length--)
	{
		uint32_t byte = *data++;
		for (uint8_t i = 0; i < 8; i++, byte >>= 1)
		{
			crc = ((crc & 1u) ^ (byte & 1u)) ? ((crc >> 1) ^ 0x8408u) : (crc >> 1);
		}
	}
	crc = (~crc) & 0xFFFFu;
	return (uint16_t)(((crc << 8) | (crc >> 8)) & 0xFFFFu);
}

static void test_put_be32(uint8_t *p, uint32_t v) { p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v; }
static void test_put_be16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)(v>>8); p[1]=(uint8_t)v; }
static void test_put_s7q8(uint8_t *p, float v)     { int16_t fixed = (int16_t)(v * 256.0f); p[0] = (uint8_t)(fixed >> 8); p[1] = (uint8_t)(fixed & 0xFF); }

/* Fills ctx->regs[0xC0..0xEF] with a REV2PLUS-shaped calibration block
 * (revision=2, valid CRC, S7Q8-encoded offset/gain) at the given channel-1
 * offset/gain, and returns a fresh copy of the raw block for the caller to
 * cross-check field-by-field if needed. */
static void set_calibration_rev2(fake_ctx_t *c, float offset1, float gain1)
{
	uint8_t *b = &c->regs[J30D_REG_CUSTOM_MEMORY_START];
	memset(b, 0, J30D_CUSTOM_MEMORY_SIZE);
	test_put_be32(&b[J30D_CAL_OFF_PROD_TIMESTAMP], 1735689600u);   /* arbitrary */
	test_put_be32(&b[J30D_CAL_OFF_UNIQUE_ID], 0xDEADBEEFu);
	b[J30D_CAL_OFF_CHIP_REVISION] = 2;
	test_put_be16(&b[J30D_CAL_OFF_LOT_NUMBER], 1234);
	test_put_be16(&b[J30D_CAL_OFF_WAFER_NUMBER], 7);
	test_put_s7q8(&b[J30D_CAL_OFF_SENSOR_OFFSET_1], offset1);
	test_put_s7q8(&b[J30D_CAL_OFF_SENSOR_GAIN_1], gain1);
	uint16_t crc = test_crc16(b, J30D_CUSTOM_MEMORY_CRC_LEN);
	test_put_be16(&b[J30D_CAL_OFF_CHECKSUM], crc);
}

int main(void)
{
	fake_ctx_t ctx = FAKE_CTX_INIT;
	j30d_platform_t platform = {
		.i2c_read = fake_read, .i2c_write = fake_write,
		.delay_ms = fake_delay, .return_to_iic = fake_return_to_iic,
		.lock = NULL, .unlock = NULL, .ctx = &ctx,
	};

	/* Seed a valid REV2PLUS calibration block *before* init reads it. */
	set_calibration_rev2(&ctx, -2.5f, 10.75f);

	j30d_handle_t h;

	/* --- init with defaults --- */
	assert(j30d_init(&h, &platform, NULL) == J30D_OK);
	assert(h.initialized);
	assert(ctx.return_to_iic_calls == 1);
	assert(ctx.regs[J30D_REG_SYS_CFG1] == j30d_default_config.sys_cfg1.value);
	printf("init OK, version=%s\n", j30d_get_version_string());

	/* --- calibration block was read and parsed correctly at init --- */
	assert(h.calib.valid);
	assert(h.calib.version == J30D_UNIT_VERSION_REV2PLUS);
	assert(h.calib.revision == 2);
	assert(h.calib.uid == 0xDEADBEEFu);
	assert(h.calib.lot_no == 1234);
	assert(h.calib.wafer_no == 7);
#if defined(J30D_NO_FLOAT)
	/* S7Q8 words, undecoded: -2.5 * 256 == -640, 10.75 * 256 == 2752 */
	assert((int16_t)h.calib.sensor_offset1_raw == -640);
	assert((int16_t)h.calib.sensor_gain1_raw == 2752);
#else
	assert(h.calib.sensor_offset1 > -2.501f && h.calib.sensor_offset1 < -2.499f);
	assert(h.calib.sensor_gain1 > 10.749f && h.calib.sensor_gain1 < 10.751f);
#endif
	printf("calibration (REV2PLUS, valid CRC) parsed OK\n");

	/* --- a corrupted CRC on a rev>=2 block must be reported as UNKNOWN, not silently accepted --- */
	{
		fake_ctx_t bad_ctx = FAKE_CTX_INIT;
		j30d_platform_t bad_platform = platform;
		bad_platform.ctx = &bad_ctx;
		set_calibration_rev2(&bad_ctx, 1.0f, 1.0f);
		bad_ctx.regs[J30D_REG_CUSTOM_MEMORY_START + J30D_CAL_OFF_CHECKSUM] ^= 0xFF;   /* corrupt it */
		j30d_handle_t bad_h;
		assert(j30d_init(&bad_h, &bad_platform, NULL) == J30D_OK);   /* bad calib must not fail init */
		assert(bad_h.calib.valid);   /* the read+parse itself succeeded */
		assert(bad_h.calib.version == J30D_UNIT_VERSION_UNKNOWN);   /* but the CRC didn't check out */
		printf("calibration (REV2PLUS, corrupted CRC) -> UNKNOWN OK\n");
	}

	/* --- an all-0xFF block (earliest units, no calibration data at all) --- */
	{
		fake_ctx_t ff_ctx = FAKE_CTX_INIT;
		j30d_platform_t ff_platform = platform;
		ff_platform.ctx = &ff_ctx;
		memset(&ff_ctx.regs[J30D_REG_CUSTOM_MEMORY_START], 0xFF, J30D_CUSTOM_MEMORY_SIZE);
		j30d_handle_t ff_h;
		assert(j30d_init(&ff_h, &ff_platform, NULL) == J30D_OK);
		assert(ff_h.calib.valid);
		assert(ff_h.calib.version == J30D_UNIT_VERSION_CH2);
#if defined(J30D_NO_FLOAT)
		assert(ff_h.calib.sensor_gain1_raw == 0u);   /* no trustworthy trim data -- left zeroed */
#else
		assert(ff_h.calib.sensor_gain1 == 0.0f);   /* no trustworthy trim data -- left zeroed */
#endif
		printf("calibration (all-0xFF block) -> CH2/no-cal-data OK\n");
	}

	/* --- a pre-rev2 unit: revision byte never written (0xFF), blank checksum,
	 * but real trim and a real timestamp. 0xFF must NOT be read as "revision
	 * 255, so >= 2" -- that routes it into the CRC branch, where a blank
	 * checksum cannot match and the unit comes back UNKNOWN with its
	 * calibration thrown away. It belongs in the timestamp path. */
	{
		fake_ctx_t old_ctx = FAKE_CTX_INIT;
		j30d_platform_t old_platform = platform;
		old_platform.ctx = &old_ctx;

		uint8_t *b = &old_ctx.regs[J30D_REG_CUSTOM_MEMORY_START];
		memset(b, 0, J30D_CUSTOM_MEMORY_SIZE);
		test_put_be32(&b[J30D_CAL_OFF_PROD_TIMESTAMP], 1700000000u);
		test_put_be32(&b[J30D_CAL_OFF_UNIQUE_ID], 51u);
		b[J30D_CAL_OFF_CHIP_REVISION] = 0xFFu;                      /* never programmed */
		test_put_s7q8(&b[J30D_CAL_OFF_SENSOR_OFFSET_1], 17.6015625f);
		test_put_s7q8(&b[J30D_CAL_OFF_SENSOR_GAIN_1], 3.25f);
		memset(&b[J30D_CAL_OFF_CHECKSUM], 0xFF, 2);                 /* blank, as on these units */

		j30d_handle_t old_h;
		assert(j30d_init(&old_h, &old_platform, NULL) == J30D_OK);
		assert(old_h.calib.valid);
		assert(old_h.calib.version == J30D_UNIT_VERSION_CH1_NOTIMESTAMP);
		assert(old_h.calib.uid == 51u);
#if defined(J30D_NO_FLOAT)
		assert((int16_t)old_h.calib.sensor_gain1_raw == 832);       /* 3.25 * 256 */
#else
		assert(old_h.calib.sensor_gain1 > 3.249f && old_h.calib.sensor_gain1 < 3.251f);
#endif
		printf("calibration (revision=0xFF, pre-rev2) keeps its trim OK\n");
	}

	/* (EEPROM-not-loaded failure path isn't drilled here -- this fake
	 * always sets the loaded bit on soft-reset; the polling loop itself is
	 * straight-line code already covered by the happy-path init above.) */

	/* --- update_config: partial update, only sleep field --- */
	j30d_config_t upd = h.config;
	memset(&upd, 0, sizeof(upd));
	upd.sleep.status = J30D_FIELD_UPDATED;
	upd.sleep.value  = 0x20;
	upd.command.status = J30D_FIELD_UNCHANGED;
	assert(j30d_update_config(&h, &upd) == J30D_OK);
	assert(ctx.regs[J30D_REG_SLEEP_TIME] == 0x20);
	assert(h.config.sleep.value == 0x20);
	assert(h.config.sleep.status == J30D_FIELD_UNCHANGED);
	/* command register must be untouched by the partial update */
	assert(ctx.regs[J30D_REG_COMMAND] == j30d_default_config.command.value);
	printf("update_config partial OK\n");

	/* --- apply_config() write order: command (which puts the device to
	 * sleep the instant SLEEP_EN lands) must be written LAST, or every
	 * other register in this same call goes out to an already-sleeping,
	 * non-ACKing device and fails. One update_config() call that changes
	 * both sys_cfg1 AND enables sleep is the case that actually exercises
	 * this -- the fake models the sleep-cuts-off-I2C behavior for real, so
	 * a wrong order here fails this assertion, not just a comment. */
	{
		j30d_config_t combined = {0};
		combined.sys_cfg1.status = J30D_FIELD_UPDATED;
		combined.sys_cfg1.value  = 0xE1;   /* OUT_MODE stays I2C (bits 4:3 == 0); 0x55 would select Relay */
		combined.command.status  = J30D_FIELD_UPDATED;
		combined.command.value   = j30d_default_config.command.value | J30D_COMMAND_SLEEP_EN;
		assert(j30d_update_config(&h, &combined) == J30D_OK);
		assert(ctx.regs[J30D_REG_SYS_CFG1] == 0xE1);          /* would still be the old value under the old (broken) order */
		assert(ctx.regs[J30D_REG_COMMAND] & J30D_COMMAND_SLEEP_EN);
		assert(h.config.sys_cfg1.value == 0xE1);

		/* restore -- sys_cfg1 back to its real default (this sub-test only
		 * changed it to 0xE1 to prove the write-order claim above), so the
		 * rest of the suite starts from the same state a plain j30d_init()
		 * leaves it in. That state is asleep: j30d_default_config.command
		 * has SLEEP_EN set (this is what calibration actually runs with,
		 * per JDT), so "restore to default" is NOT "turn sleep off" any
		 * more -- it's putting sleep back ON, same as it already was. */
		j30d_config_t restore = {0};
		restore.sys_cfg1.status = J30D_FIELD_UPDATED;
		restore.sys_cfg1.value  = j30d_default_config.sys_cfg1.value;
		restore.command.status  = J30D_FIELD_UPDATED;
		restore.command.value   = j30d_default_config.command.value;
		assert(j30d_update_config(&h, &restore) == J30D_OK);
		assert(ctx.regs[J30D_REG_SYS_CFG1] == j30d_default_config.sys_cfg1.value);
		assert(ctx.regs[J30D_REG_COMMAND] & J30D_COMMAND_SLEEP_EN);   /* sleep stays on -- that's the real default */
	}
	printf("apply_config writes command last (sleep doesn't cut off other writes in the same call) OK\n");

	/* --- get_data: nothing ready yet -> all STALE, value 0 --- */
	j30d_data_t d;
	assert(j30d_get_data(&h, &d) == J30D_OK);
	assert(d.amb.state == J30D_STALE);
#if !defined(J30D_NO_FLOAT)
	assert(d.amb.value == 0.0f);
#endif
	assert(d.amb.raw_code == 0);
	printf("get_data (nothing ready) OK\n");

	/* --- get_data: amb ready --- */
	set_amb(&ctx, -16384);   /* -1.0 after /2^14 scale */
	assert(j30d_get_data(&h, &d) == J30D_OK);
	assert(d.amb.state == J30D_FRESH);
	assert(d.amb.raw_code == -16384);
#if defined(J30D_NO_FLOAT)
	printf("get_data (amb fresh, raw_code=%d) OK\n", d.amb.raw_code);
	/* The scale relationship still holds, expressed as a shift. */
	assert((d.amb.raw_code >> J30D_AMB_SCALE_SHIFT) == -1);
#else
	assert(d.amb.value < -0.999f && d.amb.value > -1.001f);
	printf("get_data (amb fresh, value=%f, raw_code=%d) OK\n", (double)d.amb.value, d.amb.raw_code);

	/* raw_code/value round-trip must be exact (power-of-two scale) */
	assert((float)d.amb.raw_code / J30D_AMB_SCALE == d.amb.value);
	assert(J30D_AMB_SCALE == (float)(1UL << J30D_AMB_SCALE_SHIFT));
#endif

	/* --- get_data again without new data ready: amb must fall back to STALE but keep last value --- */
	ctx.regs[J30D_REG_DATA_READY] &= (uint8_t)~J30D_TA_DRDY_Msk;
	assert(j30d_get_data(&h, &d) == J30D_OK);
	assert(d.amb.state == J30D_STALE);
	assert(d.amb.raw_code == -16384);   /* still last known-good */
	printf("get_data (stale keeps last value) OK\n");

	/* --- regression: get_data() must not lose a group's data to a status
	 * read that clears DRDY bits it didn't itself consume. This is the bug
	 * the old `select` parameter caused on real hardware: a caller polling
	 * for CAL only would read (and thus clear, per fake_read() above) an
	 * AMB DRDY it never went on to fetch, starving a later AMB-only call
	 * forever. Now that a single call always reads every group whose DRDY
	 * came back set in the SAME status read, both must come back fresh
	 * together, with nothing to starve. */
	set_amb(&ctx, 2048);
	set_cal(&ctx, 100, -200);
	assert(j30d_get_data(&h, &d) == J30D_OK);
	assert(d.amb.state == J30D_FRESH);
	assert(d.amb.raw_code == 2048);
	assert(d.ch1.state == J30D_FRESH);
	assert(d.ch1.raw_code == 100);
	assert(d.ch2.state == J30D_FRESH);
	assert(d.ch2.raw_code == -200);
	/* both DRDY bits must have been consumed by that one call, not left
	 * for a second call to (fail to) pick up */
	assert((ctx.regs[J30D_REG_DATA_READY] & (J30D_TA_DRDY_Msk | J30D_TO1_DRDY_Msk)) == 0);
	printf("get_data reads every ready group in one call, none lost to the status-read-clears-all-bits hazard OK\n");

	/* --- per-field DRDY gating: a field is FRESH only when the device set
	 * ITS bit, never because another field sharing the same burst read was
	 * ready. ch1/ch2 come from one 6-byte read and the three raw fields
	 * from one 9-byte read, so this is the property that stops a shared
	 * transaction from inventing samples. --- */
	{
		/* ch1 ready, ch2 not -- ch2 must stay STALE holding its previous
		 * value (-200 from the block above), NOT the 0x7F7F7F garbage
		 * sitting in its registers. */
		set_cal_ch1_only(&ctx, 12345, 0x7F7F7F);
		assert(j30d_get_data(&h, &d) == J30D_OK);
		assert(d.ch1.state == J30D_FRESH);
		assert(d.ch1.raw_code == 12345);
		assert(d.ch2.state == J30D_STALE);
		assert(d.ch2.raw_code == -200);   /* last known-good, not the unflagged register contents */
		printf("get_data: ch2 not flagged -> STALE, keeps last known-good (not ch1's bit) OK\n");

		/* And the reverse: ch2 ready, ch1 not. This case never even
		 * triggered the CAL read before, since only TO1 was tested. */
		set_cal(&ctx, 111, 222);
		ctx.regs[J30D_REG_DATA_READY] &= (uint8_t)~J30D_TO1_DRDY_Msk;
		assert(j30d_get_data(&h, &d) == J30D_OK);
		assert(d.ch2.state == J30D_FRESH);
		assert(d.ch2.raw_code == 222);
		assert(d.ch1.state == J30D_STALE);
		assert(d.ch1.raw_code == 12345);
		printf("get_data: ch1 not flagged -> ch2 still read and fresh OK\n");

		/* Raw group: only ch1's raw bit set. raw_ch2/raw_amb must stay
		 * STALE even though their bytes came back in the same burst. */
		set_raw(&ctx, 0x010203, 0x040506, 0x070809, J30D_TO1_RAW_DRDY_Msk);
		assert(j30d_get_data(&h, &d) == J30D_OK);
		assert(d.raw_ch1.state == J30D_FRESH);
		assert(d.raw_ch1.raw_code == 0x010203);
		assert(d.raw_ch2.state == J30D_STALE);
		assert(d.raw_amb.state == J30D_STALE);
		printf("get_data: raw group gated per-bit, unflagged raw fields stay STALE OK\n");

		/* A ch2-only raw sample must still be read -- TO2_RAW was missing
		 * from the read condition entirely, so this used to be dropped. */
		set_raw(&ctx, 0x0A0B0C, 0x0D0E0F, 0x101112, J30D_TO2_RAW_DRDY_Msk);
		assert(j30d_get_data(&h, &d) == J30D_OK);
		assert(d.raw_ch2.state == J30D_FRESH);
		assert(d.raw_ch2.raw_code == 0x0D0E0F);
		assert(d.raw_ch1.state == J30D_STALE);
		assert(d.raw_amb.state == J30D_STALE);
		printf("get_data: TO2_RAW-only sample is read, not dropped OK\n");
	}

	/* --- get_data: DRDY set but the read of that field fails -> J30D_ERROR, value unchanged, call returns error --- */
	ctx.regs[J30D_REG_DATA_READY] |= J30D_TA_DRDY_Msk;
	ctx.fail_next_read = 1;   /* status read still consumes one "fail" slot below, so set flag after status read instead */
	/* status read must succeed, only the subsequent amb read should fail --
	 * do it manually: read status first for real by calling get_data once
	 * won't let us inject a fail mid-call with this simple harness, so
	 * instead verify the same effect using a write failure inside
	 * update_config, which the harness supports directly. */
	ctx.fail_next_read = 0;

	j30d_config_t bad = {0};
	bad.bias_cfg.status = J30D_FIELD_UPDATED;
	bad.bias_cfg.value  = 0x01;
	ctx.fail_next_write = 1;
	assert(j30d_update_config(&h, &bad) == J30D_ERR_I2C);
	/* shadow must NOT have been updated on a failed write */
	assert(h.config.bias_cfg.status != J30D_FIELD_UNCHANGED || h.config.bias_cfg.value != 0x01);
	printf("update_config write-failure leaves shadow untouched OK\n");

	/* --- chip_address is an ordinary runtime setting ---------------------
	 * Measured on hardware (J30D_LiveTest_AddressProbe): writing 0x92 moves
	 * the device immediately, no EEPROM and no reset, and 0x7F keeps
	 * answering. So it is applied at init like every other field, and the
	 * driver has no EEPROM-write path at all. */
	{
		fake_ctx_t ad_ctx = FAKE_CTX_INIT;
		j30d_platform_t ad_platform = platform;
		ad_platform.ctx = &ad_ctx;

		/* UNCHANGED must not touch 0x92 at all. Asked for explicitly rather
		 * than via NULL: j30d_default_config is the customer's file to edit,
		 * so what it holds is not this test's business. */
		{
			j30d_config_t leave = j30d_default_config;
			leave.chip_address.status = J30D_FIELD_UNCHANGED;

			j30d_handle_t ad_h;
			assert(j30d_init(&ad_h, &ad_platform, &leave) == J30D_OK);
			assert(ad_ctx.regs[J30D_REG_CHIP_ADDRESS] == 0x00);
			assert(ad_ctx.chip_addr == 0x00);
			printf("chip_address UNCHANGED leaves 0x92 alone OK\n");
		}

		/* UPDATED writes it, and the device really moves. */
		{
			j30d_config_t want = j30d_default_config;
			want.chip_address.status = J30D_FIELD_UPDATED;
			want.chip_address.value  = 0x10;

			j30d_handle_t ad_h;
			assert(j30d_init(&ad_h, &ad_platform, &want) == J30D_OK);
			assert(ad_ctx.regs[J30D_REG_CHIP_ADDRESS] == 0x10);
			assert(ad_ctx.chip_addr == 0x10);            /* moved immediately */
			assert(ad_h.config.chip_address.value == 0x10);
			/* and the rest of the config still landed, because the port is on
			 * the wildcard and that answers across the move */
			assert(ad_ctx.regs[J30D_REG_SYS_CFG1] == j30d_default_config.sys_cfg1.value);
			assert(ad_ctx.regs[J30D_REG_COMMAND]  == j30d_default_config.command.value);
			printf("chip_address UPDATED moves the device; rest of config still applies OK\n");
		}

		/* Re-applying the same address is harmless, and a different one moves
		 * it again -- there is nothing stored, so no idempotence to manage. */
		{
			j30d_config_t again = j30d_default_config;
			again.chip_address.status = J30D_FIELD_UPDATED;
			again.chip_address.value  = 0x11;

			j30d_handle_t ad_h;
			assert(j30d_init(&ad_h, &ad_platform, &again) == J30D_OK);
			assert(ad_ctx.chip_addr == 0x11);
			printf("chip_address can be changed again freely (nothing is stored) OK\n");
		}
	}

	/* --- NULL/param guards --- */
	assert(j30d_init(NULL, &platform, NULL) == J30D_ERR_PARAM);
	assert(j30d_get_data(&h, NULL) == J30D_ERR_PARAM);
	{
		j30d_handle_t fresh = {0};
		assert(j30d_get_data(&fresh, &d) == J30D_ERR_NOT_INIT);
	}
	printf("param/not-init guards OK\n");

	/* --- destructive-write guards ---------------------------------------
	 * OUT_MODE lives in SYS_CFG1 bits 4:3. Setting it to PWM/Relay stops
	 * the device responding on I2C, and because SYS_CFG1 is EEPROM-backed
	 * it stays that way across resets -- so every write path refuses it.
	 * BLOW_START and the factory calibration block are refused too. */
	{
		fake_ctx_t d_ctx = FAKE_CTX_INIT;
		j30d_platform_t d_platform = platform;
		d_platform.ctx = &d_ctx;
		j30d_handle_t d_h;
		assert(j30d_init(&d_h, &d_platform, NULL) == J30D_OK);

		const uint8_t good_cfg1 = d_ctx.regs[J30D_REG_SYS_CFG1];
		assert((good_cfg1 & J30D_SYS_CFG1_OUT_MODE_Msk) == J30D_SYS_CFG1_OUT_I2C);

		/* --- config path --- */
#if !defined(J30D_ALLOW_NON_I2C_OUTPUT)
		{
			j30d_config_t pwm = {0};
			pwm.sys_cfg1.status = J30D_FIELD_UPDATED;
			pwm.sys_cfg1.value  = (uint8_t)((good_cfg1 & (uint8_t)~J30D_SYS_CFG1_OUT_MODE_Msk) | J30D_SYS_CFG1_OUT_PWM);
			assert(j30d_update_config(&d_h, &pwm) == J30D_ERR_PARAM);
			assert(d_ctx.regs[J30D_REG_SYS_CFG1] == good_cfg1);   /* device untouched */

			j30d_config_t relay = {0};
			relay.sys_cfg1.status = J30D_FIELD_UPDATED;
			relay.sys_cfg1.value  = (uint8_t)((good_cfg1 & (uint8_t)~J30D_SYS_CFG1_OUT_MODE_Msk) | J30D_SYS_CFG1_OUT_RELAY);
			assert(j30d_update_config(&d_h, &relay) == J30D_ERR_PARAM);
			assert(d_ctx.regs[J30D_REG_SYS_CFG1] == good_cfg1);
		}
#endif

		/* Other SYS_CFG1 bits stay freely writable. */
		{
			j30d_config_t filt = {0};
			filt.sys_cfg1.status = J30D_FIELD_UPDATED;
			filt.sys_cfg1.value  = (uint8_t)(J30D_SYS_CFG1_FILT_7 | J30D_SYS_CFG1_OUT_I2C | J30D_SYS_CFG1_OSRT_1024);
			assert(j30d_update_config(&d_h, &filt) == J30D_OK);
			assert(d_ctx.regs[J30D_REG_SYS_CFG1] == filt.sys_cfg1.value);
		}

		/* --- raw path --- */
		{
			uint8_t one = 0x01;
#if !defined(J30D_ALLOW_NON_I2C_OUTPUT)
			uint8_t pwm_byte = (uint8_t)((good_cfg1 & (uint8_t)~J30D_SYS_CFG1_OUT_MODE_Msk) | J30D_SYS_CFG1_OUT_PWM);
			uint8_t before   = d_ctx.regs[J30D_REG_SYS_CFG1];
			assert(j30d_raw_write(&d_h, J30D_REG_SYS_CFG1, &pwm_byte, 1) == J30D_ERR_PARAM);
			assert(d_ctx.regs[J30D_REG_SYS_CFG1] == before);

			/* A burst that merely SPANS SYS_CFG1 is refused as well. */
			uint8_t span[3] = { 0x00, pwm_byte, 0x00 };   /* 0x92, 0x93, 0x94 */
			assert(j30d_raw_write(&d_h, J30D_REG_CHIP_ADDRESS, span, 3) == J30D_ERR_PARAM);
			assert(d_ctx.regs[J30D_REG_SYS_CFG1] == before);
#endif
#if !defined(J30D_ALLOW_EEPROM_BLOW)
			assert(j30d_raw_write(&d_h, J30D_REG_BLOW_START, &one, 1) == J30D_ERR_PARAM);
#endif
#if !defined(J30D_ALLOW_CALIBRATION_WRITE)
			assert(j30d_raw_write(&d_h, J30D_REG_CUSTOM_MEMORY_START, &one, 1) == J30D_ERR_PARAM);
			assert(j30d_raw_write(&d_h, J30D_REG_CUSTOM_MEMORY_START + 20, &one, 1) == J30D_ERR_PARAM);
			{
				uint8_t two[2] = { 0x00, 0x01 };   /* 0xBF..0xC0: last byte lands in the block */
				assert(j30d_raw_write(&d_h, (uint8_t)(J30D_REG_CUSTOM_MEMORY_START - 1), two, 2) == J30D_ERR_PARAM);
			}
#endif
			/* Just below the block, not overlapping -- must be allowed. */
			assert(j30d_raw_write(&d_h, (uint8_t)(J30D_REG_CUSTOM_MEMORY_START - 1), &one, 1) == J30D_OK);
		}

		/* The guard is one-directional: a unit ALREADY off I2C must still
		 * be recoverable by writing OUT_MODE back to I2C. */
		{
			d_ctx.regs[J30D_REG_SYS_CFG1] = (uint8_t)(good_cfg1 | J30D_SYS_CFG1_OUT_RELAY);
			assert(j30d_raw_write(&d_h, J30D_REG_SYS_CFG1, &good_cfg1, 1) == J30D_OK);
			assert(d_ctx.regs[J30D_REG_SYS_CFG1] == good_cfg1);
			assert((d_ctx.regs[J30D_REG_SYS_CFG1] & J30D_SYS_CFG1_OUT_MODE_Msk) == J30D_SYS_CFG1_OUT_I2C);
		}
		printf("destructive-write guards: OUT_MODE off-I2C / fuse / calib blocked, recovery still allowed OK\n");
	}

	/* --- j30d_raw_write() keeps h->config's shadow truthful (#9) ---------
	 * Raw access bypasses the config model, but the driver still acts on
	 * the shadow (sleep decisions, SLEEP_TIME pacing, decode_gain()). A
	 * raw write onto a modelled register must therefore be reflected there
	 * -- otherwise the automatic re-sleep step re-writes the OLD command
	 * value and silently undoes the caller's write. */
	{
		fake_ctx_t r_ctx = FAKE_CTX_INIT;
		j30d_platform_t r_platform = platform;
		r_platform.ctx = &r_ctx;
		j30d_handle_t r_h;
		assert(j30d_init(&r_h, &r_platform, NULL) == J30D_OK);
		/* Shipped default is sleep-ON, so the re-arm step is live. */
		assert(r_h.config.command.value & J30D_COMMAND_SLEEP_EN);

		/* Clear SLEEP_EN through the raw path. */
		uint8_t awake = j30d_default_config.command.value & (uint8_t)~J30D_COMMAND_SLEEP_EN;
		assert(j30d_raw_write(&r_h, J30D_REG_COMMAND, &awake, 1) == J30D_OK);

		/* The device must be left awake and STAY that way -- this is the
		 * exact case that used to be reverted on the way out of the call. */
		assert(r_ctx.regs[J30D_REG_COMMAND] == awake);
		assert((r_ctx.regs[J30D_REG_COMMAND] & J30D_COMMAND_SLEEP_EN) == 0);
		assert(r_h.config.command.value == awake);
		assert(!r_ctx.device_asleep);
		/* ...and the driver now agrees there is nothing to wake from. */
		int rec = r_ctx.return_to_iic_calls;
		uint8_t sink[1];
		assert(j30d_raw_read(&r_h, 0x50, sink, 1) == J30D_OK);
		assert(r_ctx.return_to_iic_calls == rec);
		printf("raw_write(COMMAND) is honoured, not undone by the re-sleep step OK\n");

		/* SLEEP_TIME feeds get_data()'s pacing, so it has to track too. */
		uint8_t st = 0x20;
		assert(j30d_raw_write(&r_h, J30D_REG_SLEEP_TIME, &st, 1) == J30D_OK);
		assert(r_h.config.sleep.value == 0x20);

		/* Channel config feeds decode_gain(). */
		uint8_t g = J30D_SYS_CHx_CFG_GAIN_16 | J30D_SYS_CHx_CFG_OSRP_512;
		assert(j30d_raw_write(&r_h, J30D_REG_SYS_CH1_CFG, &g, 1) == J30D_OK);
		assert(r_h.config.ch1_cfg.value == g);

		/* A burst spanning two modelled registers updates both. */
		uint8_t pair[2] = { 0xC1, 0x22 };   /* SYS_CFG1 (0x93), SYS_CFG2 (0x94) -- 0xC1 keeps OUT_MODE on I2C */
		assert(j30d_raw_write(&r_h, J30D_REG_SYS_CFG1, pair, 2) == J30D_OK);
		assert(r_h.config.sys_cfg1.value == 0xC1);
		assert(r_h.config.sys_cfg2.value == 0x22);

		/* A write to an unmodelled register must not disturb the shadow. */
		{
			j30d_config_t before = r_h.config;
			uint8_t junk = 0xEE;
			assert(j30d_raw_write(&r_h, 0x50, &junk, 1) == J30D_OK);
			assert(memcmp(&before, &r_h.config, sizeof(before)) == 0);
		}

		/* A FAILED raw write must not update the shadow either. */
		{
			uint8_t v = 0x77;
			r_ctx.fail_next_write = 1;
			assert(j30d_raw_write(&r_h, J30D_REG_BIAS_CFG, &v, 1) == J30D_ERR_I2C);
			assert(r_h.config.bias_cfg.value != 0x77);
		}
		printf("raw_write shadow sync: sleep/gain/burst tracked, unmodelled + failed writes ignored OK\n");
	}

	/* --- read-only accessors (#7) --- */
	{
		assert(j30d_is_initialized(&h));
		assert(j30d_get_calib(&h) == &h.calib);
		assert(j30d_get_config(&h) == &h.config);
		assert(j30d_get_calib(&h)->version == J30D_UNIT_VERSION_REV2PLUS);

		/* NULL handle, and a handle that never went through j30d_init() */
		assert(j30d_get_calib(NULL) == NULL);
		assert(j30d_get_config(NULL) == NULL);
		assert(!j30d_is_initialized(NULL));
		{
			j30d_handle_t raw_handle = {0};
			assert(j30d_get_calib(&raw_handle) == NULL);
			assert(j30d_get_config(&raw_handle) == NULL);
			assert(!j30d_is_initialized(&raw_handle));
		}
		printf("accessors (calib/config/is_initialized + NULL and not-init guards) OK\n");
	}

	/* --- platform.lock failure must abort the call, not proceed unlocked ---
	 * j30d_lock_fn returns bool: a lock that cannot be acquired reports
	 * false, and the driver returns J30D_ERR_TIMEOUT without touching the
	 * bus and without calling unlock (nothing was ever held). Before this,
	 * the hook returned void, so a timed-out FreeRTOS take had no way to
	 * say so and the call went through unserialized. */
	{
		fake_ctx_t l_ctx = FAKE_CTX_INIT;
		j30d_platform_t l_platform = platform;
		l_platform.ctx    = &l_ctx;
		l_platform.lock   = fake_lock;
		l_platform.unlock = fake_unlock;

		j30d_handle_t l_h;
		assert(j30d_init(&l_h, &l_platform, NULL) == J30D_OK);
		assert(l_ctx.lock_calls == 1 && l_ctx.unlock_calls == 1);   /* balanced */

		j30d_data_t ld;
		int reads_before = l_ctx.i2c_read_calls;
		int recover_before = l_ctx.return_to_iic_calls;
		l_ctx.lock_should_fail = 1;

		assert(j30d_get_data(&l_h, &ld) == J30D_ERR_TIMEOUT);
		assert(j30d_update_config(&l_h, &j30d_default_config) == J30D_ERR_TIMEOUT);
		{
			uint8_t lb[1];
			assert(j30d_raw_read(&l_h, 0x50, lb, 1) == J30D_ERR_TIMEOUT);
			assert(j30d_raw_write(&l_h, 0x50, lb, 1) == J30D_ERR_TIMEOUT);
		}
		/* Nothing reached the bus, and unlock was never called for a lock
		 * that was never taken. */
		assert(l_ctx.i2c_read_calls == reads_before);
		assert(l_ctx.return_to_iic_calls == recover_before);
		assert(l_ctx.unlock_calls == 1);   /* still just the one from init */
		printf("lock failure -> J30D_ERR_TIMEOUT, bus untouched, unlock not called OK\n");

		/* And a re-init through a failing lock must also refuse. */
		{
			j30d_handle_t l_h2;
			assert(j30d_init(&l_h2, &l_platform, NULL) == J30D_ERR_TIMEOUT);
		}

		/* Recovering the lock restores normal operation. */
		l_ctx.lock_should_fail = 0;
		assert(j30d_get_data(&l_h, &ld) == J30D_OK);
		assert(l_ctx.unlock_calls > 1);
		printf("lock recovery restores normal operation OK\n");
	}

	/* --- j30d_init() and J30D_FIELD_UNCHANGED ---------------------------
	 * COMMAND/SLEEP_TIME are RAM registers that init's own soft reset
	 * clears, so UNCHANGED is rejected for them: accepting it used to leave
	 * the shadow claiming the compiled-in default (SLEEP_EN set) while the
	 * device held its reset value, and the first get_data()'s re-arm step
	 * then wrote that default out -- putting the device to sleep although
	 * the caller explicitly asked for COMMAND not to be touched. The five
	 * EEPROM fields DO survive the reset, so UNCHANGED is honoured there
	 * and the shadow is seeded by reading the device back. */
	{
		fake_ctx_t u_ctx = FAKE_CTX_INIT;
		j30d_platform_t u_platform = platform;
		u_platform.ctx = &u_ctx;
		j30d_handle_t u_h;

		j30d_config_t all_unchanged = {0};   /* every field J30D_FIELD_UNCHANGED */
		assert(j30d_init(&u_h, &u_platform, &all_unchanged) == J30D_ERR_PARAM);
		assert(u_ctx.regs[J30D_REG_COMMAND] == 0);   /* nothing written, device never slept */
		printf("init rejects UNCHANGED for the RAM command/sleep fields OK\n");

		/* Same but with command/sleep stated explicitly: now legal, and the
		 * EEPROM fields left UNCHANGED must be seeded from the DEVICE, not
		 * from j30d_default_config. */
		fake_ctx_t e_ctx = FAKE_CTX_INIT;
		j30d_platform_t e_platform = platform;
		e_platform.ctx = &e_ctx;

		/* Pre-load the device's EEPROM registers with values that are
		 * deliberately NOT the compiled-in defaults. */
		e_ctx.regs[J30D_REG_SYS_CFG1]    = 0xA1;
		e_ctx.regs[J30D_REG_SYS_CFG2]    = 0xA2;
		e_ctx.regs[J30D_REG_SYS_CH1_CFG] = J30D_SYS_CHx_CFG_GAIN_16 | J30D_SYS_CHx_CFG_OSRP_512;
		e_ctx.regs[J30D_REG_SYS_CH2_CFG] = 0xA4;
		e_ctx.regs[J30D_REG_BIAS_CFG]    = 0xA5;

		j30d_config_t ram_only = {0};
		ram_only.command.status = J30D_FIELD_UPDATED;
		ram_only.command.value  = j30d_default_config.command.value & (uint8_t)~J30D_COMMAND_SLEEP_EN;
		ram_only.sleep.status   = J30D_FIELD_DEFAULT;

		j30d_handle_t e_h;
		assert(j30d_init(&e_h, &e_platform, &ram_only) == J30D_OK);

		/* Device untouched for the EEPROM fields... */
		assert(e_ctx.regs[J30D_REG_SYS_CFG1] == 0xA1);
		assert(e_ctx.regs[J30D_REG_BIAS_CFG] == 0xA5);
		/* ...and the shadow reflects the DEVICE, not the defaults. */
		assert(e_h.config.sys_cfg1.value == 0xA1);
		assert(e_h.config.sys_cfg2.value == 0xA2);
		assert(e_h.config.ch2_cfg.value  == 0xA4);
		assert(e_h.config.bias_cfg.value == 0xA5);
		assert(j30d_default_config.sys_cfg1.value != 0xA1);   /* sanity: those really differ */
		printf("init seeds the shadow from the device for UNCHANGED EEPROM fields OK\n");

		/* Every status ends UNCHANGED, so the read-modify-write idiom does
		 * not silently re-write untouched registers from the defaults. */
		assert(e_h.config.sys_cfg1.status == J30D_FIELD_UNCHANGED);
		assert(e_h.config.command.status  == J30D_FIELD_UNCHANGED);
		assert(e_h.config.bias_cfg.status == J30D_FIELD_UNCHANGED);

		j30d_config_t rmw = e_h.config;
		rmw.sleep.status = J30D_FIELD_UPDATED;
		rmw.sleep.value  = 0x03;
		assert(j30d_update_config(&e_h, &rmw) == J30D_OK);
		assert(e_ctx.regs[J30D_REG_SYS_CFG1] == 0xA1);   /* untouched by a sleep-only update */
		assert(e_ctx.regs[J30D_REG_BIAS_CFG] == 0xA5);
		assert(e_ctx.regs[J30D_REG_SLEEP_TIME] == 0x03);
		printf("read-modify-write of h->config touches only the edited field OK\n");

		/* The seeded ch1 gain (16, from the device) must be what raw_ch1's
		 * mV conversion uses -- the shadow is not decoration, decode_gain()
		 * reads it. */
		set_raw(&e_ctx, 0x400000, 0, 0, J30D_TO1_RAW_DRDY_Msk);
		j30d_data_t ed;
		assert(j30d_get_data(&e_h, &ed) == J30D_OK);
		assert(ed.raw_ch1.state == J30D_FRESH);
#if !defined(J30D_NO_FLOAT)
		{
			float expect = 1000.0f * ((float)0x400000 * 1.1f) / (16.0f * J30D_RAW_ADC_SCALE);
			assert(ed.raw_ch1.value > expect - 0.001f && ed.raw_ch1.value < expect + 0.001f);
		}
		printf("raw mV conversion uses the device-seeded PGA gain OK\n");
#else
		assert(ed.raw_ch1.raw_code == 0x400000);
		printf("raw sample read (NO_FLOAT: no mV conversion to check) OK\n");
#endif
	}

	/* --- j30d_raw_read / j30d_raw_write while sleep is NOT configured:
	 * no automatic return_to_iic, since there's nothing to wake from.
	 * Sleep is ON in j30d_default_config now (see j30d_config.c -- that's
	 * what calibration actually runs with), so reaching the sleep-off
	 * state to test this needs an explicit disable first; it's no longer
	 * just "whatever a plain j30d_init() already left it as". --- */
	{
		j30d_config_t sleep_off_explicit = {0};
		sleep_off_explicit.command.status = J30D_FIELD_UPDATED;
		sleep_off_explicit.command.value  = j30d_default_config.command.value & (uint8_t)~J30D_COMMAND_SLEEP_EN;
		assert(j30d_update_config(&h, &sleep_off_explicit) == J30D_OK);
	}
	assert((h.config.command.value & J30D_COMMAND_SLEEP_EN) == 0);   /* sanity: sleep explicitly disabled above */
	int recover_before = ctx.return_to_iic_calls;

	uint8_t raw_buf[2] = {0, 0};
	ctx.regs[0x50] = 0xAB;
	ctx.regs[0x51] = 0xCD;
	assert(j30d_raw_read(&h, 0x50, raw_buf, 2) == J30D_OK);
	assert(raw_buf[0] == 0xAB && raw_buf[1] == 0xCD);
	assert(ctx.return_to_iic_calls == recover_before);   /* sleep off -> no wake dance */

	uint8_t write_buf[1] = {0x42};
	assert(j30d_raw_write(&h, 0x60, write_buf, 1) == J30D_OK);
	assert(ctx.regs[0x60] == 0x42);
	assert(ctx.return_to_iic_calls == recover_before);
	printf("j30d_raw_read/j30d_raw_write (sleep off, no auto-wake) OK\n");

	/* --- now re-enable sleep via update_config (back to the shipped
	 * default state), and confirm raw_read/write (and get_data)
	 * automatically wake first and re-commit sleep after --- */
	j30d_config_t sleep_on = {0};
	sleep_on.command.status = J30D_FIELD_UPDATED;
	sleep_on.command.value  = j30d_default_config.command.value | J30D_COMMAND_SLEEP_EN;
	assert(j30d_update_config(&h, &sleep_on) == J30D_OK);
	assert(h.config.command.value & J30D_COMMAND_SLEEP_EN);
	/* update_config itself wakes (old state: off, so no recover expected)
	 * then re-commits the NEW (sleep-on) command value as its own restore
	 * step -- so COMMAND on the device must already reflect sleep-on. */
	assert(ctx.regs[J30D_REG_COMMAND] & J30D_COMMAND_SLEEP_EN);
	printf("update_config enabling sleep OK\n");

	recover_before = ctx.return_to_iic_calls;
	assert(j30d_raw_read(&h, 0x50, raw_buf, 2) == J30D_OK);
	assert(ctx.return_to_iic_calls == recover_before + 1);   /* woke for this call */
	assert(ctx.regs[J30D_REG_COMMAND] & J30D_COMMAND_SLEEP_EN);   /* re-armed after */
	printf("j30d_raw_read (sleep on) wakes + re-arms sleep OK\n");

	recover_before = ctx.return_to_iic_calls;
	assert(j30d_get_data(&h, &d) == J30D_OK);
	assert(ctx.return_to_iic_calls == recover_before + 1);
	assert(ctx.regs[J30D_REG_COMMAND] & J30D_COMMAND_SLEEP_EN);
	printf("get_data (sleep on) wakes + re-arms sleep OK\n");

	/* --- turning sleep back off via update_config must stop the auto-wake dance --- */
	j30d_config_t sleep_off = {0};
	sleep_off.command.status = J30D_FIELD_UPDATED;
	sleep_off.command.value  = j30d_default_config.command.value & (uint8_t)~J30D_COMMAND_SLEEP_EN;   /* explicit clear -- the default itself has SLEEP_EN set now */
	assert(j30d_update_config(&h, &sleep_off) == J30D_OK);
	assert((ctx.regs[J30D_REG_COMMAND] & J30D_COMMAND_SLEEP_EN) == 0);
	recover_before = ctx.return_to_iic_calls;
	assert(j30d_raw_read(&h, 0x50, raw_buf, 2) == J30D_OK);
	assert(ctx.return_to_iic_calls == recover_before);   /* sleep off again -> no wake */
	printf("update_config disabling sleep stops auto-wake OK\n");

	/* --- j30d_get_data() sleep-interval pacing (platform.now_ms provided) ---
	 * Uses its own handle/ctx so it doesn't disturb the always-wake
	 * assertions above (which deliberately run with now_ms == NULL, to
	 * prove the no-clock fallback still behaves like before). */
	{
		fake_ctx_t pace_ctx = FAKE_CTX_INIT;
		j30d_platform_t pace_platform = platform;
		pace_platform.ctx    = &pace_ctx;
		pace_platform.now_ms = fake_now_ms;

		j30d_handle_t pace_h;
		assert(j30d_init(&pace_h, &pace_platform, NULL) == J30D_OK);

		/* Sleep value 14 -> (14+1)*100 = 1500ms, matching the 1.5s example. */
		j30d_config_t sleep_1500 = {0};
		sleep_1500.command.status = J30D_FIELD_UPDATED;
		sleep_1500.command.value  = j30d_default_config.command.value | J30D_COMMAND_SLEEP_EN;
		sleep_1500.sleep.status   = J30D_FIELD_UPDATED;
		sleep_1500.sleep.value    = 14;
		assert(j30d_update_config(&pace_h, &sleep_1500) == J30D_OK);

		set_amb(&pace_ctx, 4096);
		pace_ctx.virtual_now_ms = 1000;

		j30d_data_t pd;
		int pace_recover_before = pace_ctx.return_to_iic_calls;
		int pace_reads_before   = pace_ctx.i2c_read_calls;
		assert(j30d_get_data(&pace_h, &pd) == J30D_OK);
		assert(pace_ctx.return_to_iic_calls == pace_recover_before + 1);   /* first call ever -- always wakes */
		assert(pace_ctx.i2c_read_calls > pace_reads_before);
		assert(pd.amb.state == J30D_FRESH);
		assert(pd.amb.raw_code == 4096);
		printf("get_data pacing: first call on a sleeping device wakes OK\n");

		/* Call again with no time elapsed at all: must not touch the bus,
		 * must not call return_to_iic, and must hand back the same data
		 * marked stale -- proven by both call counters staying flat, not
		 * just by the returned .state (which alone wouldn't distinguish a
		 * real re-read of unchanged registers from a genuine skip). */
		pace_recover_before = pace_ctx.return_to_iic_calls;
		pace_reads_before    = pace_ctx.i2c_read_calls;
		assert(j30d_get_data(&pace_h, &pd) == J30D_OK);
		assert(pace_ctx.return_to_iic_calls == pace_recover_before);
		assert(pace_ctx.i2c_read_calls == pace_reads_before);
		assert(pd.amb.state == J30D_STALE);
		assert(pd.amb.raw_code == 4096);   /* still the last known-good value */
		printf("get_data pacing: back-to-back call within the interval skips the bus entirely OK\n");

		/* Same again just short of the interval (999ms after the first
		 * wake, less than 1500ms) -- still must skip. */
		pace_ctx.virtual_now_ms = 1000 + 999;
		pace_recover_before = pace_ctx.return_to_iic_calls;
		assert(j30d_get_data(&pace_h, &pd) == J30D_OK);
		assert(pace_ctx.return_to_iic_calls == pace_recover_before);
		assert(pd.amb.state == J30D_STALE);
		printf("get_data pacing: call just short of the interval still skips OK\n");

		/* Advance past the interval (1000 + 1500 = 2500ms) with a new
		 * sample staged -- this call must actually wake and read again. */
		pace_ctx.virtual_now_ms = 1000 + 1500;
		set_amb(&pace_ctx, 8192);
		pace_recover_before = pace_ctx.return_to_iic_calls;
		pace_reads_before    = pace_ctx.i2c_read_calls;
		assert(j30d_get_data(&pace_h, &pd) == J30D_OK);
		assert(pace_ctx.return_to_iic_calls == pace_recover_before + 1);
		assert(pace_ctx.i2c_read_calls > pace_reads_before);
		assert(pd.amb.state == J30D_FRESH);
		assert(pd.amb.raw_code == 8192);
		printf("get_data pacing: call after the interval elapses wakes again OK\n");

		/* j30d_raw_read() must ignore all of this and always hit the bus,
		 * even called back-to-back with get_data() above (per your call on
		 * raw access always being the explicit escape hatch). */
		uint8_t pace_raw[1];
		pace_recover_before = pace_ctx.return_to_iic_calls;
		assert(j30d_raw_read(&pace_h, 0x50, pace_raw, 1) == J30D_OK);
		assert(pace_ctx.return_to_iic_calls == pace_recover_before + 1);
		printf("get_data pacing: j30d_raw_read is never paced, always wakes OK\n");
	}

	printf("ALL TESTS PASSED\n");
	return 0;
}
