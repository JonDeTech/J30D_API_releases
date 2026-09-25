/*
 * j30d_live_test.c
 *
 * See j30d_live_test.h for what this is and the safety notes on which
 * registers it will and will not touch -- read those before extending
 * the fuzz sets below.
 */

#include "j30d_live_test.h"
#include "j30d.h"
#include "j30d_regs.h"
#include "platform_stm32_freertos.h"
#include "log.h"
#include "i2c.h"            /* hi2c1 -- the probe below bypasses the driver */
#include "stm32l0xx_hal.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"   /* xPortGetFreeHeapSize() -- not part of the CMSIS-RTOS2 API surface */

#include <string.h>
#include <stdbool.h>

/* ---- sequential-phase check bookkeeping --------------------------------
 * Single task, so plain (non-atomic) counters are fine here. */
static int g_seq_checks   = 0;
static int g_seq_failures = 0;

#define J30D_CHECK(cond, ...)                          \
	do {                                                \
		g_seq_checks++;                                 \
		if (!(cond)) {                                  \
			g_seq_failures++;                           \
			log_error(__VA_ARGS__);                     \
		}                                                \
	} while (0)

/* Simple deterministic PRNG (xorshift32) -- deliberately not a real RNG:
 * this test wants repeatable "garbage" byte sequences run to run, not
 * cryptographic randomness. */
static uint32_t g_rng_state = 0x1234ABCDu;
static uint8_t rng_byte(void)
{
	g_rng_state ^= g_rng_state << 13;
	g_rng_state ^= g_rng_state >> 17;
	g_rng_state ^= g_rng_state << 5;
	return (uint8_t)(g_rng_state & 0xFFu);
}


/* ---- I2C address experiment --------------------------------------------
 * Question: can the device's I2C address be changed by writing Chip_Address
 * (0x92) ALONE, with no EEPROM programming?
 *
 * It matters because the alternative -- committing to EEPROM -- is a ~250ms
 * erase/program cycle that cannot be made atomic against an MCU reset, and
 * has already destroyed the factory calibration on two units. If a plain
 * register write is enough, a customer can treat the address exactly like
 * every other setting this driver applies: written at every boot, never
 * stored. Nothing in EEPROM is ever touched, so nothing can be corrupted.
 *
 * SAFE BY CONSTRUCTION: never writes BLOW_START (j30d_raw_write refuses it
 * regardless) and never touches the calibration block. 0x92 is a shadow
 * register that reloads from EEPROM at the next reset, so a power cycle
 * undoes the entire experiment.
 *
 * Sleep is disabled for the duration -- otherwise the part stops ACKing
 * between calls and every probe below would fail for the wrong reason.
 */

/* Deliberately bypasses the driver: the port targets one fixed address, and
 * the whole question is whether some OTHER address answers. */
static bool addr_responds(uint8_t addr7)
{
	uint8_t v = 0;
	return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), J30D_REG_ERROR_CODE,
	                        I2C_MEMADD_SIZE_8BIT, &v, 1, 10) == HAL_OK;
}

static bool addr_write_byte(uint8_t addr7, uint8_t reg, uint8_t value)
{
	return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr7 << 1), reg,
	                         I2C_MEMADD_SIZE_8BIT, &value, 1, 10) == HAL_OK;
}

/* Find an address that is not the one we are already on and that nothing
 * answers. Writing the address the device already has proves nothing -- it
 * still responds afterwards either way, which is exactly how the first run
 * of this test produced a confident but meaningless "it worked". */
static bool pick_free_addr(uint8_t current, uint8_t *out)
{
	static const uint8_t candidates[] = { 0x10u, 0x11u, 0x12u, 0x20u, 0x21u, 0x22u, 0x30u, 0x31u };

	for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++)
	{
		uint8_t c = candidates[i];
		if (c == current)         { continue; }
		if (addr_responds(c))     { continue; }
		*out = c;
		return true;
	}
	return false;
}

void J30D_LiveTest_AddressProbe(void)
{
	j30d_platform_t platform;
	j30d_handle_t   h;

	log_info("=== I2C address probe: write 0x92 only, NO EEPROM programming ===");

	j30d_platform_stm32_freertos_init(&platform);
	if (j30d_init(&h, &platform, NULL) != J30D_OK)
	{
		log_error("address probe: init failed -- ABORTING");
		return;
	}

	/* Sleep off, so nothing below fails merely because the part dozed. */
	{
		j30d_config_t awake = {0};
		awake.command.status = J30D_FIELD_UPDATED;
		awake.command.value  = (uint8_t)(j30d_default_config.command.value & ~J30D_COMMAND_SLEEP_EN);
		awake.sleep.status   = J30D_FIELD_DEFAULT;
		if (j30d_update_config(&h, &awake) != J30D_OK)
		{
			log_error("address probe: could not disable sleep -- ABORTING");
			return;
		}
	}

	uint8_t original = 0xFFu;
	if (j30d_raw_read(&h, J30D_REG_CHIP_ADDRESS, &original, 1) != J30D_OK)
	{
		log_error("address probe: could not read Chip_Address -- ABORTING");
		return;
	}
	original = (uint8_t)(original & 0x7Fu);
	log_info("Chip_Address (0x92) currently reads 0x%02X", (unsigned)original);

	if (!addr_responds(0x7Fu))
	{
		log_error("address probe: 0x7F silent before any change -- ABORTING");
		return;
	}
	const bool old_responds_before = addr_responds(original);
	log_info("before: 0x7F responds=1, own addr 0x%02X responds=%d",
	         (unsigned)original, (int)old_responds_before);

	/* Every precondition below is fatal. Continuing past one is how this
	 * test previously reported success without having tested anything. */
	uint8_t target = 0u;
	if (!pick_free_addr(original, &target))
	{
		log_error("address probe: no free candidate address found -- ABORTING");
		return;
	}
	log_info("target: 0x%02X (free, and different from the current address)", (unsigned)target);

	/* The experiment: a plain register write, nothing more. */
	if (j30d_raw_write(&h, J30D_REG_CHIP_ADDRESS, &target, 1) != J30D_OK)
	{
		log_error("address probe: write to 0x92 failed -- ABORTING");
		return;
	}

	uint8_t readback = 0xFFu;
	(void)j30d_raw_read(&h, J30D_REG_CHIP_ADDRESS, &readback, 1);
	log_info("after write: 0x92 reads 0x%02X (wrote 0x%02X)",
	         (unsigned)(readback & 0x7Fu), (unsigned)target);

	const bool target_after   = addr_responds(target);
	const bool old_after      = addr_responds(original);
	const bool wildcard_after = addr_responds(0x7Fu);
	log_info("after:  target 0x%02X responds=%d, old 0x%02X responds=%d, 0x7F responds=%d",
	         (unsigned)target, (int)target_after,
	         (unsigned)original, (int)old_after, (int)wildcard_after);

	/* Moving means the new address answers AND the old one stops. If both
	 * answer, the write did not take effect and the device simply still has
	 * its old address (0x7F aside -- the wildcard always answers). */
	if (target_after && (!old_after || original == 0x7Fu))
	{
		log_info("VERDICT: the address MOVED without programming EEPROM.");
		log_info("         Treat it like any other setting: write 0x92 at every boot.");
		log_info("         Nothing is stored, so nothing can be corrupted.");
		log_info("         0x7F %s answer alongside it.", wildcard_after ? "DOES still" : "does NOT");
	}
	else if (target_after && old_after)
	{
		log_info("VERDICT: INCONCLUSIVE -- both 0x%02X and 0x%02X answer.",
		         (unsigned)target, (unsigned)original);
		log_info("         The device did not move; something else is at 0x%02X.",
		         (unsigned)target);
	}
	else
	{
		log_info("VERDICT: writing 0x92 did NOT move the device (0x%02X silent).",
		         (unsigned)target);
		log_info("         Chip_Address is latched at EEPROM load, like output_mode.");
		log_info("         It can then only be set by programming EEPROM and resetting,");
		log_info("         i.e. at calibration time by JDT tooling -- not in firmware.");
	}

	/* Put it back, via whichever address is answering. The write is volatile
	 * anyway, but do not leave the bench in an odd state. */
	{
		bool restored = false;
		if (wildcard_after) { restored = addr_write_byte(0x7Fu, J30D_REG_CHIP_ADDRESS, original); }
		if (!restored && target_after)
		{
			restored = addr_write_byte(target, J30D_REG_CHIP_ADDRESS, original);
		}
		if (!restored)
		{
			log_error("address probe: could not restore 0x92 to 0x%02X", (unsigned)original);
		}
	}

	/* Back to the shipped defaults, sleep included. */
	(void)j30d_init(&h, &platform, NULL);
	log_info("=== I2C address probe done ===");
}


/* ---- phase 1: sequential functional pass -------------------------------
 * Exercises every j30d_* function once each, including the parameter-
 * guard and DEFAULT-resolution paths the concurrency phase below doesn't
 * bother with. Single task, easy to reason about, runs first so a basic
 * driver failure shows up before the noisier concurrent phase. */
static void run_functional_pass(j30d_handle_t *h, const j30d_platform_t *platform)
{
	log_info("--- phase 1: sequential functional pass ---");

	J30D_CHECK(j30d_init(h, platform, NULL) == J30D_OK, "init (defaults) failed");
	J30D_CHECK(h->initialized, "handle not marked initialized after init");

	const char *ver = j30d_get_version_string();
	J30D_CHECK(ver != NULL && ver[0] != '\0', "version string empty/NULL");
	log_info("driver version: %s", ver ? ver : "(null)");

	log_info("calibration: valid=%d version=%d revision=%u uid=0x%08lX lot=%u wafer=%u",
	         (int)h->calib.valid, (int)h->calib.version, (unsigned)h->calib.revision,
	         (unsigned long)h->calib.uid, (unsigned)h->calib.lot_no, (unsigned)h->calib.wafer_no);

	/* update_config: every field explicitly DEFAULT (not UNCHANGED) --
	 * exercises the DEFAULT-resolution path apply_field() takes, which the
	 * "just change one field" calls elsewhere never touch. */
	{
		j30d_config_t all_default = {0};
		all_default.command.status  = J30D_FIELD_DEFAULT;
		all_default.sleep.status    = J30D_FIELD_DEFAULT;
		all_default.sys_cfg1.status = J30D_FIELD_DEFAULT;
		all_default.sys_cfg2.status = J30D_FIELD_DEFAULT;
		all_default.ch1_cfg.status  = J30D_FIELD_DEFAULT;
		all_default.ch2_cfg.status  = J30D_FIELD_DEFAULT;
		all_default.bias_cfg.status = J30D_FIELD_DEFAULT;
		J30D_CHECK(j30d_update_config(h, &all_default) == J30D_OK, "update_config (all DEFAULT) failed");
		J30D_CHECK(h->config.sys_cfg1.value == j30d_default_config.sys_cfg1.value, "sys_cfg1 not resolved to default");
	}

	/* update_config: partial, single field */
	{
		j30d_config_t just_ch1 = {0};
		just_ch1.ch1_cfg.status = J30D_FIELD_UPDATED;
		just_ch1.ch1_cfg.value  = j30d_default_config.ch1_cfg.value;   /* same value, still exercises the write path */
		J30D_CHECK(j30d_update_config(h, &just_ch1) == J30D_OK, "update_config (partial) failed");
	}

	/* get_data: every call now refreshes all six fields at once (see its
	 * doc comment in j30d.h -- there's no per-group select anymore). Real
	 * sensor needs a few sample cycles to have everything ready, and
	 * different fields can become fresh on different cycles, so poll for
	 * up to ~2s tracking each field independently rather than stopping as
	 * soon as any one of them goes fresh. */
	{
		bool saw_ch1 = false, saw_ch2 = false, saw_amb = false;
		bool saw_raw_ch1 = false, saw_raw_ch2 = false, saw_raw_amb = false;
		j30d_data_t d;

		for (int attempt = 0; attempt < 40; attempt++)
		{
			J30D_CHECK(j30d_get_data(h, &d) == J30D_OK, "get_data returned error (attempt %d)", attempt);
			saw_ch1     |= (d.ch1.state == J30D_FRESH);
			saw_ch2     |= (d.ch2.state == J30D_FRESH);
			saw_amb     |= (d.amb.state == J30D_FRESH);
			saw_raw_ch1 |= (d.raw_ch1.state == J30D_FRESH);
			saw_raw_ch2 |= (d.raw_ch2.state == J30D_FRESH);
			saw_raw_amb |= (d.raw_amb.state == J30D_FRESH);
			if (saw_ch1 && saw_ch2 && saw_amb && saw_raw_ch1 && saw_raw_ch2 && saw_raw_amb) { break; }
			osDelay(50);
		}
		/* Each field is now gated on its OWN DRDY bit, so which fields go
		 * FRESH depends on the conversion mode the device is actually in.
		 * This runs under j30d_default_config, i.e.
		 * J30D_COMMAND_MODE_SEL_SINGLE -- so only the converting channel
		 * is flagged, and channel 2 legitimately may never go FRESH.
		 *
		 * Do NOT "fix" a ch2-never-fresh report by asserting it here: this
		 * check used to pass unconditionally because get_data() marked ch2
		 * fresh off channel 1's DRDY bit, reporting whatever bytes were
		 * sitting in the ch2 registers as a new sample. Channel 2 is
		 * logged rather than asserted; if you need it asserted, switch the
		 * device to J30D_COMMAND_MODE_SEL_DUAL first and assert it there. */
		J30D_CHECK(saw_ch1,     "ch1 never went FRESH after ~2s -- sensor not sampling?");
		J30D_CHECK(saw_amb,     "amb never went FRESH after ~2s -- sensor not sampling?");
		J30D_CHECK(saw_raw_ch1, "raw_ch1 never went FRESH after ~2s -- sensor not sampling?");
		J30D_CHECK(saw_raw_amb, "raw_amb never went FRESH after ~2s -- sensor not sampling?");
		log_info("channel 2 freshness in single-conversion mode: ch2=%d raw_ch2=%d "
		         "(0 is expected here -- the device only flags the converting channel)",
		         (int)saw_ch2, (int)saw_raw_ch2);
	}

	/* raw_read: ID0/ID1 -- not a chip identifier despite the register
	 * names. This is free EEPROM scratch space JDT or the customer can
	 * write whatever they want into; it reads back 0x00/0x00 on every
	 * unit we use, and that's expected, not a fault. No fixed value
	 * asserted here -- this just proves raw_read works and logs what's
	 * actually on this board. */
	{
		uint8_t id0 = 0, id1 = 0;
		J30D_CHECK(j30d_raw_read(h, J30D_REG_ID0, &id0, 1) == J30D_OK, "raw_read ID0 failed");
		J30D_CHECK(j30d_raw_read(h, J30D_REG_ID1, &id1, 1) == J30D_OK, "raw_read ID1 failed");
		log_info("EEPROM scratch ID0=0x%02X ID1=0x%02X (OEM-writable, 0x00/0x00 is normal/expected)", id0, id1);
	}

	/* raw_write/raw_read round-trip on SLEEP_TIME -- a plain RAM register
	 * j30d_default_config already restores below, safe to scribble. */
	{
		uint8_t written = 0x2Au;
		uint8_t readback = 0;
		J30D_CHECK(j30d_raw_write(h, J30D_REG_SLEEP_TIME, &written, 1) == J30D_OK, "raw_write SLEEP_TIME failed");
		J30D_CHECK(j30d_raw_read(h, J30D_REG_SLEEP_TIME, &readback, 1) == J30D_OK, "raw_read SLEEP_TIME failed");
		J30D_CHECK(readback == written, "raw_write/raw_read SLEEP_TIME round-trip mismatch (wrote 0x%02X, read 0x%02X)", written, readback);
	}

	/* Single-shot sleep toggle (the rapid-fire version is phase 2's job). */
	{
		j30d_config_t sleep_on = {0};
		sleep_on.command.status = J30D_FIELD_UPDATED;
		sleep_on.command.value  = j30d_default_config.command.value | J30D_COMMAND_SLEEP_EN;
		sleep_on.sleep.status   = J30D_FIELD_UPDATED;
		sleep_on.sleep.value    = 2;   /* (2+1)*100 = 300ms */
		J30D_CHECK(j30d_update_config(h, &sleep_on) == J30D_OK, "update_config (enable sleep) failed");

		uint8_t scratch;
		J30D_CHECK(j30d_raw_read(h, J30D_REG_ID0, &scratch, 1) == J30D_OK, "raw_read while sleep-configured failed to wake+read");

		/* j30d_default_config.command has SLEEP_EN set (that's the shipped
		 * default -- matches what calibration runs with), so "disable
		 * sleep" has to explicitly clear the bit, not just re-apply the
		 * default. */
		j30d_config_t sleep_off = {0};
		sleep_off.command.status = J30D_FIELD_UPDATED;
		sleep_off.command.value  = j30d_default_config.command.value & ~J30D_COMMAND_SLEEP_EN;
		J30D_CHECK(j30d_update_config(h, &sleep_off) == J30D_OK, "update_config (disable sleep) failed");
	}

	/* param guards -- must not crash, must report J30D_ERR_PARAM */
	{
		j30d_data_t d;
		J30D_CHECK(j30d_get_data(h, NULL) == J30D_ERR_PARAM, "get_data(NULL out) did not return ERR_PARAM");
		J30D_CHECK(j30d_raw_read(h, J30D_REG_ID0, NULL, 1) == J30D_ERR_PARAM, "raw_read(NULL buf) did not return ERR_PARAM");
		J30D_CHECK(j30d_raw_write(h, J30D_REG_SLEEP_TIME, NULL, 1) == J30D_ERR_PARAM, "raw_write(NULL buf) did not return ERR_PARAM");
		(void)d;
	}

	/* NOTE: there used to be a "garbage config barrage" here that threw
	 * pseudo-random bytes at sys_cfg1/sys_cfg2/ch1_cfg/ch2_cfg/bias_cfg.
	 * Removed -- sys_cfg1 (0x93) includes OUT_MODE (J30D_SYS_CFG1_OUT_MODE_*),
	 * which selects the device's communication interface (I2C / PWM /
	 * relay). A random byte there has a real chance of switching the
	 * device off I2C entirely, which of course then looks exactly like a
	 * wedged bus and doesn't recover until something puts a valid I2C
	 * selection back -- not a driver robustness finding, just an unsafe
	 * test. Not adding it back: we're testing the driver's functionality,
	 * not defending against a register write no sane caller would make.
	 */

	log_info("phase 1 done: %d checks, %d failed", g_seq_checks, g_seq_failures);
}

/* ---- phase 2: concurrent abuse ------------------------------------------
 * Three tasks hammering the SAME handle at once for J30D_ABUSE_MS. The
 * actual point of this phase: platform.lock/unlock is what's supposed to
 * make this safe, not careful call sequencing -- so don't be careful here.
 * Each worker keeps its own local counters and logs+self-deletes when its
 * time is up; the caller just waits out the duration (no synchronization
 * primitive needed since nothing here depends on a specific interleaving).
 */

#define J30D_ABUSE_MS       5000u
#define J30D_ABUSE_MARGIN_MS 500u   /* headroom past J30D_ABUSE_MS before the caller moves on */

typedef struct
{
	j30d_handle_t *h;
	uint32_t        start_tick;
} abuse_ctx_t;

static bool abuse_time_left(const abuse_ctx_t *ctx)
{
	return (osKernelGetTickCount() - ctx->start_tick) < J30D_ABUSE_MS;
}

static void abuse_get_data_task(void *arg)
{
	abuse_ctx_t *ctx = (abuse_ctx_t *)arg;
	int calls = 0, errors = 0;

	while (abuse_time_left(ctx))
	{
		j30d_data_t d;
		if (j30d_get_data(ctx->h, &d) != J30D_OK) { errors++; }
		calls++;
		osDelay(1);
	}

	log_info("abuse worker [get_data]: %d calls, %d errors", calls, errors);
	osThreadExit();
}

static void abuse_config_toggle_task(void *arg)
{
	abuse_ctx_t *ctx = (abuse_ctx_t *)arg;
	int toggles = 0, errors = 0;
	bool sleep_on = false;

	/* j30d_default_config.command has SLEEP_EN set (the shipped default),
	 * so toggling has to explicitly clear the bit on the "off" half of
	 * the cycle -- ORing it in on one side and using the bare default on
	 * the other only worked back when the default itself was sleep-off;
	 * with sleep-on as the default that "toggle" stopped toggling at all
	 * (both branches came out identical), silently turning this lane into
	 * a no-op. */
	while (abuse_time_left(ctx))
	{
		j30d_config_t cfg = {0};
		cfg.command.status = J30D_FIELD_UPDATED;
		cfg.command.value  = sleep_on ? (j30d_default_config.command.value | J30D_COMMAND_SLEEP_EN)
		                               : (j30d_default_config.command.value & ~J30D_COMMAND_SLEEP_EN);
		if (j30d_update_config(ctx->h, &cfg) != J30D_OK) { errors++; }
		sleep_on = !sleep_on;
		toggles++;
		osDelay(30);
	}

	/* Leave the device in its real shipped state (sleep ON) -- the
	 * recovery phase's own j30d_init() will restore full defaults
	 * regardless, but no reason to hand it off in the non-default state. */
	{
		j30d_config_t restore_default = {0};
		restore_default.command.status = J30D_FIELD_UPDATED;
		restore_default.command.value  = j30d_default_config.command.value;
		(void)j30d_update_config(ctx->h, &restore_default);
	}

	log_info("abuse worker [config toggle]: %d toggles, %d errors", toggles, errors);
	osThreadExit();
}

static void abuse_raw_access_task(void *arg)
{
	abuse_ctx_t *ctx = (abuse_ctx_t *)arg;
	int ops = 0, errors = 0;

	while (abuse_time_left(ctx))
	{
		/* Round-trip on SLEEP_TIME (safe, RAM, restored by the final
		 * defaults reload) -- proves raw_write/raw_read survive being
		 * called from a task that's also fighting for the bus with the
		 * other two workers. */
		uint8_t v = rng_byte();
		uint8_t back = 0;
		if (j30d_raw_write(ctx->h, J30D_REG_SLEEP_TIME, &v, 1) != J30D_OK) { errors++; }
		if (j30d_raw_read(ctx->h, J30D_REG_SLEEP_TIME, &back, 1) != J30D_OK) { errors++; }
		ops++;

		/* Read-only spot checks elsewhere in the map -- never written,
		 * see j30d_live_test.h for why. */
		uint8_t scratch;
		(void)j30d_raw_read(ctx->h, J30D_REG_ID0, &scratch, 1);
		(void)j30d_raw_read(ctx->h, J30D_REG_BIAS_CFG, &scratch, 1);
		ops++;

		osDelay(5);
	}

	log_info("abuse worker [raw access]: %d ops, %d errors", ops, errors);
	osThreadExit();
}

static void run_concurrent_abuse(j30d_handle_t *h)
{
	log_info("--- phase 2: concurrent abuse (%lu ms, 3 tasks on one handle) ---", (unsigned long)J30D_ABUSE_MS);

	static abuse_ctx_t ctx;
	ctx.h = h;
	ctx.start_tick = osKernelGetTickCount();

	/* 192*4 (768 bytes) each, not 256*4 -- measured on real hardware, 256*4
	 * (1024 bytes) per task left only ~3.2KB free right before this call
	 * (see the free-heap log line below), and three tasks at 1024B plus
	 * FreeRTOS's own per-task TCB/heap_4 bookkeeping (roughly another
	 * ~100 bytes per task) don't fit in that -- the third osThreadNew()
	 * silently failed. 768 bytes still comfortably covers what these
	 * tasks actually do (one j30d_* call, osDelay(), and a log_info() on
	 * exit -- t1/t2 above already ran a full 5s at 1024B without ever
	 * getting close to that budget), while leaving real headroom instead
	 * of the ~100 bytes 1024B left for the third task. */
	const osThreadAttr_t get_data_attr   = { .name = "j30dAbuseGet",  .stack_size = 192 * 4, .priority = (osPriority_t)osPriorityNormal };
	const osThreadAttr_t config_attr     = { .name = "j30dAbuseCfg",  .stack_size = 192 * 4, .priority = (osPriority_t)osPriorityNormal };
	const osThreadAttr_t raw_attr        = { .name = "j30dAbuseRaw",  .stack_size = 192 * 4, .priority = (osPriority_t)osPriorityNormal };

	log_info("free heap before abuse tasks: %u bytes", (unsigned)xPortGetFreeHeapSize());

	osThreadId_t t1 = osThreadNew(abuse_get_data_task,     &ctx, &get_data_attr);
	osThreadId_t t2 = osThreadNew(abuse_config_toggle_task, &ctx, &config_attr);
	osThreadId_t t3 = osThreadNew(abuse_raw_access_task,    &ctx, &raw_attr);

	/* This used to just log an error and carry on if a task failed to
	 * spawn, which is how the previous run silently downgraded to a
	 * 2-lane abuse test and still reported an overall PASS -- the whole
	 * point of phase 2 is exercising platform.lock/unlock with all three
	 * lanes fighting over the bus at once, so losing a lane isn't a
	 * degraded-but-OK result, it's this phase not testing what it claims
	 * to. Make it a hard failure instead. */
	J30D_CHECK(t1 && t2 && t3, "run_concurrent_abuse: osThreadNew failed (t1=%p t2=%p t3=%p) -- out of heap/stack budget? free heap was logged above",
	           (void*)t1, (void*)t2, (void*)t3);

	osDelay(J30D_ABUSE_MS + J30D_ABUSE_MARGIN_MS);
	log_info("phase 2 done (workers self-delete on their own timers -- see their log lines above for tallies)");
}

/* ---- phase 3: recovery check --------------------------------------------
 * Whatever phases 1-2 did to the device/shadow config, a full j30d_init()
 * must always bring both back to a clean, working state. This is the
 * actual "can we recover" answer -- everything before it is just trying
 * to make this phase's job harder. */
static void run_recovery_check(j30d_handle_t *h, const j30d_platform_t *platform)
{
	log_info("--- phase 3: recovery check ---");

	J30D_CHECK(j30d_init(h, platform, NULL) == J30D_OK, "recovery: j30d_init failed");
	J30D_CHECK(h->initialized, "recovery: handle not initialized after re-init");

	bool saw_fresh = false;
	for (int attempt = 0; attempt < 30 && !saw_fresh; attempt++)
	{
		j30d_data_t d;
		if (j30d_get_data(h, &d) == J30D_OK)
		{
			saw_fresh = (d.ch1.state == J30D_FRESH) || (d.ch2.state == J30D_FRESH) || (d.amb.state == J30D_FRESH);
		}
		if (!saw_fresh) { osDelay(50); }
	}
	J30D_CHECK(saw_fresh, "recovery: no FRESH reading within ~1.5s after re-init -- device did not come back cleanly");

	if (saw_fresh)
	{
		log_info("recovery: OK -- device producing fresh data again after full abuse");
	}
	else
	{
		log_error("recovery: FAILED -- device unresponsive or stuck after abuse, even after a full re-init");
	}
}

void J30D_LiveTest_Run(void)
{
	g_seq_checks = 0;
	g_seq_failures = 0;

	log_info("==================================================");
	log_info("J30D live test starting");
	log_info("==================================================");

	j30d_platform_t platform;
	j30d_platform_stm32_freertos_init(&platform);

	j30d_handle_t h;

	run_functional_pass(&h, &platform);
	run_concurrent_abuse(&h);
	run_recovery_check(&h, &platform);

	log_info("==================================================");
	log_info("J30D live test finished: %d sequential checks, %d failed", g_seq_checks, g_seq_failures);
	log_info("(concurrent-phase worker tallies are in their own log lines above -- not folded into this count)");
	log_info("%s", (g_seq_failures == 0) ? "RESULT: PASS" : "RESULT: FAIL");
	log_info("==================================================");
}
