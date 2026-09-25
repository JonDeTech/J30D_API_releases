/*
 * j30d_live_test.h
 *
 * On-target stress/recovery test for the J30D driver, running on real
 * hardware through the FreeRTOS STM32 port. NOT part of the portable
 * j30d_api -- this is a development/debug tool specific to new_evk_sw,
 * same category as j30d_adapter.c.
 *
 * Unlike test/fake_sensor_test.c (host-side, gcc, a fake in-memory sensor,
 * excluded from the embedded build -- see .cproject), this file compiles
 * INTO the firmware and talks to a real J30D over real I2C. It exercises
 * every j30d_* function, hammers config/sleep changes and raw register
 * access from several FreeRTOS tasks at once (the actual point: proving
 * the platform.lock/unlock design holds up under real concurrent access,
 * not just the fake harness's single-threaded call sequences), and
 * finishes with a full re-init to confirm the device and driver recover
 * to a clean working state no matter what was thrown at them.
 *
 * Safety: this test deliberately never touches J30D_REG_BLOW_START (blows
 * an EEPROM fuse -- production use only, irreversible) or the factory
 * calibration block at J30D_REG_CUSTOM_MEMORY_START..+48 (would corrupt
 * this specific unit's calibration data permanently). Every register this
 * test writes to is one j30d_init() already rewrites from
 * j30d_default_config, or one this file explicitly restores afterward --
 * see the comments in j30d_live_test.c before extending the fuzz set.
 *
 * Also never write raw/random bytes to sys_cfg1 (0x93), sys_cfg2 (0x94),
 * ch1_cfg/ch2_cfg (0x95/0x96), or bias_cfg (0x97). sys_cfg1 in particular
 * includes OUT_MODE, which selects the device's communication interface
 * (I2C, PWM, or relay) -- a random byte there can switch the device off
 * I2C entirely, which then looks exactly like a wedged bus. Worse,
 * sys_cfg1 is EEPROM-backed, so it survives power cycles: the unit comes
 * back up off I2C every time. An earlier version of this test fuzzed these
 * registers with random bytes and did exactly that -- that's testing
 * whether a caller can misconfigure the interface off itself, not the
 * driver's functionality, so it's gone rather than fixed.
 *
 * As of driver 0.4.0 this is enforced rather than merely requested: the
 * driver refuses any write that would move OUT_MODE off I2C (and any write
 * to BLOW_START or the calibration block) with J30D_ERR_PARAM, on both the
 * config and the raw path. Writing OUT_MODE back TO I2C is still allowed,
 * so a unit already in that state can be recovered. Keep this test away
 * from those registers anyway -- relying on the guard to catch a fuzz case
 * is not the same as not fuzzing it.
 *
 * NOT wired into the normal boot/scheduler path -- nothing calls
 * J30D_LiveTest_Run() automatically. Call it yourself (temporarily from a
 * task after osKernelStart(), or bind it to a spare debug UART command)
 * when you want to run it. It uses its OWN j30d_handle_t, independent of
 * the adapter's g_j30d_handle -- but it shares the same physical sensor
 * and the same underlying I2C bus mutex (platform_stm32_freertos_init()
 * always hands back the same static mutex/hi2c1), so after running this,
 * the ADAPTER's cached config shadow (g_j30d_handle.config) is stale --
 * this test's own j30d_init() calls reset the device out from under it.
 * Either reboot, or call J30D_Adapter_Setup() again, before trusting the
 * normal telemetry pipeline afterward.
 *
 * Results go to the existing log_info/log_error (log.h) -- watch your
 * usual debug UART/log sink while it runs. Takes several seconds
 * (dominated by the concurrency-abuse phase's fixed duration).
 */

#ifndef SENSORS_J30D_LIVE_TEST_H_
#define SENSORS_J30D_LIVE_TEST_H_

/* Runs the full test. Blocks the calling task for several seconds. Must be
 * called after osKernelStart() (it uses the FreeRTOS platform port --
 * vTaskDelay, mutexes, osThreadNew -- none of which are valid before the
 * scheduler is running; that's a different constraint than
 * J30D_Adapter_WakeForProbe()'s bare-metal-port pre-scheduler use). */
void J30D_LiveTest_Run(void);

/* Standalone experiment, NOT part of J30D_LiveTest_Run(): does writing
 * Chip_Address (0x92) on its own move the device to a new I2C address, with
 * no EEPROM programming?
 *
 * Safe on any unit. It never writes BLOW_START and never touches the
 * calibration block; 0x92 is a shadow register that reloads from EEPROM at
 * the next reset, so a power cycle undoes everything it does. That is the
 * point -- the EEPROM-programming alternative is a ~250ms cycle that cannot
 * be made atomic against an MCU reset and has already destroyed the
 * calibration on two units.
 *
 * Logs a VERDICT line. If the address moves, a customer can write 0x92 at
 * every boot like any other setting and never touch EEPROM at all. If it
 * does not, the address is latched at EEPROM load and can only be set by
 * JDT calibration tooling. */
void J30D_LiveTest_AddressProbe(void);

#endif /* SENSORS_J30D_LIVE_TEST_H_ */
