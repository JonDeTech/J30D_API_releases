/*
 * j30d_platform.h
 *
 * Everything OS- and hardware-specific that the driver core needs, supplied
 * by the customer as a set of function pointers. The driver core never
 * calls a vendor HAL, an RTOS API, or touches a GPIO register directly --
 * all of that lives in a small "port" file the customer writes once per
 * platform (reference ports for bare-metal/STM32 HAL, FreeRTOS and Zephyr
 * are provided separately, outside the driver core).
 *
 * Every callback receives back the `ctx` pointer stored in j30d_platform_t,
 * so one implementation can serve multiple handles/buses without any
 * global state.
 */

#ifndef J30D_PLATFORM_H_
#define J30D_PLATFORM_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "j30d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Read `len` bytes starting at device register `reg` into `buf`.
 * Return J30D_OK on success, J30D_ERR_I2C/J30D_ERR_TIMEOUT on failure. */
typedef j30d_err_t (*j30d_i2c_read_fn)(void *ctx, uint8_t reg, uint8_t *buf, size_t len);

/* Write `len` bytes from `buf` starting at device register `reg`. */
typedef j30d_err_t (*j30d_i2c_write_fn)(void *ctx, uint8_t reg, const uint8_t *buf, size_t len);

/* Blocking delay in milliseconds. Required -- the reset/wake handshakes
 * need real timing and the driver core cannot assume a particular tick
 * source (osDelay, k_msleep, a SysTick busy-wait, ...). */
typedef void (*j30d_delay_ms_fn)(void *ctx, uint32_t ms);

/* Performs the device's documented "return to IIC idle" sequence: release
 * the I2C peripheral's ownership of SCL/SDA, drive SCL low for >10ms, then
 * hand the pins back to the I2C peripheral. This is not a generic stuck-bus
 * workaround, holding SCL low for more than 10ms is
 * the ASIC's own defined path back to IIC idle from wherever it currently
 * is: out of Sleep mode (where SDA is multiplexed as the INT output), or
 * out of PWM/Relay output mode (where SDA is multiplexed as a push-pull
 * output and IIC is otherwise unusable -- see output_mode in
 * J30D_REG_SYS_CFG1). Same one sequence serves both cases. Entirely
 * platform specific:
 *   - STM32 HAL:  HAL_I2C_MspDeInit() + direct GPIO toggling + HAL_I2C_MspInit()
 *   - Zephyr:      i2c_recover_bus(), falling back to gpio_dt toggling +
 *                  i2c_configure() if the controller driver has no recovery support
 *   - bare-metal:  direct peripheral/GPIO register pokes
 *
 * This seizes the *entire* physical bus, not just this device's slice of
 * it -- it must only be called while the bus lock (below) is held, and if
 * any other I2C device shares this bus, correctness depends on that
 * device's own driver also respecting the same lock (this driver has no
 * way to enforce that on its own -- see j30d_lock_fn).
 *
 * If your port implements release_to_int, this must also disable the pin
 * interrupt before SDA becomes an I2C pin again. */
typedef void (*j30d_return_to_iic_fn)(void *ctx);

/* Optional. Serializes access to the *physical I2C bus* -- not just this
 * one handle. If multiple j30d_handle_t instances (or other I2C devices
 * you control) share a bus, all of them must be given the SAME lock/unlock
 * pair, reachable through their respective `ctx`, e.g. all pointing at one
 * shared mutex. Map to xSemaphoreTake/Give under FreeRTOS, k_mutex_lock/
 * unlock under Zephyr, or a critical-section (__disable_irq/__enable_irq)
 * if you fake concurrency with interrupts on bare-metal.
 *
 * Leave both NULL on a single-threaded bare-metal build where only one
 * execution context ever calls into this handle -- the driver then
 * performs no locking at all, which is the correct (and only sane)
 * choice in that case. */
/* lock returns true if the lock was actually acquired, false if it was not
 * (a mutex take that timed out, say). On false the driver aborts the call
 * with J30D_ERR_TIMEOUT WITHOUT touching the bus and WITHOUT calling unlock
 * -- so a lock implementation that gives up must report it rather than
 * letting the caller through, which would silently void the mutual
 * exclusion this hook exists to provide. An implementation that blocks
 * forever, or one that cannot fail, simply always returns true. */
/* Hand SDA to the device as its INT output and enable the pin interrupt.
 * Optional; called only in interrupt mode, once the sleep command has been
 * written. return_to_iic() reclaims the pins.
 *
 * Your return_to_iic() must disable the pin interrupt before SDA becomes an
 * I2C pin again, or every byte on the bus is an edge on that line.
 *
 * Interrupt mode makes the I2C bus exclusive to the J30D: while the device
 * sleeps, SDA is not an I2C pin. Do not use on a shared bus. */
typedef void (*j30d_release_to_int_fn)(void *ctx);

typedef bool (*j30d_lock_fn)(void *ctx);
typedef void (*j30d_unlock_fn)(void *ctx);

/* Optional. A free-running millisecond timestamp (HAL_GetTick(),
 * xTaskGetTickCount(), k_uptime_get_32(), ...) -- doesn't need to mean
 * anything except "time elapsed since some fixed point", and wraparound is
 * handled correctly (unsigned subtraction) as long as no single interval
 * being measured exceeds ~24 days, which sleep intervals never do.
 *
 * Only used to pace j30d_get_data() while sleep is configured: without it,
 * every j30d_get_data() call on a sleeping device pays a full wake
 * (return_to_iic(), tens of ms) even if called far more often than the
 * device's own sleep interval could possibly produce new data -- see the
 * comment on sleep_is_configured()/ensure_awake() in j30d.c. Leave NULL to
 * keep that always-wake behavior (e.g. if your platform has no cheap tick
 * source, or you're calling j30d_get_data() at a rate you already control
 * yourself). j30d_raw_read()/j30d_raw_write() never consult this -- they
 * always perform the access you asked for. */
typedef uint32_t (*j30d_now_ms_fn)(void *ctx);

typedef struct
{
	j30d_i2c_read_fn     i2c_read;      /* required */
	j30d_i2c_write_fn    i2c_write;     /* required */
	j30d_delay_ms_fn     delay_ms;      /* required */
	j30d_return_to_iic_fn return_to_iic; /* required */
	j30d_lock_fn         lock;          /* optional -- see above */
	j30d_unlock_fn       unlock;        /* optional -- see above */
	j30d_now_ms_fn       now_ms;        /* optional -- see above */
	j30d_release_to_int_fn release_to_int; /* optional -- required for interrupt mode */
	void                 *ctx;          /* passed back to every callback above */
} j30d_platform_t;

#ifdef __cplusplus
}
#endif

#endif /* J30D_PLATFORM_H_ */
