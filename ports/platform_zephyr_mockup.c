/*
 * platform_zephyr_mockup.c
 *
 * *** MOCKUP / SKETCH -- NOT COMPILED OR TESTED AGAINST REAL ZEPHYR ***
 *
 * This is here to show the *shape* a Zephyr port would take, using the
 * idioms Zephyr expects (devicetree specs, k_mutex, k_msleep, the standard
 * I2C API), so you have a starting point when you actually bring this up
 * on a Zephyr target. Node names, the devicetree binding, and pin control
 * states below are illustrative -- you'll need a real devicetree overlay
 * for your board before any of this compiles.
 *
 * Expected devicetree shape (overlay, illustrative):
 *
 *   &i2c1 {
 *       j30d0: j30d@7f {
 *           compatible = "i2c-device";
 *           reg = <0x7f>;
 *           scl-gpios = <&gpiob 6 GPIO_ACTIVE_HIGH>;
 *       };
 *   };
 *
 * Differences from the STM32 HAL ports worth calling out:
 *
 *   - return_to_iic: holds SCL low through a devicetree GPIO spec, the same
 *     way the STM32 ports do, then reconfigures the I2C controller to hand
 *     the pins back. Zephyr's i2c_recover_bus() does not do this job: it
 *     clocks SCL to free a stuck target and never holds it low for 10ms.
 *     `scl-gpios` is therefore required in your overlay, plus a pinctrl
 *     default state for the I2C node to restore.
 *
 *   - lock/unlock: k_mutex, not a counting semaphore -- fine here since
 *     j30d.c never expects recursive locking from itself, only mutual
 *     exclusion between callers.
 *
 *   - delay_ms: k_msleep() yields to the scheduler like FreeRTOS's
 *     vTaskDelay(), same reasoning as the FreeRTOS port.
 */

#include "j30d_platform.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>

#include <string.h>

/* SCL must be held low for more than 10ms to return the device to I2C idle;
 * 15ms covers that with margin. The settle delay is 0, which compiles the
 * call out -- k_msleep(0) would yield to the scheduler instead. */
#define J30D_ZEP_SCL_LOW_MS      15u
#define J30D_ZEP_SETTLE_MS       0u

/* How long zep_lock() waits for the bus mutex. A timed-out take returns false
 * and j30d.c turns that into J30D_ERR_TIMEOUT without touching the bus. */
#define J30D_ZEP_MUTEX_WAIT_MS   500u

typedef struct
{
	const struct i2c_dt_spec  i2c;          /* from DEVICE_DT_GET + reg addr via devicetree */
	const struct gpio_dt_spec scl_gpio;     /* SCL as a GPIO, for the return-to-IIC sequence */
	struct k_mutex             mutex;
} j30d_zep_ctx_t;

static j30d_err_t zep_i2c_read(void *ctx, uint8_t reg, uint8_t *buf, size_t len)
{
	j30d_zep_ctx_t *c = (j30d_zep_ctx_t *)ctx;
	int rc = i2c_write_read_dt(&c->i2c, &reg, 1, buf, len);
	if (rc == 0)       return J30D_OK;
	if (rc == -ETIMEDOUT) return J30D_ERR_TIMEOUT;
	return J30D_ERR_I2C;
}

static j30d_err_t zep_i2c_write(void *ctx, uint8_t reg, const uint8_t *buf, size_t len)
{
	j30d_zep_ctx_t *c = (j30d_zep_ctx_t *)ctx;
	uint8_t tmp[1 + 16];   /* adjust if any single register write exceeds 16 bytes */

	if (len + 1 > sizeof(tmp))
	{
		return J30D_ERR_PARAM;
	}
	tmp[0] = reg;
	memcpy(&tmp[1], buf, len);

	int rc = i2c_write_dt(&c->i2c, tmp, len + 1);
	if (rc == 0)       return J30D_OK;
	if (rc == -ETIMEDOUT) return J30D_ERR_TIMEOUT;
	return J30D_ERR_I2C;
}

static void zep_delay_ms(void *ctx, uint32_t ms)
{
	(void)ctx;
	k_msleep(ms);
}

static void zep_return_to_iic(void *ctx)
{
	j30d_zep_ctx_t *c = (j30d_zep_ctx_t *)ctx;

	/* SCL low for more than 10ms, as a GPIO. */
	gpio_pin_configure_dt(&c->scl_gpio, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&c->scl_gpio, 0);
	k_msleep(J30D_ZEP_SCL_LOW_MS);
	gpio_pin_set_dt(&c->scl_gpio, 1);

	/* Re-apply the bus's default pinctrl state / re-init the controller so
	 * it owns SCL/SDA again. i2c_configure() with the bus's existing
	 * config is a reasonable proxy for "reinit" on most controller
	 * drivers; swap for whatever your controller driver actually needs. */
	uint32_t dev_config = 0;
	if (i2c_get_config(c->i2c.bus, &dev_config) == 0)
	{
		i2c_configure(c->i2c.bus, dev_config);
	}
#if J30D_ZEP_SETTLE_MS > 0
	k_msleep(J30D_ZEP_SETTLE_MS);
#endif
}

static bool zep_lock(void *ctx)
{
	j30d_zep_ctx_t *c = (j30d_zep_ctx_t *)ctx;
	/* Non-zero means the lock was not taken (-EAGAIN on timeout); report
	 * that rather than letting the caller through unserialized. */
	return k_mutex_lock(&c->mutex, K_MSEC(J30D_ZEP_MUTEX_WAIT_MS)) == 0;
}

static void zep_unlock(void *ctx)
{
	j30d_zep_ctx_t *c = (j30d_zep_ctx_t *)ctx;
	k_mutex_unlock(&c->mutex);
}

/* One ctx instance per physical sensor/bus. Adjust the devicetree node label
 * to match your overlay, and swap for a caller-supplied instance if you need
 * more than one. */
static j30d_zep_ctx_t j30d_zep_ctx_default = {
	.i2c      = I2C_DT_SPEC_GET(DT_NODELABEL(j30d0)),
	.scl_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(j30d0), scl_gpios),
};

/*
 * Fills `platform` in ready to hand to j30d_init(). Call once at startup.
 * As with the FreeRTOS port: if other devices share this bus, they need to
 * cooperate with the same mutex for return_to_iic() to be safe, since it
 * takes SCL away from the controller.
 */
void j30d_platform_zephyr_init(j30d_platform_t *platform)
{
	k_mutex_init(&j30d_zep_ctx_default.mutex);

	memset(platform, 0, sizeof(*platform));
	platform->i2c_read      = zep_i2c_read;
	platform->i2c_write     = zep_i2c_write;
	platform->delay_ms      = zep_delay_ms;
	platform->return_to_iic = zep_return_to_iic;
	platform->lock          = zep_lock;
	platform->unlock        = zep_unlock;
	platform->ctx           = &j30d_zep_ctx_default;
}
