/*
 * platform_stm32_baremetal.c
 *
 * Reference j30d_platform_t implementation for STM32 (STM32Cube HAL),
 * no OS / single execution context. I2C1 @ 100kHz, sensor on GPIOB6/7,
 * with no mutex and no RTOS delay -- there is nothing to serialize
 * against when only one context ever calls into the driver.
 *
 * lock/unlock are left NULL in j30d_platform_stm32_baremetal_init() --
 * that's the correct choice here, not an oversight: see j30d_platform.h.
 * If you *do* have more than one execution context calling this handle
 * (e.g. main loop + an ISR), do not reach for this file -- you need real
 * mutual exclusion, which on bare-metal means a critical section
 * (__disable_irq/__enable_irq) wired into lock/unlock, not "no locking".
 *
 * Adjust the pin macros and J30D_BM_GPIO_CLK_ENABLE() below for your board
 * if it differs from this EVK. J30D_BM_I2C_ADDR stays the 0x7F wildcard --
 * see chip_address in j30d_config.h.
 */

#include "platform_stm32_baremetal.h"
#include "main.h"      /* GPIO pin defines, HAL_GPIO_*, GPIO_TypeDef */
#include "i2c.h"       /* hi2c1, MX_I2C1_Init() */
#include "stm32l0xx_hal.h"

#include <string.h>

#define J30D_BM_I2C_ADDR      (0x7Fu << 1)   /* wildcard address 0x7F, shifted for the HAL */
#define J30D_BM_I2C_TIMEOUT   10u            /* ms, matches i2c_timeout in i2c.c */
#define J30D_BM_I2C_RETRIES   3u

/* SCL must be held low for more than 10ms to return the device to I2C idle;
 * 15ms covers that plus per-platform delay rounding. The settle delay
 * afterwards is 0, which compiles the call out -- HAL_Delay(0) still waits
 * a tick. */
#define J30D_BM_SCL_LOW_MS       15u
#define J30D_BM_SETTLE_MS        0u

#define J30D_BM_SCL_Pin        I2C_SCL_Pin
#define J30D_BM_SCL_GPIO_Port  I2C_SCL_GPIO_Port
#define J30D_BM_SDA_Pin        I2C_SDA_Pin
#define J30D_BM_SDA_GPIO_Port  I2C_SDA_GPIO_Port
#define J30D_BM_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()   /* the port SCL and SDA are on */

/* Everything this port needs, reachable from `ctx`. A bare-metal build
 * with a single J30D sensor can just point ctx at &hi2c1 directly and drop
 * this struct -- it exists mainly so this file has the same shape as the
 * FreeRTOS/Zephyr ports, and so it generalizes to more than one I2C bus
 * without change. */
typedef struct
{
	I2C_HandleTypeDef *hi2c;
} j30d_bm_ctx_t;

static j30d_err_t hal_status_to_j30d(HAL_StatusTypeDef status)
{
	switch (status)
	{
		case HAL_OK:      return J30D_OK;
		case HAL_TIMEOUT: return J30D_ERR_TIMEOUT;
		default:          return J30D_ERR_I2C;
	}
}

static j30d_err_t bm_i2c_read(void *ctx, uint8_t reg, uint8_t *buf, size_t len)
{
	j30d_bm_ctx_t *c = (j30d_bm_ctx_t *)ctx;
	HAL_StatusTypeDef status = HAL_ERROR;

	for (uint32_t attempt = 0; attempt < J30D_BM_I2C_RETRIES; ++attempt)
	{
		status = HAL_I2C_Mem_Read(c->hi2c, J30D_BM_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
		                           buf, (uint16_t)len, J30D_BM_I2C_TIMEOUT);
		if (status == HAL_OK)
		{
			break;
		}
	}
	return hal_status_to_j30d(status);
}

static j30d_err_t bm_i2c_write(void *ctx, uint8_t reg, const uint8_t *buf, size_t len)
{
	j30d_bm_ctx_t *c = (j30d_bm_ctx_t *)ctx;
	HAL_StatusTypeDef status = HAL_ERROR;

	for (uint32_t attempt = 0; attempt < J30D_BM_I2C_RETRIES; ++attempt)
	{
		status = HAL_I2C_Mem_Write(c->hi2c, J30D_BM_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
		                            (uint8_t *)buf, (uint16_t)len, J30D_BM_I2C_TIMEOUT);
		if (status == HAL_OK)
		{
			break;
		}
	}
	return hal_status_to_j30d(status);
}

static void bm_delay_ms(void *ctx, uint32_t ms)
{
	(void)ctx;
	HAL_Delay(ms);   /* SysTick-based busy-wait -- fine with no OS present */
}

static uint32_t bm_now_ms(void *ctx)
{
	(void)ctx;
	return HAL_GetTick();   /* free-running SysTick millisecond counter -- wraps every ~49.7 days, j30d.c handles that */
}

/* The GPIO bit-bang and HAL_I2C_Msp(De)Init sequence that returns the bus
 * to I2C idle, kept behind this one hook so the driver core carries no
 * platform detail. */
static void bm_return_to_iic(void *ctx)
{
	j30d_bm_ctx_t *c = (j30d_bm_ctx_t *)ctx;

	/* Release the I2C peripheral's ownership of SCL/SDA. */
	HAL_GPIO_WritePin(J30D_BM_SDA_GPIO_Port, J30D_BM_SDA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(J30D_BM_SCL_GPIO_Port, J30D_BM_SCL_Pin, GPIO_PIN_SET);
	HAL_I2C_MspDeInit(c->hi2c);

	GPIO_InitTypeDef gpio_init = {0};
	J30D_BM_GPIO_CLK_ENABLE();

	/* SDA becomes an input (device drives it during the wake handshake) */
	gpio_init.Pin  = J30D_BM_SDA_Pin;
	gpio_init.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(J30D_BM_SDA_GPIO_Port, &gpio_init);

	/* SCL is what we bit-bang to wake the device */
	gpio_init.Pin  = J30D_BM_SCL_Pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(J30D_BM_SCL_GPIO_Port, &gpio_init);

	HAL_GPIO_WritePin(J30D_BM_SCL_GPIO_Port, J30D_BM_SCL_Pin, GPIO_PIN_RESET);
	HAL_Delay(J30D_BM_SCL_LOW_MS);
	HAL_GPIO_WritePin(J30D_BM_SCL_GPIO_Port, J30D_BM_SCL_Pin, GPIO_PIN_SET);

	/* Hand the pins back to the I2C peripheral. */
	HAL_I2C_MspInit(c->hi2c);
#if J30D_BM_SETTLE_MS > 0
	HAL_Delay(J30D_BM_SETTLE_MS);
#endif
}

/* One ctx instance per physical sensor/bus. Static storage is the usual
 * bare-metal choice -- swap for a caller-supplied instance if you need
 * more than one. */
static j30d_bm_ctx_t j30d_bm_ctx_default = { .hi2c = &hi2c1 };

/*
 * Fills `platform` in ready to hand to j30d_init(). lock/unlock are left
 * NULL deliberately -- see file header comment.
 */
void j30d_platform_stm32_baremetal_init(j30d_platform_t *platform)
{
	memset(platform, 0, sizeof(*platform));
	platform->i2c_read      = bm_i2c_read;
	platform->i2c_write     = bm_i2c_write;
	platform->delay_ms      = bm_delay_ms;
	platform->return_to_iic = bm_return_to_iic;
	platform->lock          = NULL;
	platform->unlock        = NULL;
	platform->now_ms        = bm_now_ms;
	platform->ctx           = &j30d_bm_ctx_default;
}
