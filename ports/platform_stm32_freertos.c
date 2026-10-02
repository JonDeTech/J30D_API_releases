/*
 * platform_stm32_freertos.c
 *
 * Reference j30d_platform_t implementation for STM32 (STM32Cube HAL) under
 * FreeRTOS. Same HAL calls as platform_stm32_baremetal.c, but:
 *   - delay_ms uses vTaskDelay() instead of a busy-wait, so other tasks run
 *     during the (several 10s of ms) delays in j30d_init()'s handshake.
 *   - lock/unlock wrap a FreeRTOS mutex, so two tasks calling into the SAME
 *     j30d_handle_t are serialized instead of corrupting each other.
 *
 * IMPORTANT: i2c_read/i2c_write below do NOT take the mutex themselves.
 * The mutex is taken once by j30d.c, around the *entire* body of
 * j30d_init()/j30d_update_config()/j30d_get_data() (which may issue several
 * i2c_read/i2c_write/return_to_iic calls each) -- not per HAL transaction.
 * If you add your own locking inside i2c_read/i2c_write on top of this,
 * with a non-recursive mutex, you will deadlock.
 *
 * If other, non-J30D devices share this I2C bus, `mutex` below must be the
 * SAME mutex those drivers use to guard the bus -- not a private one
 * scoped to just this sensor. See j30d_platform.h / the return_to_iic
 * discussion for why: return_to_iic() reconfigures the I2C peripheral's
 * pins out from under the whole bus, and only cooperating callers of the
 * same mutex are protected against that.
 */

#include "platform_stm32_freertos.h"
#include "main.h"
#include "i2c.h"
#include "stm32l0xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

#define J30D_FR_I2C_ADDR      (0x7Fu << 1)
#define J30D_FR_I2C_TIMEOUT   10u
#define J30D_FR_I2C_RETRIES   3u
#define J30D_FR_MUTEX_WAIT_MS 500u

/* SCL must be held low for more than 10ms to return the device to I2C idle;
 * 15ms covers that plus per-platform delay rounding. The settle delay
 * afterwards is 0, which compiles the call out -- passing 0 to the delay
 * function would yield to the scheduler instead. */
#define J30D_FR_SCL_LOW_MS       15u
/* 0 means no delay at all -- the call below is compiled out. Do not simply
 * pass 0: vTaskDelay(0) is a yield, and HAL_Delay(0) still waits a tick. */
#define J30D_FR_SETTLE_MS        0u

/* Interrupt mode: SDA is the device's INT output while it sleeps. Pins 4-15
 * map to EXTI4_15 on STM32L0; change this if SDA is on a lower pin. Your
 * handler must clear the pending bit, call j30d_notify_interrupt(), and
 * signal a task to do the read. See j30d_platform.h. */
#define J30D_FR_INT_IRQn       EXTI4_15_IRQn
/* M0+ has 2 priority bits, so 0-3, and no BASEPRI: any priority is safe for
 * ...FromISR. On M3/M4/M7 this must be >= configMAX_SYSCALL_INTERRUPT_PRIORITY. */
#define J30D_FR_INT_PRIO       3u

#define J30D_FR_SCL_Pin        I2C_SCL_Pin
#define J30D_FR_SCL_GPIO_Port  I2C_SCL_GPIO_Port
#define J30D_FR_SDA_Pin        I2C_SDA_Pin
#define J30D_FR_SDA_GPIO_Port  I2C_SDA_GPIO_Port

typedef struct
{
	I2C_HandleTypeDef *hi2c;
	SemaphoreHandle_t   mutex;   /* shared with any other device on this bus */
} j30d_fr_ctx_t;

static j30d_err_t hal_status_to_j30d(HAL_StatusTypeDef status)
{
	switch (status)
	{
		case HAL_OK:      return J30D_OK;
		case HAL_TIMEOUT: return J30D_ERR_TIMEOUT;
		default:          return J30D_ERR_I2C;
	}
}

static j30d_err_t fr_i2c_read(void *ctx, uint8_t reg, uint8_t *buf, size_t len)
{
	j30d_fr_ctx_t *c = (j30d_fr_ctx_t *)ctx;
	HAL_StatusTypeDef status = HAL_ERROR;

	for (uint32_t attempt = 0; attempt < J30D_FR_I2C_RETRIES; ++attempt)
	{
		status = HAL_I2C_Mem_Read(c->hi2c, J30D_FR_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
		                           buf, (uint16_t)len, J30D_FR_I2C_TIMEOUT);
		if (status == HAL_OK)
		{
			break;
		}
	}
	return hal_status_to_j30d(status);
}

static j30d_err_t fr_i2c_write(void *ctx, uint8_t reg, const uint8_t *buf, size_t len)
{
	j30d_fr_ctx_t *c = (j30d_fr_ctx_t *)ctx;
	HAL_StatusTypeDef status = HAL_ERROR;

	for (uint32_t attempt = 0; attempt < J30D_FR_I2C_RETRIES; ++attempt)
	{
		status = HAL_I2C_Mem_Write(c->hi2c, J30D_FR_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
		                            (uint8_t *)buf, (uint16_t)len, J30D_FR_I2C_TIMEOUT);
		if (status == HAL_OK)
		{
			break;
		}
	}
	return hal_status_to_j30d(status);
}

static void fr_delay_ms(void *ctx, uint32_t ms)
{
	(void)ctx;
	vTaskDelay(pdMS_TO_TICKS(ms));
}

static uint32_t fr_now_ms(void *ctx)
{
	(void)ctx;
	/* xTaskGetTickCount() is in RTOS ticks, not ms -- portTICK_PERIOD_MS
	 * converts using the project's actual configTICK_RATE_HZ rather than
	 * assuming 1000Hz. Wraps when the tick count wraps; j30d.c's use of
	 * this is wraparound-safe (unsigned subtraction). */
	return (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void fr_return_to_iic(void *ctx)
{
	j30d_fr_ctx_t *c = (j30d_fr_ctx_t *)ctx;

	/* Disarm before SDA returns to the I2C peripheral, or every byte on the
	 * bus becomes an edge on this line. */
	HAL_NVIC_DisableIRQ(J30D_FR_INT_IRQn);

	HAL_GPIO_WritePin(J30D_FR_SDA_GPIO_Port, J30D_FR_SDA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(J30D_FR_SCL_GPIO_Port, J30D_FR_SCL_Pin, GPIO_PIN_SET);
	HAL_I2C_MspDeInit(c->hi2c);

	GPIO_InitTypeDef gpio_init = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();

	gpio_init.Pin  = J30D_FR_SDA_Pin;
	gpio_init.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(J30D_FR_SDA_GPIO_Port, &gpio_init);

	gpio_init.Pin  = J30D_FR_SCL_Pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(J30D_FR_SCL_GPIO_Port, &gpio_init);

	HAL_GPIO_WritePin(J30D_FR_SCL_GPIO_Port, J30D_FR_SCL_Pin, GPIO_PIN_RESET);
	vTaskDelay(pdMS_TO_TICKS(J30D_FR_SCL_LOW_MS));
	HAL_GPIO_WritePin(J30D_FR_SCL_GPIO_Port, J30D_FR_SCL_Pin, GPIO_PIN_SET);

	HAL_I2C_MspInit(c->hi2c);
#if J30D_FR_SETTLE_MS > 0
	vTaskDelay(pdMS_TO_TICKS(J30D_FR_SETTLE_MS));
#endif
}

/* Blocks up to J30D_FR_MUTEX_WAIT_MS and reports whether the take succeeded.
 * A timed-out take must return false: j30d.c turns that into
 * J30D_ERR_TIMEOUT without touching the bus. Returning true anyway voids the
 * mutual exclusion and releases a semaphore this task never held. */
static bool fr_lock(void *ctx)
{
	j30d_fr_ctx_t *c = (j30d_fr_ctx_t *)ctx;
	return xSemaphoreTake(c->mutex, pdMS_TO_TICKS(J30D_FR_MUTEX_WAIT_MS)) == pdTRUE;
}

static void fr_unlock(void *ctx)
{
	j30d_fr_ctx_t *c = (j30d_fr_ctx_t *)ctx;
	xSemaphoreGive(c->mutex);
}

static j30d_fr_ctx_t j30d_fr_ctx_default = { .hi2c = &hi2c1, .mutex = NULL };

/*
 * Fills `platform` in ready to hand to j30d_init(). Creates the mutex on
 * first call. Call once at startup, before any task touches the handle.
 *
 * If you have multiple J30D sensors (or other I2C devices) on the SAME
 * physical bus, do not call this twice and use two different mutexes --
 * instead share one j30d_fr_ctx_t.mutex (and one hi2c) across all of them.
 */
/* Hands SDA to the device as its INT output and arms the pin interrupt.
 * Called only in interrupt mode. fr_return_to_iic() reclaims both pins. */
static void fr_release_to_int(void *ctx)
{
	j30d_fr_ctx_t *c = (j30d_fr_ctx_t *)ctx;

	HAL_GPIO_WritePin(J30D_FR_SDA_GPIO_Port, J30D_FR_SDA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(J30D_FR_SCL_GPIO_Port, J30D_FR_SCL_Pin, GPIO_PIN_SET);
	HAL_I2C_MspDeInit(c->hi2c);

	GPIO_InitTypeDef gpio_init = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Falling edge: the device holds the line high and pulls it low. It
	 * drives the line, so no pull resistor is needed. */
	gpio_init.Pin  = J30D_FR_SDA_Pin;
	gpio_init.Mode = GPIO_MODE_IT_FALLING;
	gpio_init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(J30D_FR_SDA_GPIO_Port, &gpio_init);

	/* SCL stays ours: it is how the device is woken. */
	gpio_init.Pin  = J30D_FR_SCL_Pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(J30D_FR_SCL_GPIO_Port, &gpio_init);

	HAL_NVIC_SetPriority(J30D_FR_INT_IRQn, J30D_FR_INT_PRIO, 0);
	HAL_NVIC_EnableIRQ(J30D_FR_INT_IRQn);
}

void j30d_platform_stm32_freertos_init(j30d_platform_t *platform)
{
	if (j30d_fr_ctx_default.mutex == NULL)
	{
		j30d_fr_ctx_default.mutex = xSemaphoreCreateMutex();
	}

	memset(platform, 0, sizeof(*platform));
	platform->i2c_read      = fr_i2c_read;
	platform->i2c_write     = fr_i2c_write;
	platform->delay_ms      = fr_delay_ms;
	platform->return_to_iic = fr_return_to_iic;
	platform->lock          = fr_lock;
	platform->unlock        = fr_unlock;
	platform->now_ms        = fr_now_ms;
	platform->release_to_int = fr_release_to_int;
	platform->ctx           = &j30d_fr_ctx_default;
}
