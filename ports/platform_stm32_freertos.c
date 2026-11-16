/*
 * platform_stm32_freertos.c
 *
 * Reference j30d_platform_t implementation for STM32 (STM32Cube HAL) under
 * FreeRTOS. Same HAL calls as platform_stm32_baremetal.c, but:
 *   - delay_ms uses vTaskDelay() instead of a busy-wait, so other tasks run
 *     during the (several 10s of ms) delays in j30d_init()'s handshake.
 *   - lock/unlock wrap a FreeRTOS mutex, so two tasks calling into the SAME
 *     j30d_handle_t are serialized instead of corrupting each other.
 *   - each HAL transaction runs with the scheduler suspended. The polled HAL
 *     calls hold SCL low whenever the task is not running -- between the
 *     register byte and the read, or between received bytes -- and a task
 *     preempted there for about 10 ms sends the device the return-to-IIC
 *     signal, which stops a converting device. Interrupts stay enabled, so
 *     the HAL's own timeouts still work.
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

/* SCL must be held low for more than 10ms to return the device to I2C idle.
 * The exact threshold varies from unit to unit, so 15ms leaves margin. The
 * settle delay afterwards is 0, which compiles the call out --
 * fr_ticks_at_least(0) is still one tick. */
#define J30D_FR_SCL_LOW_MS       15u
#define J30D_FR_SETTLE_MS        0u

/* fr_now_ms() and fr_ticks_at_least() need a whole number of milliseconds
 * per tick: configTICK_RATE_HZ must divide 1000 (1000, 500, 200, 100, ...).
 * At a faster rate portTICK_PERIOD_MS is 0, fr_now_ms() never advances, and
 * j30d_get_data() paces every call out. */
typedef char j30d_fr_tick_rate_must_divide_1000[(1000u % configTICK_RATE_HZ) == 0u ? 1 : -1];

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
#define J30D_FR_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()   /* the port SCL and SDA are on */

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
		vTaskSuspendAll();   /* no preemption mid-transaction -- see the top of this file */
		status = HAL_I2C_Mem_Read(c->hi2c, J30D_FR_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
		                           buf, (uint16_t)len, J30D_FR_I2C_TIMEOUT);
		(void)xTaskResumeAll();
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
		vTaskSuspendAll();
		status = HAL_I2C_Mem_Write(c->hi2c, J30D_FR_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
		                            (uint8_t *)buf, (uint16_t)len, J30D_FR_I2C_TIMEOUT);
		(void)xTaskResumeAll();
		if (status == HAL_OK)
		{
			break;
		}
	}
	return hal_status_to_j30d(status);
}

/* vTaskDelay(n) returns between n-1 and n tick periods later, since the
 * first period is whatever remains of the current one. A delay of at least
 * `ms` therefore needs the tick count rounded up, plus one. */
static TickType_t fr_ticks_at_least(uint32_t ms)
{
	const uint32_t tick_ms = (uint32_t)portTICK_PERIOD_MS;
	return (TickType_t)((ms + tick_ms - 1u) / tick_ms + 1u);
}

static void fr_delay_ms(void *ctx, uint32_t ms)
{
	(void)ctx;
	vTaskDelay(fr_ticks_at_least(ms));
}

static uint32_t fr_now_ms(void *ctx)
{
	(void)ctx;
	/* xTaskGetTickCount() is in RTOS ticks, not ms. With a whole number of
	 * ms per tick (checked above) the product wraps in step with the tick
	 * count, which keeps j30d.c's unsigned subtraction exact. */
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
	J30D_FR_GPIO_CLK_ENABLE();

	gpio_init.Pin  = J30D_FR_SDA_Pin;
	gpio_init.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(J30D_FR_SDA_GPIO_Port, &gpio_init);

	gpio_init.Pin  = J30D_FR_SCL_Pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(J30D_FR_SCL_GPIO_Port, &gpio_init);

	HAL_GPIO_WritePin(J30D_FR_SCL_GPIO_Port, J30D_FR_SCL_Pin, GPIO_PIN_RESET);
	vTaskDelay(fr_ticks_at_least(J30D_FR_SCL_LOW_MS));
	HAL_GPIO_WritePin(J30D_FR_SCL_GPIO_Port, J30D_FR_SCL_Pin, GPIO_PIN_SET);

	HAL_I2C_MspInit(c->hi2c);
#if J30D_FR_SETTLE_MS > 0
	vTaskDelay(fr_ticks_at_least(J30D_FR_SETTLE_MS));
#endif
}

/* Blocks up to J30D_FR_MUTEX_WAIT_MS and reports whether the take succeeded.
 * A timed-out take must return false: j30d.c turns that into
 * J30D_ERR_TIMEOUT without touching the bus. Returning true anyway voids the
 * mutual exclusion and releases a semaphore this task never held. A mutex
 * that could not be created reports false the same way. */
static bool fr_lock(void *ctx)
{
	j30d_fr_ctx_t *c = (j30d_fr_ctx_t *)ctx;
	if (c->mutex == NULL)
	{
		return false;
	}
	return xSemaphoreTake(c->mutex, pdMS_TO_TICKS(J30D_FR_MUTEX_WAIT_MS)) == pdTRUE;
}

static void fr_unlock(void *ctx)
{
	j30d_fr_ctx_t *c = (j30d_fr_ctx_t *)ctx;
	xSemaphoreGive(c->mutex);
}

static j30d_fr_ctx_t j30d_fr_ctx_default = { .hi2c = &hi2c1, .mutex = NULL };

/* Hands SDA to the device as its INT output and arms the pin interrupt.
 * Called only in interrupt mode. fr_return_to_iic() reclaims both pins. */
static void fr_release_to_int(void *ctx)
{
	j30d_fr_ctx_t *c = (j30d_fr_ctx_t *)ctx;

	HAL_GPIO_WritePin(J30D_FR_SDA_GPIO_Port, J30D_FR_SDA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(J30D_FR_SCL_GPIO_Port, J30D_FR_SCL_Pin, GPIO_PIN_SET);
	HAL_I2C_MspDeInit(c->hi2c);

	GPIO_InitTypeDef gpio_init = {0};
	J30D_FR_GPIO_CLK_ENABLE();

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

/*
 * Fills `platform` in ready to hand to j30d_init(). Creates the mutex on
 * first call. Call once at startup, before any task touches the handle.
 * If the FreeRTOS heap cannot hold the mutex, configASSERT() fires; with
 * asserts compiled out, every j30d_* call returns J30D_ERR_TIMEOUT.
 *
 * If you have multiple J30D sensors (or other I2C devices) on the SAME
 * physical bus, do not call this twice and use two different mutexes --
 * instead share one j30d_fr_ctx_t.mutex (and one hi2c) across all of them.
 */
void j30d_platform_stm32_freertos_init(j30d_platform_t *platform)
{
	if (j30d_fr_ctx_default.mutex == NULL)
	{
		j30d_fr_ctx_default.mutex = xSemaphoreCreateMutex();
		configASSERT(j30d_fr_ctx_default.mutex != NULL);
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
